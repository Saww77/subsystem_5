"""Wire compatibility, freshness and reconnect regression tests."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import CommandUnavailable, Dashboard
from mqtt_protocol import decode_telemetry

def motion_packet(faults=0,left=30,right=24):
    value=1|(1<<2)|(faults<<5)|(left<<9)|(right<<18)|(42<<27)|(35<<34)|(100<<41)|(200<<54)
    return value.to_bytes(9,'little')

class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.d=Dashboard('192.0.2.1',clock=lambda:self.now)
        self.d.broker_connected=self.d.monitor_ready=True
    def send(self,kind,data,retained=False):
        self.d.on_message(None,None,SimpleNamespace(topic=self.d.root+'/'+kind,payload=data,retain=retained))
    def test_motion_and_faults(self):
        self.send('motion',motion_packet(4))
        item=self.d.snapshot()['telemetry']['motion']
        self.assertTrue(item['fresh'])
        self.assertEqual(item['data']['left_rpm'],30)
        self.assertEqual(item['data']['right_pwm'],35)
        self.assertEqual(item['data']['left_progress'],100)
        self.assertEqual(item['data']['fault_labels'],['Wheel stalled'])
        self.send('vehicle',bytes([5|(1<<3)]))
        self.assertEqual(self.d.snapshot()['telemetry']['vehicle']['data']['state'],'FAULT')
    def test_all_wire_payloads(self):
        self.assertEqual(decode_telemetry('line',bytes([5|(2<<3)]))['barcode'],'B')
        self.assertEqual(decode_telemetry('terrain',(125|(2<<11)).to_bytes(2,'little')),dict(height_mm=125,phase=2))
        obstacle=(500|(80<<12)|(200<<23)|(300<<35)).to_bytes(6,'little')
        self.assertEqual(decode_telemetry('obstacle',obstacle),dict(nearest_mm=500,width_mm=80,left_mm=200,right_mm=300))
    def test_reverse_phase_uses_existing_avoidance_byte(self):
        self.assertEqual(decode_telemetry('avoidance', b'\x0e'),
                         dict(phase=14, phase_label='Reversing to rescan', side=None))
        with self.assertRaises(ValueError):
            decode_telemetry('avoidance', b'\x0f')

    def test_extended_obstacle_sample_and_freshness(self):
        packet = bytes.fromhex('8c f0 ff ff 7f 94 06')
        self.send('obstacle', packet, retained=True)
        self.assertIsNone(self.d.snapshot()['telemetry']['obstacle']['data'])
        self.send('obstacle', packet)
        sample = self.d.snapshot()['telemetry']['obstacle']
        self.assertTrue(sample['fresh'])
        self.assertEqual(sample['data'], dict(nearest_mm=140, width_mm=None,
                                              left_mm=None, right_mm=655, angle_deg=43))
        self.now += 3
        for invalid in (packet[:-1], (int.from_bytes(packet, 'little') | (1 << 55)).to_bytes(7, 'little'),
                        (121 << 47).to_bytes(7, 'little')):
            self.send('obstacle', invalid)
        self.assertFalse(self.d.snapshot()['telemetry']['obstacle']['fresh'])
        self.assertEqual(self.d.snapshot()['telemetry']['obstacle']['data'], sample['data'])

    def test_each_topic_has_independent_freshness(self):
        self.send('motion',motion_packet())
        self.now+=3
        self.send('status',b'\x07')
        self.send('vehicle',b'\x01')
        state=self.d.snapshot()
        self.assertTrue(state['pico']['fresh'])
        self.assertFalse(state['telemetry']['motion']['fresh'])
        self.assertTrue(state['telemetry']['vehicle']['fresh'])
        self.assertIsNone(state['telemetry']['terrain']['data'])
    def test_retained_and_bad_payloads_do_not_refresh(self):
        self.send('motion',motion_packet(),True)
        self.assertIsNone(self.d.snapshot()['telemetry']['motion']['data'])
        self.send('motion',motion_packet());self.now+=3
        for packet in (b'',b'\x00',bytes([255])*9):self.send('motion',packet)
        self.assertFalse(self.d.snapshot()['telemetry']['motion']['fresh'])
        self.assertEqual(self.d.snapshot()['telemetry']['motion']['data']['left_rpm'],30)
    def test_reconnect_needs_new_sample(self):
        self.send('motion',motion_packet())
        self.d.on_disconnect(None,None,None,None,None)
        self.d.broker_connected=self.d.monitor_ready=True
        self.assertFalse(self.d.snapshot()['telemetry']['motion']['fresh'])
        self.send('motion',motion_packet())
        self.assertTrue(self.d.snapshot()['telemetry']['motion']['fresh'])
    def test_padding_and_range_rejection(self):
        packets=[('vehicle',b'\x07'),('line',b'\xc0'),('line',b'\x28'),
                 ('terrain',(2001).to_bytes(2,'little')),('terrain',b'\x00\xc0'),
                 ('obstacle',(4001).to_bytes(6,'little')),('obstacle',(1<<47).to_bytes(6,'little')),
                 ('motion',(1<<67).to_bytes(9,'little')),('motion',(101<<27).to_bytes(9,'little')),
                 ('avoidance',b'\x0f'),('avoidance',b'\x31'),('avoidance',b'\x41')]
        for kind,payload in packets:
            with self.subTest(kind=kind,payload=payload):
                with self.assertRaises(ValueError):decode_telemetry(kind,payload)
    def test_history_and_events_are_bounded(self):
        for _ in range(200):self.send('motion',motion_packet())
        self.assertEqual(len(self.d.snapshot()['motion_history']),120)
        self.assertLessEqual(len(self.d.snapshot()['events']),40)
    def test_controller_explains_start_block_without_weakening_guards(self):
        packet=bytearray(25);packet[0]=1;packet[3]=7;packet[4]=1|2|4|64
        packet[6:8]=(900).to_bytes(2,'little');packet[10:12]=(850).to_bytes(2,'little')
        packet[24]=7
        self.send('status',b'\x07')
        self.send('controller',packet)
        state=self.d.snapshot()
        self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['start_block_label'],'Junction / line stop active')
        packet[3]=0;packet[4]=1|2|64;self.send('controller',packet)
        self.assertTrue(self.d.snapshot()['start_allowed'])
        packet[0]=2
        with self.assertRaises(ValueError):decode_telemetry('controller',packet)

    def test_obstacle_interlock_distinguishes_live_echo_fault(self):
        packet = bytearray(31)
        packet[0], packet[3], packet[4] = 2, 8, 1 | 2 | 8 | 64
        self.send('status', b'\x07')
        self.send('controller', packet)
        self.send('vehicle', b'\x40')
        state = self.d.snapshot()
        self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['start_block_label'],
                         'Ultrasonic echo unavailable')
        self.d.client = SimpleNamespace()
        with self.assertRaisesRegex(CommandUnavailable, 'Ultrasonic echo unavailable'):
            self.d.send('start')

        self.now += 3
        self.send('controller', packet)
        state = self.d.snapshot()
        self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['start_block_label'],
                         'Obstacle interlock active')

        packet[3], packet[4] = 0, 1 | 2 | 64
        self.send('vehicle', b'\x00')
        self.send('controller', packet)
        self.assertTrue(self.d.snapshot()['start_allowed'])

    def test_timing_diagnostics_and_stopped_reset(self):
        packet=bytearray(31);packet[0]=2;packet[1]=5;packet[3]=3;packet[4]=1|2|64;packet[5]=2
        packet[25:29]=(12000).to_bytes(4,'little');packet[29:31]=(15000).to_bytes(2,'little')
        motor=bytearray(motion_packet(2));motor[0]&=~3
        self.send('status',b'\x07');self.send('motion',motor);self.send('controller',packet)
        state=self.d.snapshot()
        self.assertTrue(state['reset_allowed']);self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['timing_gap_us'],12000)
        self.assertEqual(state['telemetry']['controller']['data']['timing_limit_us'],15000)
        packet[4]|=128;self.send('controller',packet)
        self.assertFalse(self.d.snapshot()['reset_allowed'])
        packet[0]=1
        with self.assertRaises(ValueError):decode_telemetry('controller',packet)
        self.send('ack',b'\x04');self.assertEqual(self.d.snapshot()['last_ack']['command'],'reset')
        self.send('ack',b'\x06');self.assertTrue(self.d.snapshot()['last_ack']['rejected'])

    def test_stopped_detour_without_motor_fault_can_reset(self):
        packet = bytearray(31)
        packet[0:5] = bytes((2, 5, 6, 8, 1 | 2 | 64))
        self.send('controller', packet)
        self.send('avoidance', b'\x0c\x1d' + (219).to_bytes(2, 'little'))
        detour = self.d.snapshot()['telemetry']['avoidance']['data']
        self.assertEqual(detour['failure'], 'A near echo persisted at the left sweep limit')
        self.assertEqual(detour['forward_mm'], 219)
        self.assertTrue(self.d.snapshot()['reset_allowed'])
        self.send('motion', motion_packet())
        # Only the Pico decides whether the wheels have stopped during Reset.
        self.assertTrue(self.d.snapshot()['reset_allowed'])
        self.assertFalse(self.d.snapshot()['start_allowed'])
        packet[4] |= 32
        self.send('controller', packet)
        self.assertFalse(self.d.snapshot()['reset_allowed'])
        self.assertIn('current command', self.d.snapshot()['reset_block'])
        self.send('ack', b'\x0c')
        self.assertTrue(self.d.snapshot()['last_ack']['complete'])
        self.assertFalse(self.d.snapshot()['last_ack']['rejected'])
        self.send('ack', b'\x0e')
        self.assertTrue(self.d.snapshot()['last_ack']['rejected'])

    def test_saved_sweep_is_distinct_from_current_range_and_failure(self):
        scan = (208 | (100 << 12) | (4094 << 23) | (390 << 35) | (60 << 47) | (1 << 54))
        self.send('obstacle', scan.to_bytes(7, 'little'))
        self.send('avoidance', b'\x0c\x15' + (606).to_bytes(2, 'little'))
        self.send('avoidance', b'\x0c\x15' + (611).to_bytes(2, 'little'))
        state = self.d.snapshot()
        saved = state['telemetry']['obstacle']['data']
        detour = state['telemetry']['avoidance']['data']
        self.assertTrue(saved['saved_profile'] and saved['left_open'])
        self.assertEqual((saved['nearest_mm'], saved['width_mm'], detour['forward_mm']), (208, 100, 611))
        self.assertIn('departure path', detour['failure'])
        self.assertEqual(sum(e['kind'] == 'vehicle' for e in state['events']), 1)

    def test_active_detour_is_reported_as_progress(self):
        packet = bytearray(31)
        packet[:5] = bytes((2, 3, 6, 8, 1 | 2 | 64))
        self.send('controller', packet)
        self.send('avoidance', b'\x12')
        data = self.d.snapshot()['telemetry']['controller']['data']
        self.assertTrue(data['detour_active'])
        self.assertEqual(data['start_block_label'], 'Automatic obstacle avoidance')
        self.assertIn('Turning outward', data['start_hint'])
        self.assertNotIn('Move nearby objects away', data['start_hint'])

    def test_rejected_start_reports_actual_detour_phase(self):
        packet = bytearray(31)
        packet[:5] = bytes((2, 5, 6, 8, 1 | 2 | 64))
        packet[24] = 8
        self.send('controller', packet)
        self.send('avoidance', bytes((12, 0x14)) + (190).to_bytes(2, 'little'))
        data = self.d.snapshot()['telemetry']['controller']['data']
        self.assertEqual(data['last_reject_label'], 'Detour stopped')
        self.assertIn('Too little room', data['detour_failure'])
        self.assertIn('during Checking side clearance', data['last_reject_hint'])
        self.assertNotIn('Move nearby objects away', data['last_reject_hint'])
        self.assertEqual(self.d.telemetry['controller']['data']['last_reject_label'], 'Obstacle interlock active')
        # An expired failure must not explain a fresh unrelated rejection.
        self.now += 3
        self.send('controller', packet)
        data = self.d.snapshot()['telemetry']['controller']['data']
        self.assertEqual(data['last_reject_label'], 'Obstacle interlock active')
        self.assertNotIn('detour_failure', data)

    def test_packet_mismatch_is_visible_and_clears_on_valid_data(self):
        self.send('obstacle', b'\x00' * 9)
        state = self.d.snapshot()
        self.assertEqual(state['dashboard_release'], 'Clearance v3')
        self.assertIn('received 9 bytes; expected 6 or 7 bytes', state['protocol_errors']['obstacle'])
        self.assertIn('same revision and topic root', state['protocol_errors']['obstacle'])
        self.assertFalse(state['telemetry']['obstacle']['fresh'])
        errors = len([item for item in state['events'] if item['kind'] == 'error'])
        self.send('obstacle', b'\x00' * 9)
        self.assertEqual(len([item for item in self.d.snapshot()['events'] if item['kind'] == 'error']), errors)
        packet = (220 | (2047 << 12) | (4094 << 23) | (4095 << 35) | (60 << 47) | (1 << 54))
        self.send('obstacle', packet.to_bytes(7, 'little'))
        self.assertNotIn('obstacle', self.d.snapshot()['protocol_errors'])
        self.assertTrue(self.d.snapshot()['telemetry']['obstacle']['fresh'])

    def test_stall_explains_power_and_encoder_checks(self):
        packet = bytearray(31)
        packet[:6] = bytes((2, 5, 4, 3, 1 | 2 | 64, 4))
        self.send('controller', packet)
        data = self.d.snapshot()['telemetry']['controller']['data']
        self.assertIn('no encoder pulses', data['motor_fault_hint'])
        self.assertIn('GP2/GP3', data['start_hint'])
        self.assertIn('motor supply', data['start_hint'])
        self.assertFalse(self.d.snapshot()['start_allowed'])

    def test_subscribes_every_topic(self):
        result=[]
        client=SimpleNamespace(subscribe=lambda topics:(result.extend(topics) or (0,1)))
        self.d.on_connect(client,None,None,SimpleNamespace(is_failure=False),None)
        self.assertEqual({topic.rsplit('/',1)[-1] for topic,qos in result},
                         {'status','ack','motion','line','terrain','obstacle','vehicle','controller','avoidance'})

if __name__=='__main__':unittest.main()
