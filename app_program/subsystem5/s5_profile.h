#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#include "s5_range.h"
#include "s5_servo_config.h"
#include <stdint.h>

#define S5_FINE_STEP 5u
#define S5_SCAN_POINTS ((S5_SCAN_MAX_ANGLE - S5_SCAN_MIN_ANGLE) / S5_FINE_STEP + 1u)
#define S5_COARSE_STEP 6u
#define S5_COARSE_FIRST ((S5_SCAN_MAX_ANGLE - S5_MAX_ANGLE) / S5_FINE_STEP)
#define S5_COARSE_LAST ((S5_SCAN_MAX_ANGLE - S5_MIN_ANGLE) / S5_FINE_STEP)
#define S5_CENTRE_INDEX ((S5_SCAN_MAX_ANGLE - S5_CENTRE_ANGLE) / S5_FINE_STEP)
#define S5_NO_TARGET S5_SCAN_POINTS
#define S5_UNKNOWN_WIDTH 2047u
#define S5_MAX_PROJECTED_FRONT_MM 511u
#define S5_PROFILE_TOLERANCE_MM 100u

/* The reported offset uses the standard sweep so the wire format stays fixed. */
static inline unsigned s5_servo_angle(unsigned scan_offset_deg)
{
    return S5_MAX_ANGLE - scan_offset_deg;
}

static inline unsigned s5_scan_bearing(unsigned index)
{
    return S5_SCAN_MAX_ANGLE - index * S5_FINE_STEP;
}

typedef enum {
    S5_WIDTH_OK, S5_WIDTH_FACE, S5_WIDTH_LEFT_EDGE, S5_WIDTH_RIGHT_EDGE,
    S5_WIDTH_BOTH_EDGES, S5_WIDTH_LEFT_ECHO, S5_WIDTH_RIGHT_ECHO, S5_WIDTH_BOTH_ECHOES,
} S5_WIDTH_STATUS;

typedef struct {
    unsigned nearest_mm : 12;
    unsigned width_mm : 11; /* Flat-face estimate from the centre and edge echoes. */
    unsigned front_mm : 9; /* Nearest longitudinal projection, capped at 511 mm. */
    unsigned left_mm : 12;
    unsigned right_mm : 12;
    unsigned angle_offset_deg : 7;
    unsigned left_path_mm : 12;
    unsigned right_path_mm : 12;
    unsigned width_status : 3;
    unsigned left_clipped : 1;
    unsigned right_clipped : 1;
    signed left_edge_mm : 13; /* Outer sweep bounds used by the motion planner. */
    signed right_edge_mm : 13;
    unsigned left_edge_valid : 1;
    unsigned right_edge_valid : 1;
} S5_PROFILE;

/* Index zero is the configured left endpoint, decreasing 5 degrees per sample. */
typedef struct {
    uint8_t bytes[(S5_SCAN_POINTS * 12u + 7u) / 8u];
} S5_SCAN;

/* Magnitudes of each edge bracket, in degrees from straight ahead. */
typedef struct {
    unsigned face_mm;
    unsigned left_hit_deg, left_background_deg;
    unsigned right_hit_deg, right_background_deg;
    unsigned left_hit_mm, right_hit_mm;
    bool off_axis, on_rays;
} S5_WIDTH_SCAN;

typedef struct {
    unsigned left_excess_tenths, right_excess_tenths;
    unsigned min_face_mm, max_face_mm;
} S5_WIDTH_CALIBRATION;

void s5_scan_set(S5_SCAN *scan, unsigned index, unsigned range);
unsigned s5_scan_get(const S5_SCAN *scan, unsigned index);
unsigned s5_scan_target(const S5_SCAN *scan);
bool s5_same_obstacle(unsigned range, unsigned reference);
bool s5_background_beyond(unsigned range, unsigned reference);
/* Confirm a background bearing from the full fine-scan sample window. */
unsigned s5_edge_background(const unsigned *ranges, unsigned count, unsigned reference);
/* Optionally reports the angular edge brackets of a measurable flat face. */
S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor,
                      S5_WIDTH_SCAN *width_scan);
/* Lateral span. Bearings are servo angle minus 90 degrees, in tenths. */
unsigned s5_width_from_corners(unsigned left_mm, int left_bearing_tenths,
                               unsigned right_mm, int right_bearing_tenths);
unsigned s5_refined_ray_width(const S5_WIDTH_SCAN *scan);
unsigned s5_calibrated_width(const S5_WIDTH_SCAN *scan, const S5_WIDTH_CALIBRATION *calibration);
#endif
