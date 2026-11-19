#include "s5_profile.h"

#define MAX_WIDTH_MM 2000
#define MAX_EDGE_BRACKET_STEPS 2u
#define MIN_DISTANT_ECHOES 3u
#define WIDTH_RAY_TOLERANCE_MM 8u
#define WIDTH_MIN_EDGE_RISE_MM 3u
#define WIDTH_FACE_TOLERANCE_MM 12u
#define WIDTH_STABLE_CORNER_MM 6u
#define WIDTH_EDGE_CANDIDATES 3u
#define WIDTH_MAX_SUPPORT_GAP 3u
#define TENTHS_PER_STEP (S5_FINE_STEP * 10u)
#define HALF_ANGLE_STEP_TENTHS 20u
#define COSINE_SCALE (32768 * HALF_ANGLE_STEP_TENTHS / 2u)

_Static_assert(S5_CLEAR_MM <= S5_MAX_PROJECTED_FRONT_MM, "Front projection must cover the obstacle window");

/* Sweep sine and cosine tables in 5-degree steps, scaled by 1024. */
static const unsigned short sine[] = {
    0, 89, 178, 265, 350, 433, 512, 587, 658, 724, 784, 839, 887, 928, 962, 989,
};
static const unsigned short cosine[] = {
    1024, 1020, 1008, 989, 962, 928, 886, 838, 784, 724, 658, 587, 512, 433, 350, 265,
};

static unsigned angle_steps(unsigned index)
{
    return index < S5_CENTRE_INDEX ? S5_CENTRE_INDEX - index : index - S5_CENTRE_INDEX;
}

static unsigned project_forward_mm(unsigned index, unsigned range_mm)
{
    return range_mm * cosine[angle_steps(index)] / 1024u;
}

static int edge_bound_mm(unsigned bearing, unsigned obstacle_depth)
{
    unsigned angle = angle_steps(bearing);
    int projection = (obstacle_depth * sine[angle] + cosine[angle] - 1u) / cosine[angle];
    return bearing < S5_CENTRE_INDEX ? -projection : projection;
}

/* Sine of 0..90 degrees at one-degree steps, scaled by 32768. */
static const uint16_t width_sine[] = {
    0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,
    5690, 6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668,
    11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,
    16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,
    21063, 21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730,
    25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,
    28378, 28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,
    30792, 30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,
    32270, 32365, 32449, 32524, 32588, 32643, 32688, 32723, 32748, 32763,
    32768,
};

static int cosine_tenths(unsigned angle_tenths)
{
    /* Interpolate sine at half the angle to retain accuracy close to zero.
     * cos(angle) = 1 - 2 sin^2(angle / 2). */
    unsigned step = angle_tenths / HALF_ANGLE_STEP_TENTHS;
    unsigned fraction = angle_tenths % HALF_ANGLE_STEP_TENTHS;
    unsigned half_sine = width_sine[step] * HALF_ANGLE_STEP_TENTHS;
    if (fraction) half_sine += (width_sine[step + 1u] - width_sine[step]) * fraction;
    int reduction = (int)(((uint64_t)half_sine * half_sine + COSINE_SCALE) / (2u * COSINE_SCALE));
    return (int)COSINE_SCALE - reduction;
}

/* The module reports half the Tx-to-target-to-Rx path. For a ray
 * through the midpoint of the rotating pair, remove the half-baseline
 * in quadrature. Work in half millimetres before rounding. */
static unsigned width_ray_range(unsigned echo_mm)
{
    unsigned path = 2u * echo_mm;
    unsigned spacing = S5_TRANSDUCER_SPACING_MM;
    if (path <= spacing) return 0;
    if (!spacing) return echo_mm;

    unsigned square = path * path - spacing * spacing;
    unsigned low = 0, high = path;
    while (low < high) {
        unsigned midpoint = (low + high + 1u) / 2u;
        if (midpoint * midpoint <= square) low = midpoint;
        else high = midpoint - 1u;
    }
    return (low + 1u) / 2u;
}

/* Positive bearings and lateral coordinates point to the car's left. */
static int64_t lateral_scaled(unsigned range_mm, int bearing_tenths)
{
    unsigned angle = (unsigned)(bearing_tenths < 0 ? -bearing_tenths : bearing_tenths);
    int64_t projection = (int64_t)range_mm * cosine_tenths(900u - angle);
    return bearing_tenths < 0 ? -projection : projection;
}

static unsigned lateral_width(int64_t left, int64_t right)
{
    if (left <= right) return S5_UNKNOWN_WIDTH;
    unsigned width = ((uint64_t)(left - right) + COSINE_SCALE / 2u) / COSINE_SCALE;
    return width > 0 && width <= MAX_WIDTH_MM ? width : S5_UNKNOWN_WIDTH;
}

unsigned s5_width_from_corners(unsigned left_mm, int left_bearing_tenths,
                               unsigned right_mm, int right_bearing_tenths)
{
    if (!s5_range_measured(left_mm) || !s5_range_measured(right_mm) ||
        left_bearing_tenths > 900 || left_bearing_tenths < -900 ||
        right_bearing_tenths > 900 || right_bearing_tenths < -900 ||
        left_bearing_tenths <= right_bearing_tenths)
        return S5_UNKNOWN_WIDTH;

    unsigned left_range = width_ray_range(left_mm), right_range = width_ray_range(right_mm);
    if (!left_range || !right_range) return S5_UNKNOWN_WIDTH;
    return lateral_width(lateral_scaled(left_range, left_bearing_tenths),
                         lateral_scaled(right_range, right_bearing_tenths));
}

unsigned s5_refined_ray_width(const S5_WIDTH_SCAN *scan)
{
    if (!scan->on_rays || !scan->face_mm || !s5_range_measured(scan->left_hit_mm) ||
        !s5_range_measured(scan->right_hit_mm) ||
        scan->left_hit_deg > S5_SCAN_MAX_ANGLE - S5_CENTRE_ANGLE ||
        scan->right_hit_deg > S5_CENTRE_ANGLE - S5_SCAN_MIN_ANGLE)
        return S5_UNKNOWN_WIDTH;
    unsigned left_bearing = scan->left_hit_deg * 10u;
    unsigned right_bearing = scan->right_hit_deg * 10u;
    int left_depth = (int)((uint64_t)width_ray_range(scan->left_hit_mm) *
                          (unsigned)cosine_tenths(left_bearing) / COSINE_SCALE);
    int right_depth = (int)((uint64_t)width_ray_range(scan->right_hit_mm) *
                           (unsigned)cosine_tenths(right_bearing) / COSINE_SCALE);
    int face = (int)width_ray_range(scan->face_mm);
    if (!face) return S5_UNKNOWN_WIDTH;
    if (left_depth < face - (int)WIDTH_RAY_TOLERANCE_MM || left_depth > face + (int)WIDTH_RAY_TOLERANCE_MM ||
        right_depth < face - (int)WIDTH_RAY_TOLERANCE_MM || right_depth > face + (int)WIDTH_RAY_TOLERANCE_MM)
        return S5_UNKNOWN_WIDTH;
    return s5_width_from_corners(scan->left_hit_mm, (int)left_bearing,
                                 scan->right_hit_mm, -(int)right_bearing);
}

unsigned s5_calibrated_width(const S5_WIDTH_SCAN *scan, const S5_WIDTH_CALIBRATION *calibration)
{
    if (!scan->off_axis || !calibration->min_face_mm || scan->face_mm < calibration->min_face_mm ||
        scan->face_mm > calibration->max_face_mm || scan->face_mm > S5_CLEAR_MM ||
        scan->left_hit_deg >= scan->left_background_deg || scan->right_hit_deg >= scan->right_background_deg ||
        scan->left_background_deg > S5_SCAN_MAX_ANGLE - S5_CENTRE_ANGLE ||
        scan->right_background_deg > S5_CENTRE_ANGLE - S5_SCAN_MIN_ANGLE ||
        scan->left_background_deg - scan->left_hit_deg > MAX_EDGE_BRACKET_STEPS * S5_FINE_STEP ||
        scan->right_background_deg - scan->right_hit_deg > MAX_EDGE_BRACKET_STEPS * S5_FINE_STEP)
        return S5_UNKNOWN_WIDTH;

    /* Midpoint of each echo transition, expressed in tenths of a degree. */
    unsigned left = (scan->left_hit_deg + scan->left_background_deg) * 5u;
    unsigned right = (scan->right_hit_deg + scan->right_background_deg) * 5u;
    if (left <= calibration->left_excess_tenths || right <= calibration->right_excess_tenths)
        return S5_UNKNOWN_WIDTH;
    left -= calibration->left_excess_tenths;
    right -= calibration->right_excess_tenths;
    unsigned face = width_ray_range(scan->face_mm);
    if (!face || !s5_range_measured(scan->left_hit_mm) || !s5_range_measured(scan->right_hit_mm) ||
        !width_ray_range(scan->left_hit_mm) || !width_ray_range(scan->right_hit_mm))
        return S5_UNKNOWN_WIDTH;

    /* Corrected edge bearings intersect the face plane. Off-axis hit
     * ranges belong to the measured echoes, so do not reuse them on these rays. */
    int64_t left_edge = lateral_scaled(face, (int)left) * COSINE_SCALE / cosine_tenths(left);
    int64_t right_edge = lateral_scaled(face, -(int)right) * COSINE_SCALE / cosine_tenths(right);
    return lateral_width(left_edge, right_edge);
}

void s5_scan_set(S5_SCAN *scan, unsigned index, unsigned range)
{
    unsigned offset = index * 3u / 2u;
    if (index & 1u) {
        scan->bytes[offset] = (scan->bytes[offset] & 15u) | ((range & 15u) << 4u);
        scan->bytes[offset + 1u] = range >> 4u;
    } else {
        scan->bytes[offset] = range & 255u;
        scan->bytes[offset + 1u] = (scan->bytes[offset + 1u] & 240u) | (range >> 8u);
    }
}

unsigned s5_scan_get(const S5_SCAN *scan, unsigned index)
{
    unsigned offset = index * 3u / 2u;
    unsigned word = scan->bytes[offset] | ((unsigned)scan->bytes[offset + 1u] << 8u);
    return (word >> ((index & 1u) * 4u)) & 4095u;
}

unsigned s5_scan_target(const S5_SCAN *scan)
{
    /* Give priority to a return directly in the vehicle's path. */
    if (s5_scan_get(scan, S5_CENTRE_INDEX) <= S5_CLEAR_MM) return S5_CENTRE_INDEX;
    unsigned target = S5_NO_TARGET;
    unsigned closest = S5_CLEAR_MM + 1;
    for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP) {
        unsigned range = s5_scan_get(scan, i);
        if (range < closest) {
            target = i;
            closest = range;
        }
    }
    return target;
}

bool s5_same_obstacle(unsigned range, unsigned reference)
{
    return s5_range_measured(range) && s5_range_measured(reference) &&
           range <= reference + S5_PROFILE_TOLERANCE_MM &&
           reference <= range + S5_PROFILE_TOLERANCE_MM;
}

bool s5_background_beyond(unsigned range, unsigned reference)
{
    return range == S5_OPEN_RANGE ||
           (s5_range_measured(range) && range > reference + S5_PROFILE_TOLERANCE_MM);
}

unsigned s5_edge_background(const unsigned *ranges, unsigned count, unsigned reference)
{
    unsigned shortest = S5_UNKNOWN_RANGE;
    unsigned previous = S5_UNKNOWN_RANGE;
    unsigned distant_echoes = 0;
    bool repeated = false;
    for (unsigned i = 0; i < count; ++i) {
        unsigned range = ranges[i];
        if (s5_range_measured(range)) {
            /* A single near return leaves the bearing in the edge bracket. */
            if (!s5_background_beyond(range, reference)) return S5_UNKNOWN_RANGE;
            ++distant_echoes;
        }
        if (s5_range_available(s5_confirm_scan_range(previous, range))) repeated = true;
        if (s5_range_available(range) && range < shortest) shortest = range;
        if (s5_range_available(range) || !s5_range_measured(previous)) previous = range;
    }
    /* Beyond the edge, separate reflectors can produce different distances. */
    return repeated || distant_echoes >= MIN_DISTANT_ECHOES ? shortest : S5_UNKNOWN_RANGE;
}

typedef struct {
    unsigned last_hit, boundary, clearance;
} S5_EDGE;

typedef struct {
    unsigned range_mm, bearing_tenths;
    bool on_ray;
} S5_CORNER;

/* Infer the bearing of a stable off-axis echo on the same flat face. */
static unsigned corner_bearing_tenths(unsigned range, unsigned depth)
{
    range = width_ray_range(range);
    depth = width_ray_range(depth);
    if (!range || !depth || depth >= range) return 0;
    int expected_cosine = (int)(((uint64_t)depth * COSINE_SCALE + range / 2u) / range);
    unsigned low = 0, high = 900;
    while (low < high) {
        unsigned midpoint = (low + high) / 2u;
        if (cosine_tenths(midpoint) <= expected_cosine) high = midpoint;
        else low = midpoint + 1u;
    }
    if (low && cosine_tenths(low - 1u) - expected_cosine < expected_cosine - cosine_tenths(low))
        --low;
    return low;
}

static unsigned face_ray_range(unsigned bearing, unsigned face_depth)
{
    unsigned projection = cosine[angle_steps(bearing)];
    return (face_depth * 1024u + projection / 2u) / projection;
}

static bool width_ray_matches(unsigned range, unsigned bearing, unsigned face_depth)
{
    unsigned expected = face_ray_range(bearing, face_depth);
    return range + WIDTH_RAY_TOLERANCE_MM >= expected &&
           range <= expected + WIDTH_RAY_TOLERANCE_MM;
}

/* A repeated corner echo can be visible at multiple aimed angles. With
 * limited angular evidence, use two ranges that locate the same corner. */
static bool corner_from_pair(const S5_SCAN *scan, S5_EDGE edge, unsigned depth,
                             unsigned outer, unsigned inner, S5_CORNER *corner)
{
    unsigned range = s5_scan_get(scan, outer), supporting = s5_scan_get(scan, inner);
    if (!s5_same_obstacle(supporting, depth)) return false;

    bool outermost = outer == edge.last_hit;
    bool on_ray = width_ray_matches(range, outer, depth);
    if (outermost && on_ray && width_ray_matches(supporting, inner, depth)) {
        *corner = (S5_CORNER){
            .range_mm = range, .bearing_tenths = angle_steps(outer) * TENTHS_PER_STEP, .on_ray = true,
        };
        return true;
    }

    /* Off-axis returns can include different surfaces as the beam sweeps.
     * Require two bearings that see the same distance before using slant range. */
    if (supporting < depth + WIDTH_MIN_EDGE_RISE_MM || supporting > range + WIDTH_STABLE_CORNER_MM ||
        range > supporting + WIDTH_STABLE_CORNER_MM)
        return false;
    unsigned bearing = corner_bearing_tenths(range, depth);
    if (!bearing) return false;
    *corner = (S5_CORNER){.range_mm = range, .bearing_tenths = bearing};
    return true;
}

static bool width_corner(const S5_SCAN *scan, S5_EDGE edge, unsigned depth,
                         int step, S5_CORNER *corner)
{
    unsigned edge_steps = angle_steps(edge.last_hit);
    unsigned last_range = s5_scan_get(scan, edge.last_hit);
    for (unsigned i = 0; i < WIDTH_EDGE_CANDIDATES && i < edge_steps; ++i) {
        unsigned outer = (unsigned)((int)edge.last_hit - (int)i * step);
        unsigned range = s5_scan_get(scan, outer);
        if (!s5_same_obstacle(range, depth) || range < depth + WIDTH_MIN_EDGE_RISE_MM ||
            range > face_ray_range(outer, depth) + WIDTH_RAY_TOLERANCE_MM)
            continue;
        /* An inner pair can replace a closer face echo at the last bearing.
         * It cannot explain a substantially farther last hit. */
        if (last_range > range + WIDTH_STABLE_CORNER_MM) continue;
        for (unsigned gap = 1; gap <= WIDTH_MAX_SUPPORT_GAP && i + gap <= edge_steps; ++gap) {
            unsigned inner = (unsigned)((int)outer - (int)gap * step);
            if (corner_from_pair(scan, edge, depth, outer, inner, corner)) return true;
        }
    }
    return false;
}

static unsigned width_face_depth(const S5_SCAN *scan, S5_EDGE left, S5_EDGE right, unsigned reference)
{
    if (left.last_hit > S5_CENTRE_INDEX || right.last_hit < S5_CENTRE_INDEX)
        return S5_UNKNOWN_RANGE;
    unsigned centre = s5_scan_get(scan, S5_CENTRE_INDEX);
    if (s5_same_obstacle(centre, reference)) return centre;
    if (s5_range_available(centre) || left.last_hit == S5_CENTRE_INDEX ||
        right.last_hit == S5_CENTRE_INDEX)
        return S5_UNKNOWN_RANGE;

    unsigned left_range = s5_scan_get(scan, S5_CENTRE_INDEX - 1);
    unsigned right_range = s5_scan_get(scan, S5_CENTRE_INDEX + 1);
    if (!s5_same_obstacle(left_range, reference) || !s5_same_obstacle(right_range, reference) ||
        left_range > right_range + WIDTH_STABLE_CORNER_MM ||
        right_range > left_range + WIDTH_STABLE_CORNER_MM)
        return S5_UNKNOWN_RANGE;
    /* The symmetric adjacent returns give the face depth if centre missed. */
    return project_forward_mm(S5_CENTRE_INDEX - 1, (left_range + right_range + 1u) / 2u);
}

static S5_EDGE profile_edge(const S5_SCAN *scan, unsigned anchor, unsigned limit, int step)
{
    S5_EDGE edge = {.last_hit = anchor, .clearance = S5_UNKNOWN_RANGE};
    unsigned reference = s5_scan_get(scan, anchor);
    for (int i = (int)anchor; i != (int)limit;) {
        i += step;
        unsigned range = s5_scan_get(scan, (unsigned)i);
        if (!s5_range_available(range)) continue;
        if (s5_background_beyond(range, reference)) {
            edge.boundary = (unsigned)i;
            edge.clearance = range;
            break;
        }
        if (s5_same_obstacle(range, reference)) edge.last_hit = (unsigned)i;
    }
    return edge;
}

S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor,
                      S5_WIDTH_SCAN *width_scan)
{
    if (width_scan) *width_scan = (S5_WIDTH_SCAN){0};
    S5_EDGE left = profile_edge(scan, anchor, first, -1);
    S5_EDGE right = profile_edge(scan, anchor, last, 1);
    unsigned near = anchor;
    unsigned reference = s5_scan_get(scan, anchor);
    unsigned front = reference;
    for (unsigned i = left.last_hit; i <= right.last_hit; ++i) {
        unsigned range = s5_scan_get(scan, i);
        if (!s5_same_obstacle(range, reference)) continue;
        /* The legacy telemetry angle field represents the standard bearings. */
        unsigned bearing = s5_scan_bearing(i);
        if (bearing >= S5_MIN_ANGLE && bearing <= S5_MAX_ANGLE && range < s5_scan_get(scan, near))
            near = i;
        unsigned forward = project_forward_mm(i, range);
        if (forward < front) front = forward;
    }
    S5_PROFILE result = {
        .nearest_mm = s5_scan_get(scan, near),
        .angle_offset_deg = S5_MAX_ANGLE - s5_scan_bearing(near),
        .front_mm = front < S5_MAX_PROJECTED_FRONT_MM ? front : S5_MAX_PROJECTED_FRONT_MM,
        .width_mm = S5_UNKNOWN_WIDTH,
        .left_mm = left.clearance,
        .right_mm = right.clearance,
        .left_path_mm = S5_UNKNOWN_RANGE,
        .right_path_mm = S5_UNKNOWN_RANGE,
        .left_clipped = left.last_hit == 0,
        .right_clipped = right.last_hit == S5_SCAN_POINTS - 1,
    };
    /* Project the background bearing onto the depth of the adjacent obstacle face. */
    if (s5_range_available(left.clearance) && left.last_hit - left.boundary <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(left.last_hit, s5_scan_get(scan, left.last_hit));
        result.left_edge_mm = edge_bound_mm(left.boundary, depth);
        result.left_edge_valid = true;
    }
    if (s5_range_available(right.clearance) && right.boundary - right.last_hit <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(right.last_hit, s5_scan_get(scan, right.last_hit));
        result.right_edge_mm = edge_bound_mm(right.boundary, depth);
        result.right_edge_valid = true;
    }
    /* The width estimate is independent of the conservative motion bounds. */
    if (!result.left_edge_valid || !result.right_edge_valid) {
        result.width_status = result.left_edge_valid ? S5_WIDTH_RIGHT_EDGE :
                              result.right_edge_valid ? S5_WIDTH_LEFT_EDGE : S5_WIDTH_BOTH_EDGES;
        return result;
    }
    unsigned depth = width_face_depth(scan, left, right, reference);
    if (!s5_range_measured(depth) || result.nearest_mm + WIDTH_FACE_TOLERANCE_MM < depth) {
        result.width_status = S5_WIDTH_FACE;
        return result;
    }
    if (width_scan) {
        *width_scan = (S5_WIDTH_SCAN){
            .face_mm = depth,
            .left_hit_deg = angle_steps(left.last_hit) * S5_FINE_STEP,
            .left_background_deg = angle_steps(left.boundary) * S5_FINE_STEP,
            .right_hit_deg = angle_steps(right.last_hit) * S5_FINE_STEP,
            .right_background_deg = angle_steps(right.boundary) * S5_FINE_STEP,
            .left_hit_mm = s5_scan_get(scan, left.last_hit),
            .right_hit_mm = s5_scan_get(scan, right.last_hit),
            .off_axis = s5_scan_get(scan, left.last_hit) + WIDTH_RAY_TOLERANCE_MM <
                        face_ray_range(left.last_hit, depth) &&
                        s5_scan_get(scan, right.last_hit) + WIDTH_RAY_TOLERANCE_MM <
                        face_ray_range(right.last_hit, depth),
        };
    }
    S5_CORNER left_corner, right_corner;
    bool left_known = width_corner(scan, left, depth, -1, &left_corner);
    bool right_known = width_corner(scan, right, depth, 1, &right_corner);
    if (!left_known || !right_known) {
        result.width_status = left_known ? S5_WIDTH_RIGHT_ECHO :
                              right_known ? S5_WIDTH_LEFT_ECHO : S5_WIDTH_BOTH_ECHOES;
        return result;
    }
    if (width_scan) width_scan->on_rays = left_corner.on_ray && right_corner.on_ray;
    result.width_mm = s5_width_from_corners(left_corner.range_mm, (int)left_corner.bearing_tenths,
                                            right_corner.range_mm, -(int)right_corner.bearing_tenths);
    if (result.width_mm == S5_UNKNOWN_WIDTH) result.width_status = S5_WIDTH_BOTH_ECHOES;
    return result;
}
