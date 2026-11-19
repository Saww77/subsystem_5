"""Fit left and right angular excess from centred, square bench targets."""

import argparse
import math
from pathlib import Path


def fit(samples):
    depth_min, depth_max = 270, 1
    lower = [0.0, 0.0]
    upper = [75.0, 75.0]
    for width, depth, left_hit, left_background, right_hit, right_background in samples:
        if width <= 0 or not 1 <= depth <= 270:
            raise ValueError("use a positive measured width and a depth from 1 to 270 mm")
        expected_edge = math.degrees(math.atan2(width / 2, depth))
        for side, (hit, background) in enumerate(((left_hit, left_background),
                                                   (right_hit, right_background))):
            if not 0 <= hit < background <= 75 or background - hit > 10:
                raise ValueError("use edge brackets of 1 to 10 degrees from a complete sweep")
            lower[side] = max(lower[side], hit - expected_edge)
            upper[side] = min(upper[side], background - expected_edge)
        depth_min = min(depth_min, depth)
        depth_max = max(depth_max, depth)

    if any(low > high for low, high in zip(lower, upper)):
        raise ValueError("samples disagree on the beam offset. Verify servo angles and repeat the scans")
    angles = [round((low + high) * 5) for low, high in zip(lower, upper)]
    # The observed range can vary by one sensor increment at the same position.
    return (*angles, max(1, depth_min - 5), min(270, depth_max + 5))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", required=True, action="append", nargs=6, type=int, metavar="N",
                        help="width depth left_hit left_background right_hit right_background")
    parser.add_argument("--output", type=Path, help="write a calibration header to this path")
    args = parser.parse_args()
    try:
        left, right, minimum, maximum = fit(args.sample)
    except ValueError as error:
        parser.error(str(error))
    header = (
        "/* Local calibration from measured Subsystem 5 bench targets. */\n"
        f"#define S5_WIDTH_LEFT_EXCESS_TENTHS {left}u\n"
        f"#define S5_WIDTH_RIGHT_EXCESS_TENTHS {right}u\n"
        f"#define S5_WIDTH_CAL_MIN_MM {minimum}u\n"
        f"#define S5_WIDTH_CAL_MAX_MM {maximum}u\n"
    )
    if args.output:
        args.output.write_text(header, encoding="utf-8")
    else:
        print(header, end="")


if __name__ == "__main__":
    main()
