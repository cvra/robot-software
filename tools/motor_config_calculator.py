import argparse
from math import pi


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("p", type=float, help="Transmission numerator")
    parser.add_argument("q", type=float, help="Transmission denominator")
    parser.add_argument("km", type=float, help="Torque constant in mNm/A")
    parser.add_argument("vmax", type=float, help="Maximum velocity in turns per minute")
    parser.add_argument(
        "amax",
        type=float,
        help="Maximum acceleration in 1000 radians per second squared",
    )
    parser.add_argument("tmax", type=float, help="Maximum torque in mNm")

    return parser.parse_args()


def main():
    args = parse_args()

    transmission = args.q / args.p
    km = args.km / 1000
    vmax = args.vmax * 2 * pi / 60
    amax = args.amax * 2 * pi * 1000
    tmax = args.tmax / 1000

    print("motor:")
    print(
        "    torque_cst: {:.3f} # Nm/A after transmission, before {:.4f}, datasheet: {:.1f} mNm/A".format(
            km * transmission, km, args.km
        )
    )
    print("control:")
    print(
        "    acceleration_limit: {:.1f} # rad/s^2 after transmission, before {:.1f}, datasheet: {:.0f} 10^3 rad/s^2".format(
            amax / transmission, amax, args.amax
        )
    )
    print(
        "    velocity_limit: {:.1f} # rad/s after transmission, before {:.1f}, datasheet: {:.0f} /min".format(
            vmax / transmission, vmax, args.vmax
        )
    )
    print(
        "    torque_limit: {:.1f} # Nm after transmission, before {:.4f}, datasheet: {:.1f} mNm".format(
            tmax * transmission, tmax, args.tmax
        )
    )


if __name__ == "__main__":
    main()
