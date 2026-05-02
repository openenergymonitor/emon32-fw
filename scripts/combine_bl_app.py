#!/usr/bin/env python3

import os
import sys

BL_SIZE = 8192  # 8KB bootloader region


def main():
    if len(sys.argv) != 3:
        print("usage: combine_bl_app.py <bl .bin path> <application .bin path>")
        return 1

    bl_path = sys.argv[1]
    app_path = sys.argv[2]

    if not bl_path.endswith(".bin"):
        print(f"Bootloader {bl_path} is not a .bin")
        return 1

    if not app_path.endswith(".bin"):
        print(f"Application {app_path} is not a .bin")
        return 1

    try:
        with open(bl_path, "rb") as f:
            bl_bytes = bytearray(f.read())
    except IOError as e:
        print(f"Exception {e} when opening {bl_path}")
        return 1

    try:
        with open(app_path, "rb") as f:
            app_bytes = bytearray(f.read())
    except IOError as e:
        print(f"Exception {e} when opening {app_path}")
        return 1

    if len(bl_bytes) > BL_SIZE:
        print(f"Bootloader {bl_path} exceeds bootloader region size of {BL_SIZE} bytes")
        return 1

    padding_bytes = bytearray([0xFF] * (BL_SIZE - len(bl_bytes)))

    combined_bytes = bl_bytes + padding_bytes + app_bytes

    out_path = f"{os.path.dirname(app_path)}/combined-{os.path.basename(bl_path).split('-')[0]}-{os.path.basename(app_path)}"

    try:
        with open(out_path, "wb") as f:
            f.write(combined_bytes)
    except IOError as e:
        print(f"Exception {e} when writing {out_path}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
