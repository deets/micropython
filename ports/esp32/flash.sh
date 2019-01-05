#!/usr/bin/env python3
# -*- mode: python -*-
import sys
import pathlib
import subprocess
import argparse
import platform

DEFAULT_SERIAL_PORT = "/dev/tty.SLAB_USBtoUART" if \
    platform.system() == 'Darwin' else "/dev/ttyUSB0"

BASE = pathlib.Path(__file__).parent

FIRMWARE = BASE / "build" / "firmware.bin"
ESPTOOL_PY = pathlib.Path(sys.executable).parent / "esptool.py"


def main():
    assert FIRMWARE.exists(), str(FIRMWARE)
    assert ESPTOOL_PY.exists(), str(ESPTOOL_PY)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
    )
    opts = parser.parse_args()

    subprocess.run(
        [
            sys.executable,
            str(ESPTOOL_PY),
            "--chip",  "esp32",
            "--port", opts.port,
            "--baud", "460800",
            "write_flash",
            "-z", "--flash_mode", "dio",
            "--flash_freq", "40m",
            "0x1000",
            str(FIRMWARE)
        ],
        check=True,
    )


if __name__ == '__main__':
    main()
