#!/usr/bin/env python3
# -*- mode: python -*-
import sys
import subprocess


def main():
    stacktrace = sys.stdin.read()
    stacktrace = stacktrace.replace("Backtrace:", "")
    for entry in stacktrace.strip().split():
        address, _ = entry.split(":")
        subprocess.run(
            [
                "xtensa-esp32-elf-addr2line",
                "-pfiaC",
                "-e", "build/application.elf",
                address,
            ]
        )

if __name__ == '__main__':
    main()
