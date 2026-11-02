
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Serial activation helper for CANControl devices.

Features:
- Default baud rate: 115200
- Automatic GetID command
- Supports ID=0x... and ID:0x... formats
- Automatic S1 activation code generation
- ZTE4875 protocol support
- Finite ID timeout
- Activation response monitoring
"""

import argparse
import logging
import re
import sys
import time
from enum import IntEnum

import serial

from activation_code import generate_activation_code, Channel


class ProtocolType(IntEnum):
    PROTOCOL_HUAWEI = 0x1
    PROTOCOL_INCREASE = 0x2
    PROTOCOL_ZTE3000 = 0x4
    PROTOCOL_INFY = 0x8
    PROTOCOL_EV_STATION = 0xC
    PROTOCOL_ZEEHO = 0xD
    PROTOCOL_EV_CHARGER = 0xE
    PROTOCOL_EPS6020 = 0x10
    PROTOCOL_ZTE4875 = 0x20
    PROTOCOL_SER10010K = 0x40
    PROTOCOL_BH10M100 = 0x41
    PROTOCOL_MC1503N5R = 0x42
    PROTOCOL_R24_2200 = 0x43

    @staticmethod
    def list_protocols():
        return [
            p.name.split("_", 1)[1].upper()
            for p in ProtocolType
        ]


def setup_logger():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def parse_args():

    parser = argparse.ArgumentParser(
        description="Serial activation tool for CANControl devices."
    )

    parser.add_argument(
        "-c",
        "--com-port",
        required=True,
        help="Serial COM port, e.g. COM9"
    )

    parser.add_argument(
        "-p",
        "--protocol",
        choices=ProtocolType.list_protocols(),
        default="ZTE4875",
        type=str.upper,
        help="Protocol type (default: ZTE4875)"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)"
    )

    parser.add_argument(
        "--id-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for ID response (default: 10)"
    )

    return parser.parse_args()


def pop_serial_lines(buffer):
    """
    Extract complete serial lines.
    Supports CR, LF and CRLF.
    """

    while True:

        positions = [
            p for p in (
                buffer.find(b"\r"),
                buffer.find(b"\n")
            )
            if p >= 0
        ]

        if not positions:
            return

        pos = min(positions)

        raw_line = bytes(buffer[:pos])

        delimiter = bytes(buffer[pos:pos + 1])

        del buffer[:pos + 1]

        if delimiter == b"\r" and buffer[:1] == b"\n":
            del buffer[:1]

        line = raw_line.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if line:
            yield line


def send_command(ser, command):

    payload = (command + "\r\n").encode("ascii")

    ser.write(payload)

    ser.flush()

    logging.info("Sent: %s", command)


def wait_for_device_id(ser, timeout_seconds):
    """
    Request device ID and wait for a valid response.

    Supported formats:
        ID=0xC82CD8BD2A58
        ID:0xC82CD8BD2A58
        ID=C8:2C:D8:BD:2A:58
    """

    buffer = bytearray()

    id_pattern = re.compile(
        r"\bID\s*[:=]\s*"
        r"(0x[0-9a-fA-F]{12}|"
        r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})",
        re.IGNORECASE
    )

    deadline = time.monotonic() + timeout_seconds

    send_command(ser, "GetID")

    while time.monotonic() < deadline:

        chunk = ser.read(ser.in_waiting or 1)

        if not chunk:
            continue

        buffer.extend(chunk)

        for line in pop_serial_lines(buffer):

            logging.info("Received: %s", line)

            match = id_pattern.search(line)

            if match:
                return match.group(1)

    return None


def collect_device_output(ser, seconds=3.0):
    """
    Read and display device responses.
    Does not assume any particular ACK format.
    """

    buffer = bytearray()

    deadline = time.monotonic() + seconds

    found_output = False

    while time.monotonic() < deadline:

        chunk = ser.read(ser.in_waiting or 1)

        if not chunk:
            continue

        buffer.extend(chunk)

        for line in pop_serial_lines(buffer):

            found_output = True

            logging.info("Device: %s", line)

    if buffer:

        tail = buffer.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if tail:

            found_output = True

            logging.info("Device (partial): %s", tail)

    return found_output


def main():

    args = parse_args()

    setup_logger()

    protocol = ProtocolType[
        "PROTOCOL_" + args.protocol
    ]

    logging.info(
        "Using protocol: %s (0x%X)",
        protocol.name,
        protocol.value
    )

    logging.info(
        "Serial settings: %s at %d baud",
        args.com_port,
        args.baud
    )

    try:

        with serial.Serial(
            port=args.com_port,
            baudrate=args.baud,
            timeout=0.2,
            write_timeout=2,
        ) as ser:

            logging.info(
                "Opened serial port: %s",
                ser.name
            )

            # Allow ESP32 to finish booting.
            time.sleep(1.5)

            # Clear old boot messages.
            ser.reset_input_buffer()

            # Request Device ID.
            chip_id = wait_for_device_id(
                ser,
                args.id_timeout
            )

            if not chip_id:

                logging.error(
                    "No ID response within %.1f seconds. "
                    "Check COM port, baud rate, and firmware.",
                    args.id_timeout
                )

                return 2

            logging.info(
                "Device ID: %s",
                chip_id
            )

            # Generate S1 activation code.
            code = generate_activation_code(
                chip_id,
                Channel.S1
            )

            logging.info(
                "Generated S1 activation code: %s",
                code
            )

            # Configure ZTE4875 protocol.
            send_command(
                ser,
                f"SetMSG=0,{protocol.value},0,0,0"
            )

            time.sleep(0.3)

            # Send activation code.
            send_command(
                ser,
                f"Active={code}"
            )

            # Read device response.
            got_output = collect_device_output(
                ser,
                seconds=3.0
            )

            if not got_output:

                logging.warning(
                    "No response was observed after sending "
                    "the commands. Activation is NOT confirmed."
                )

            else:

                logging.warning(
                    "Device output received. "
                    "Review the response to confirm activation. "
                    "Transmission alone does not confirm success."
                )

            return 0

    except serial.SerialException as exc:

        logging.error(
            "Serial error: %s",
            exc
        )

        return 1

    except (ValueError, KeyError) as exc:

        logging.error(
            "Activation calculation/configuration error: %s",
            exc
        )

        return 1


if __name__ == "__main__":

    sys.exit(main())
