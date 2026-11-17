#!/usr/bin/env python3
"""Configure Wi-Fi through the running wifi-manager service.

Run this file after editing the settings, or leave the station fields empty
to enter them interactively. The service stores successful connections and
returns to its existing hotspot when a connection fails.
"""

import argparse
import getpass
import json
import socket
import string
import sys


# 1: restore the existing device hotspot. 2: connect to a Wi-Fi network.
WIFI_MODE = 2
WIFI_STA_SSID = ""
WIFI_STA_PASSWORD = ""


def make_request(mode, ssid="", password=""):
    if type(mode) is not int or mode not in (1, 2):
        raise ValueError("WIFI_MODE must be 1 (hotspot) or 2 (Wi-Fi)")
    if mode == 1:
        return 9028, b"resetwifi"

    if not isinstance(ssid, str) or not ssid or "\0" in ssid:
        raise ValueError("Wi-Fi name must be a non-empty string")
    if len(ssid.encode("utf-8")) > 32:
        raise ValueError("Wi-Fi name must be at most 32 UTF-8 bytes")
    if not isinstance(password, str):
        raise ValueError("Wi-Fi password must be a string")
    if password:
        passphrase = 8 <= len(password) <= 63 and all(
            " " <= character <= "~" for character in password
        )
        hex_key = len(password) == 64 and all(
            character in string.hexdigits for character in password
        )
        if not (passphrase or hex_key):
            raise ValueError("Wi-Fi password must be 8-63 ASCII characters or 64 hex digits")

    request = {"setwifi": {"ssid": ssid, "passwd": password}}
    return 9026, json.dumps(request, ensure_ascii=True).encode("utf-8")


def send_request(port, payload):
    with socket.create_connection(("127.0.0.1", port), timeout=3) as connection:
        connection.sendall(payload)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ap", action="store_true", help="restore the device hotspot")
    parser.add_argument("--check", action="store_true", help="validate without switching networks")
    args = parser.parse_args(argv)

    mode = 1 if args.ap else WIFI_MODE
    ssid = WIFI_STA_SSID
    password = WIFI_STA_PASSWORD
    if mode == 2:
        if not ssid:
            ssid = input("Wi-Fi name (SSID): ")
        if not password:
            password = getpass.getpass("Wi-Fi password (hidden; Enter for open network): ")
    port, payload = make_request(mode, ssid, password)

    if args.check:
        print("Configuration is valid. No network changes were made.")
        return

    print("Sending request to wifi-manager. The SSH connection may be interrupted.", flush=True)
    send_request(port, payload)
    print("Request sent. The service will attempt the connection and restore the hotspot if it fails.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, EOFError) as error:
        print("Error: {}".format(error), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nCancelled.", file=sys.stderr)
        sys.exit(1)
