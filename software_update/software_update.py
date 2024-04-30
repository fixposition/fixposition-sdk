#!/usr/bin/env python3

# software_update.py
# Copyright (C) 2020  tBOT
# Copyright (C) 2022  Fixposition
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import asyncio
import json
import requests
import websockets  # type: ignore
import argparse

# For software version 2.63 or newer (after 09.03.2023)
url_upload = "http://{}/update/upload"
url_status = "ws://{}/update/ws"

# For software version 2.58.2 or older (before 17.01.2023)
# url_upload = "http://{}:8080/upload"
# url_status = "ws://{}:8080/ws"

async def wait_update_finished(
    swu_file: str, target_ip: str, timeout: int = 300
) -> None:

    print("Wait update finished")

    async def get_finish_messages() -> int:
        async with websockets.connect(url_status.format(target_ip)) as websocket:

            while True:
                message = await websocket.recv()
                try:
                    data = json.loads(message)
                except ValueError:
                    data = {"type": "UNKNOWN"}

                if data["type"] == "status":
                    if data["status"] == "START":
                        continue
                    if data["status"] == "RUN":
                        print("Updating starts")
                        continue
                    if data["status"] == "UNKNOWN":
                        continue
                    if data["status"] == "SUCCESS":
                        print("Software update successful !")
                        return 0
                    if data["status"] == "FAILURE":
                        print("Software update failed !")
                        return 1
                if data["type"] == "info":
                    print("info")
                    continue

    await asyncio.wait_for(get_finish_messages(), timeout=timeout)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("ip")
    parser.add_argument("timeout", type=int)
    args = parser.parse_args()

    print("Start uploading image...")
    try:
        response = requests.post(
            url_upload.format(args.ip), files={"file": open(args.path, "rb")},
        )

        if response.status_code != 200:
            raise Exception("Cannot upload software image: {}".format(response.status_code))

        print("Software image uploaded successfully. Wait for installation to be finished...\n")

        asyncio.get_event_loop().run_until_complete(
            wait_update_finished(args.path, args.ip, timeout=args.timeout)
        )

    except ValueError:
        print("No connection to host, exit")


if __name__ == "__main__":
    main()
