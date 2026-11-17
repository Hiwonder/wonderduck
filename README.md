# WonderDuck

English | [简体中文](README_zh.md)

WonderDuck provides robot control software and [WonderDuck Studio](https://github.com/Hiwonder/wonderduck_studio). It is based on [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck).

## Update and deployment

For a device with a configured system image, select **Rest** in the web interface and wait for it to finish. Find the version on the [releases page](https://github.com/Hiwonder/wonderduck/releases), replace `0.1.0` below with that version, and run these commands on the device:

```bash
(
set -e
set -o pipefail
cd /home/wonderduck/wonderduck
release_version=0.1.0
sudo systemctl stop robotd
git remote set-url origin https://github.com/Hiwonder/wonderduck.git
git fetch --no-tags origin "refs/heads/main:refs/remotes/origin/main"
git fetch --no-tags origin "refs/tags/runtime-v${release_version}"
git merge-base --is-ancestor FETCH_HEAD refs/remotes/origin/main
git show "FETCH_HEAD:scripts/update-source.py" | python3 - "$release_version"
python3 scripts/runtime_release.py install --version "$release_version"
sudo systemctl start robotd
)
```

Installation updates the startup configuration and preserves device calibration, models, and action groups. If a local file conflicts with the new version, the update stops and keeps the original file. Resolve the conflict and run the commands again.

Studio provides control over the local network. Connect the robot to a trusted network.

## Recover from a device status read failure

If an unconfirmed serial error occurs while reading device status, the robot stops using the servo bus. Stop `robotd` and do not start it or any other program that accesses the servo serial port during recovery. Run:

```bash
sudo systemctl stop robotd &&
sudo python3 /home/wonderduck/wonderduck/scripts/recover-status-bus.py prepare &&
sudo poweroff
```

After shutdown, disconnect the battery, charger, USB, and all other power sources so that the board, servos, and MCU are completely powered off. Reconnect power and start the device. Restarting the board alone is not sufficient. Then run:

```bash
sudo systemctl stop robotd &&
sudo python3 /home/wonderduck/wonderduck/scripts/recover-status-bus.py recover &&
sudo systemctl start robotd
```

Enter `POWER-CYCLED` only after confirming that all devices were fully powered off and restarted. If recovery fails, keep the service stopped and follow the reported instructions before retrying.
