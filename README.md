# WonderDuck

English | [简体中文](README_zh.md)

WonderDuck provides robot control software and WonderDuck Studio ([GitHub](https://github.com/Hiwonder/wonderduck_studio) | [Gitee](https://gitee.com/hiwonder_official/wonderduck_studio)). It is based on [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck).

## Update and deployment

On a device with a configured system image, select **Rest** and wait for it to finish. Choose a source:

GitHub:

```bash
repository=https://github.com/Hiwonder/wonderduck.git
```

Gitee:

```bash
repository=https://gitee.com/hiwonder_official/wonderduck.git
```

Find the version on [GitHub Releases](https://github.com/Hiwonder/wonderduck/releases) or [Gitee Releases](https://gitee.com/hiwonder_official/wonderduck/releases), replace `0.1.0` below, and run:

```bash
(
set -e
set -o pipefail
cd /home/wonderduck/wonderduck
release_version=0.1.0
case "$repository" in
  https://github.com/Hiwonder/wonderduck.git|https://gitee.com/hiwonder_official/wonderduck.git) ;;
  *) echo "Select a source above"; exit 1 ;;
esac
sudo systemctl stop robotd
git remote set-url origin "$repository"
git fetch --no-tags --no-recurse-submodules origin "+refs/heads/main:refs/remotes/origin/main"
git fetch --no-tags --no-recurse-submodules origin "refs/tags/v${release_version}"
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
