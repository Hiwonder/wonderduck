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

If a device status read encounters a serial fault, motion control is disabled while the robot attempts to restore communication and verify each servo is released. If the fault persists, check the servo and MCU connections and power, then reopen the device status page. Contact support if recovery still fails.
