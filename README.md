# WonderDuck

English | [简体中文](README_zh.md)

WonderDuck provides robot control software and WonderDuck Studio ([GitHub](https://github.com/Hiwonder/wonderduck_studio) | [Gitee](https://gitee.com/hiwonder_official/wonderduck_studio)). It is based on [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck).

## Update and deployment

On a device with a configured system image, select **Rest** and wait for it to finish. Run either command in the device terminal:

GitHub:

```bash
python3 <(curl -fsSL --connect-timeout 10 https://raw.githubusercontent.com/Hiwonder/wonderduck/main/scripts/update.py) --lang en
```

Gitee:

```bash
python3 <(curl -fsSL --connect-timeout 10 https://gitee.com/hiwonder_official/wonderduck/raw/main/scripts/update.py) --lang en
```

Choose the update source when prompted. Enter a version, or press Enter to install the latest stable release.

Installation updates the startup configuration and preserves device calibration, models, and action groups. If a local file conflicts with the new version, the update stops and keeps the original file. Resolve the conflict and run the commands again.

Studio provides control over the local network. Connect the robot to a trusted network.

## Recover from a device status read failure

If a device status read encounters a serial fault, motion control is disabled while the robot attempts to restore communication and verify each servo is released. If the fault persists, check the servo and MCU connections and power, then reopen the device status page. Contact support if recovery still fails.
