# WonderDuck

[English](README.md) | 简体中文

WonderDuck 提供机器人控制程序和 WonderDuck Studio（[GitHub](https://github.com/Hiwonder/wonderduck_studio) | [Gitee](https://gitee.com/hiwonder_official/wonderduck_studio)）。本项目基于 [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck)。

## 更新与部署

适用于已烧录并完成配置的设备镜像。先在网页点击“休息”并等待完成，然后在设备终端运行任一命令：

GitHub：

```bash
python3 <(curl -fsSL --connect-timeout 10 https://raw.githubusercontent.com/Hiwonder/wonderduck/main/scripts/update.py) --lang zh
```

Gitee：

```bash
python3 <(curl -fsSL --connect-timeout 10 https://gitee.com/hiwonder_official/wonderduck/raw/main/scripts/update.py) --lang zh
```

按提示选择更新来源并输入版本号；版本号直接按回车会自动选择最新正式版。

安装时会覆盖开机启动配置，保留设备标定、模型和动作组。如果本地文件与新版本冲突，更新会停止并提示，原文件会保留。处理问题后可重新执行更新命令。

Studio 提供局域网控制功能；使用时请将机器人接入可信网络。

## 恢复状态读取故障

设备状态读取发生串口故障时，机器人会关闭运动控制，尝试恢复通信并逐个确认舵机卸力。如果故障持续，请检查舵机和 MCU 的连接及供电，再重新打开设备状态页；仍无法恢复时请联系技术支持。
