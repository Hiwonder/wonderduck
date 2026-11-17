# WonderDuck

[English](README.md) | 简体中文

WonderDuck 提供机器人控制程序和 WonderDuck Studio（[GitHub](https://github.com/Hiwonder/wonderduck_studio) | [Gitee](https://gitee.com/hiwonder_official/wonderduck_studio)）。本项目基于 [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck)。

## 更新与部署

适用于已烧录并完成配置的设备镜像。先在网页点击“休息”并等待完成，然后选择一个来源：

```bash
repository=https://github.com/Hiwonder/wonderduck.git  # GitHub
# repository=https://gitee.com/hiwonder_official/wonderduck.git  # Gitee
```

在 [GitHub 发布页](https://github.com/Hiwonder/wonderduck/releases)或 [Gitee 发布页](https://gitee.com/hiwonder_official/wonderduck/releases)查看版本，将下面的 `0.1.0` 换成实际版本号并执行：

```bash
(
set -e
set -o pipefail
cd /home/wonderduck/wonderduck
release_version=0.1.0
case "$repository" in
  https://github.com/Hiwonder/wonderduck.git|https://gitee.com/hiwonder_official/wonderduck.git) ;;
  *) echo "请先选择上面的来源"; exit 1 ;;
esac
sudo systemctl stop robotd
git remote set-url origin "$repository"
git fetch --no-tags --no-recurse-submodules origin "refs/heads/main:refs/remotes/origin/main"
git fetch --no-tags --no-recurse-submodules origin "refs/tags/runtime-v${release_version}"
git merge-base --is-ancestor FETCH_HEAD refs/remotes/origin/main
git show "FETCH_HEAD:scripts/update-source.py" | python3 - "$release_version"
python3 scripts/runtime_release.py install --version "$release_version"
sudo systemctl start robotd
)
```

安装时会覆盖开机启动配置，保留设备标定、模型和动作组。如果本地文件与新版本冲突，更新会停止并提示，原文件会保留。处理问题后可重新执行更新命令。

Studio 提供局域网控制功能；使用时请将机器人接入可信网络。

## 恢复状态读取故障

如设备状态读取发生未确认的串口故障，机器人会停止使用舵机总线。请先停止 `robotd`，并在恢复期间不要启动任何 `robotd` 实例或其他直接访问舵机串口的程序。然后执行：

```bash
sudo systemctl stop robotd &&
sudo python3 /home/wonderduck/wonderduck/scripts/recover-status-bus.py prepare &&
sudo poweroff
```

设备关机后，断开电池、充电器、USB 和其他所有电源，使主板、舵机及 MCU 全部断电，再重新接通并开机。单独重启主板不能代替完全断电。开机后执行：

```bash
sudo systemctl stop robotd &&
sudo python3 /home/wonderduck/wonderduck/scripts/recover-status-bus.py recover &&
sudo systemctl start robotd
```

`recover` 会要求在终端输入 `POWER-CYCLED`，仅在确认所有设备均已完全断电重开后输入。如果恢复失败，请保持服务停止，检查提示后重新操作。
