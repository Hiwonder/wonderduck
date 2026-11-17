# WonderDuck

[English](README.md) | 简体中文

WonderDuck 提供机器人控制程序和 WonderDuck Studio（[GitHub](https://github.com/Hiwonder/wonderduck_studio) | [Gitee](https://gitee.com/hiwonder_official/wonderduck_studio)）。本项目基于 [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck)。

## 更新与部署

适用于已烧录并完成配置的设备镜像。先在网页点击“休息”并等待完成，然后选择一个来源：

GitHub：

```bash
repository=https://github.com/Hiwonder/wonderduck.git
```

Gitee：

```bash
repository=https://gitee.com/hiwonder_official/wonderduck.git
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
git fetch --no-tags --no-recurse-submodules origin "+refs/heads/main:refs/remotes/origin/main"
git fetch --no-tags --no-recurse-submodules origin "refs/tags/v${release_version}"
git merge-base --is-ancestor FETCH_HEAD refs/remotes/origin/main
git show "FETCH_HEAD:scripts/update-source.py" | python3 - "$release_version"
python3 scripts/runtime_release.py install --version "$release_version"
sudo systemctl start robotd
)
```

安装时会覆盖开机启动配置，保留设备标定、模型和动作组。如果本地文件与新版本冲突，更新会停止并提示，原文件会保留。处理问题后可重新执行更新命令。

Studio 提供局域网控制功能；使用时请将机器人接入可信网络。

## 恢复状态读取故障

设备状态读取发生串口故障时，机器人会关闭运动控制，尝试恢复通信并逐个确认舵机卸力。如果故障持续，请检查舵机和 MCU 的连接及供电，再重新打开设备状态页；仍无法恢复时请联系技术支持。
