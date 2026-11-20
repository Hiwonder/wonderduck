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

## 在电脑上交叉编译

以下步骤适用于 x86-64 Ubuntu 电脑，以及已烧录并配置 WonderDuck 镜像的设备。电脑需先安装 Git、curl、rsync、pkg-config、Python 3 的 `venv`、C 编译器和 Node.js 20。在电脑上任选一个来源克隆：

```bash
git clone --recurse-submodules https://github.com/Hiwonder/wonderduck.git
# 或者：git clone --recurse-submodules https://gitee.com/hiwonder_official/wonderduck.git
cd wonderduck
```

如需开发分支，在克隆命令中加入 `-b develop`。安装 Rust、ARM64 目标、cargo-zigbuild 和 Zig：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
. "$HOME/.cargo/env"
rustup toolchain install 1.99.0 --profile minimal
rustup target add aarch64-unknown-linux-gnu --toolchain 1.99.0
cargo +1.99.0 install cargo-zigbuild --version 0.23.4 --locked
build_env="$HOME/.local/share/wonderduck-build"
python3 -m venv "$build_env"
"$build_env/bin/pip" install ziglang==0.14.1
ln -sf "$("$build_env/bin/python" -c 'import pathlib, ziglang; print(pathlib.Path(ziglang.__file__).parent / "zig")')" "$build_env/bin/zig"
export PATH="$build_env/bin:$PATH" CARGO_ZIGBUILD_PYTHON_PATH="$build_env/bin/python"
```

将示例 IP 改为设备 IP，复制设备上的 ARM64 `libudev`，然后编译程序和网页：

```bash
robot=wonderduck@192.0.2.1
sysroot="$HOME/.local/share/wonderduck-sysroot"
mkdir -p "$sysroot/lib/aarch64-linux-gnu" "$sysroot/usr/lib/aarch64-linux-gnu/pkgconfig"
scp "${robot}:/lib/aarch64-linux-gnu/libudev.so.1" "$sysroot/lib/aarch64-linux-gnu/libudev.so"
cat > "$sysroot/usr/lib/aarch64-linux-gnu/pkgconfig/libudev.pc" <<'EOF'
prefix=/usr
libdir=/lib/aarch64-linux-gnu
includedir=/usr/include
Name: libudev
Description: Library to access udev device information
Version: 245
Libs: -L${libdir} -ludev
Cflags: -I${includedir}
EOF
export PKG_CONFIG_SYSROOT_DIR="$sysroot"
export PKG_CONFIG_LIBDIR="$sysroot/usr/lib/aarch64-linux-gnu/pkgconfig"
export PKG_CONFIG_ALLOW_CROSS=1
cargo +1.99.0 zigbuild --release --target aarch64-unknown-linux-gnu.2.31 -p robotd -p padd -p robotctl --bins --locked
cargo +1.99.0 zigbuild --release --target aarch64-unknown-linux-gnu.2.31 --manifest-path extensions/wonderduck_studio/webpadd/Cargo.toml --bin webpadd --locked
npm ci --prefix extensions/wonderduck_studio/console/web
npm run build --prefix extensions/wonderduck_studio/console/web
```

部署本机编译结果时，先在网页点击“休息”并等待完成，再在同一个电脑终端执行以下命令。新增的源码文件需先纳入 Git 跟踪。这些命令会覆盖程序文件，并保留模型、动作组、标定和 `/etc/robot`：

```bash
(
set -euo pipefail
ssh "$robot" 'sudo systemctl stop robotd'
git ls-files scripts/ assets/prompt-sounds/network/ | rsync -a --files-from=- ./ "${robot}:/home/wonderduck/wonderduck/"
git -C extensions/wonderduck_studio ls-files console/ webpadd/web/ | rsync -a --files-from=- extensions/wonderduck_studio/ "${robot}:/home/wonderduck/wonderduck/extensions/wonderduck_studio/"
rsync -a extensions/wonderduck_studio/console/web/build/ "${robot}:/home/wonderduck/wonderduck/extensions/wonderduck_studio/console/web/build/"
scp target/aarch64-unknown-linux-gnu/release/{robotd,padd,robotctl} extensions/wonderduck_studio/webpadd/target/aarch64-unknown-linux-gnu/release/webpadd "${robot}:/home/wonderduck/wonderduck/target/release/"
scp assets/prompt-sounds/{startup,low_battery}.wav "${robot}:/home/wonderduck/wonderduck/target/prompt-sounds/"
ssh "$robot" 'sudo systemctl restart wonderduck-prompt-sounds && sudo systemctl start robotd'
)
```

如果执行失败，机器人服务可能保持停止；请排除错误后重新执行部署。此开发部署不会更新固件、驱动或系统服务配置，也可能使设备的 Git 工作区出现修改；之后安装正式版时，请先处理更新程序提示的文件冲突。

## 在实机上编译和部署

以下命令在已配置 WonderDuck 镜像的 RK3566 实机上执行。设备正在使用的程序位于 `~/wonderduck`。克隆成功后会创建独立的编译目录 `~/wonderduck-src`。

```bash
git clone --branch main --recurse-submodules https://gitee.com/hiwonder_official/wonderduck.git "$HOME/wonderduck-src" &&
  cd "$HOME/wonderduck-src"
```

也可以将克隆地址换成 `https://github.com/Hiwonder/wonderduck.git`。如果克隆失败，请先处理 Git 的报错。主仓和 Studio 子模块都已克隆到该目录时，可以直接开始编译。

请安装 ARM64 的 Node.js 20 或更高版本。Ubuntu 20.04 默认软件源的 `nodejs` 版本过旧。1 GB 内存的设备可用 `free -h` 检查可用内存，必要时先启用交换空间。以下 Rust 命令只使用一个并行任务。 [Node.js 下载](https://nodejs.org/en/download)

```bash
(
set -euo pipefail
cd "$HOME/wonderduck-src"
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libudev-dev git curl rsync
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
. "$HOME/.cargo/env"
rustup toolchain install 1.99.0 --profile minimal
export DUCK_REVISION="$(git rev-parse HEAD)"
cargo +1.99.0 build --release --locked -j 1 -p robotd -p padd -p robotctl --bins
cargo +1.99.0 build --release --locked -j 1 --manifest-path extensions/wonderduck_studio/webpadd/Cargo.toml --bin webpadd
npm ci --prefix extensions/wonderduck_studio/console/web
npm run build --prefix extensions/wonderduck_studio/console/web
)
```

编译完成后，在网页点击「休息」并等待完成，再执行部署命令。部署使用镜像已安装的运行库，并保留设备模型、动作组和标定。

```bash
(
set -euo pipefail
cd "$HOME/wonderduck-src"
installed="$HOME/wonderduck"
sudo systemctl stop robotd
git ls-files scripts/ assets/prompt-sounds/network/ | rsync -a --files-from=- ./ "$installed/"
git -C extensions/wonderduck_studio ls-files console/ webpadd/web/ | rsync -a --files-from=- extensions/wonderduck_studio/ "$installed/extensions/wonderduck_studio/"
rsync -a extensions/wonderduck_studio/console/web/build/ "$installed/extensions/wonderduck_studio/console/web/build/"
install -m 0755 target/release/{robotd,padd,robotctl} extensions/wonderduck_studio/webpadd/target/release/webpadd "$installed/target/release/"
install -m 0644 assets/prompt-sounds/{startup,low_battery}.wav "$installed/target/prompt-sounds/"
sudo systemctl restart wonderduck-prompt-sounds
sudo systemctl start robotd
systemctl is-active robotd
)
```

打开 Studio 验证部署。测试完成后，可清理这次使用的 `~/wonderduck-src` 源码目录和临时交换文件。运行的程序位于 `~/wonderduck`。

## 恢复状态读取故障

设备状态读取发生串口故障时，机器人会关闭运动控制，尝试恢复通信并逐个确认舵机卸力。如果故障持续，请检查舵机和 MCU 的连接及供电，再重新打开设备状态页；仍无法恢复时请联系技术支持。
