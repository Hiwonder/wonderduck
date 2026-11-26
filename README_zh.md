# WonderDuck

[English](README.md) | 简体中文

WonderDuck 提供机器人控制程序和 [WonderDuck Studio](https://gitee.com/hiwonder_official/wonderduck_studio)。本项目基于 [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck)。

## 模型许可

Studio 中的 WonderDuck 机器人 GLB 模型采用 CC BY-NC-SA 4.0。[许可证全文](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

## 更新与部署

以下两种更新方式均适用于已烧录并完成配置的设备。更新前，先在 Studio 的「网络管理 → 局域网模式」中将机器人连接到可访问互联网的可信 Wi-Fi 网络。

### Studio 0.2.0 及以上

在 Studio 打开「系统设置 → 软件版本 → 检查更新」。确认版本后开始更新。页面显示下载进度、安装状态，以及安装或启动失败后的恢复结果。

### Studio 0.2.0 之前的版本

通过设备终端更新。先在 Studio 点击「休息」并等待完成，然后在设备终端执行以下命令。

```bash
python3 <(curl -fsSL --connect-timeout 10 https://gitee.com/hiwonder_official/wonderduck/raw/main/scripts/update.py) --lang zh
```

选择 **2（Gitee）**。输入版本号，或直接回车安装最新正式版。

安装时会覆盖开机启动配置，保留设备标定、模型和动作组。如果本地文件与新版本冲突，更新会停止并提示，原文件会保留。处理问题后可重新执行更新命令。

## 在电脑上交叉编译

以下步骤适用于 x86-64 Ubuntu 电脑，以及已配置 WonderDuck 镜像的设备。在电脑上安装 Git、curl、rsync、pkg-config、带 `venv` 的 Python 3、C 编译器和 Node.js 20，再克隆仓库。

```bash
git clone --branch main --recurse-submodules https://gitee.com/hiwonder_official/wonderduck.git
cd wonderduck
```

安装 Rust、ARM64 目标、cargo-zigbuild 和 Zig。

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

部署本机编译结果前，先在 Studio 点击「休息」并等待完成，再回到电脑上用于编译的终端执行以下命令。源码同步只包含 Git 已跟踪的文件。部署会覆盖程序文件，并保留设备上的模型、动作组、标定和 `/etc/robot`。

```bash
(
set -euo pipefail
test -s extensions/wonderduck_studio/console/web/build/index.html
test -s extensions/wonderduck_studio/console/web/build/debug.html
ssh "$robot" 'sudo systemctl stop robotd'
git ls-files scripts/ assets/prompt-sounds/network/ | rsync -a --files-from=- ./ "${robot}:/home/wonderduck/wonderduck/"
git -C extensions/wonderduck_studio ls-files console/ webpadd/web/ | rsync -a --files-from=- extensions/wonderduck_studio/ "${robot}:/home/wonderduck/wonderduck/extensions/wonderduck_studio/"
rsync -a extensions/wonderduck_studio/console/web/build/ "${robot}:/home/wonderduck/wonderduck/extensions/wonderduck_studio/console/web/build/"
scp target/aarch64-unknown-linux-gnu/release/{robotd,padd,robotctl} extensions/wonderduck_studio/webpadd/target/aarch64-unknown-linux-gnu/release/webpadd "${robot}:/home/wonderduck/wonderduck/target/release/"
scp assets/prompt-sounds/*.wav "${robot}:/home/wonderduck/wonderduck/target/prompt-sounds/"
ssh "$robot" 'sudo systemctl restart wonderduck-prompt-sounds && sudo systemctl start robotd'
)
```

如果部署失败，机器人服务可能保持停止。请排除错误后重新部署。如需更新固件、驱动或系统服务配置，请单独操作。若之后安装正式版时提示设备上的本地文件有冲突，请先处理冲突再重试。

## 在实机上编译和部署

在 RK3566 实机上，从 Gitee 更新现有的 `~/wonderduck` 仓库及 Studio 子模块。

```bash
(
set -euo pipefail
cd "$HOME/wonderduck"
test -z "$(git status --porcelain --untracked-files=no --ignore-submodules=all)"
test -z "$(git -C extensions/wonderduck_studio status --porcelain --untracked-files=no)"
git remote set-url origin https://gitee.com/hiwonder_official/wonderduck.git
git fetch --no-tags --no-recurse-submodules origin main
git merge-base --is-ancestor HEAD FETCH_HEAD
git merge --ff-only --no-overwrite-ignore FETCH_HEAD
git submodule sync --recursive
git submodule update --init --recursive --checkout
)
```

如果任一仓库有已跟踪文件的本地修改，或主仓无法快进，命令会停止更新。

首次编译前，安装系统依赖、Rust 1.99.0，以及带 npm 的 ARM64 Node.js 22。 [Node.js 下载](https://nodejs.org/en/download)

```bash
sudo apt-get update && sudo apt-get install -y build-essential pkg-config libudev-dev rsync
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain 1.99.0
```

将程序和 Studio 网页编译到 `target/native-build`。内存为 1 GB 且未启用交换空间的设备，执行时会启用 2 GiB 临时交换空间并在结束时清理。

```bash
(
set -euo pipefail
cd "$HOME/wonderduck"
. "$HOME/.cargo/env"
export CARGO_TARGET_DIR="$PWD/target/native-build"
export DUCK_REVISION="$(git rev-parse HEAD)"
export CARGO_NET_GIT_FETCH_WITH_CLI=true
if [ "$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)" -le 1048576 ] &&
   [ "$(awk '/^SwapTotal:/ {print $2}' /proc/meminfo)" -eq 0 ]; then
  mkdir -p "$CARGO_TARGET_DIR"
  swap_file=$(mktemp "$CARGO_TARGET_DIR/.swap.XXXXXX")
  trap 'if ! grep -qF "$swap_file" /proc/swaps || sudo swapoff "$swap_file"; then unlink "$swap_file"; fi' EXIT
  fallocate -l 2G "$swap_file"
  sudo chown root:root "$swap_file"
  sudo mkswap "$swap_file"
  sudo swapon "$swap_file"
fi
cargo +1.99.0 build --release --locked -j 1 -p robotd -p padd -p robotctl --bins
cargo +1.99.0 build --release --locked -j 1 --manifest-path extensions/wonderduck_studio/webpadd/Cargo.toml --bin webpadd
cd extensions/wonderduck_studio/console/web
npm ci
./node_modules/.bin/vite build --outDir "$CARGO_TARGET_DIR/web" --emptyOutDir
)
```

编译成功后，在网页点击「休息」并等待完成，再执行下方命令部署程序和网页。此步骤使用系统镜像已有的媒体运行库。

```bash
(
set -euo pipefail
cd "$HOME/wonderduck"
build="$PWD/target/native-build"
for program in robotd padd robotctl webpadd; do test -x "$build/release/$program"; done
test -s "$build/web/index.html"
test -s "$build/web/debug.html"
sudo systemctl stop robotd
install -m 0755 "$build"/release/{robotd,padd,robotctl,webpadd} target/release/
rsync -a --delete "$build/web/" extensions/wonderduck_studio/console/web/build/
install -m 0644 assets/prompt-sounds/*.wav target/prompt-sounds/
sudo systemctl restart wonderduck-prompt-sounds
sudo systemctl start robotd
systemctl is-active robotd
)
```

打开 Studio 验证部署，再清理 `target/native-build` 中的编译产物。

## 恢复状态读取故障

设备状态读取发生串口故障时，机器人会关闭运动控制，尝试恢复通信并逐个确认舵机卸力。如果故障持续，请检查舵机和 MCU 的连接及供电，再重新打开设备状态页；仍无法恢复时请联系技术支持。
