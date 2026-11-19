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

## Cross-compile on a computer

These steps are for an x86-64 Ubuntu computer and a robot with an already configured WonderDuck image. Install Git, curl, rsync, pkg-config, Python 3 with `venv`, a C compiler and Node.js 20 first. On the computer, clone either source:

```bash
git clone --recurse-submodules https://github.com/Hiwonder/wonderduck.git
# Alternatively: git clone --recurse-submodules https://gitee.com/hiwonder_official/wonderduck.git
cd wonderduck
```

To work on the development branch, add `-b develop` to the clone command. Install Rust, the ARM64 target, cargo-zigbuild and Zig:

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

Replace the example IP with the robot's IP. Copy its ARM64 `libudev` for linking and build the programs and web interface:

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

To deploy this local build, select **Rest** on the robot and wait for it to finish, then run the following in the same computer terminal. Newly added source files must first be tracked by Git. These commands replace program files but preserve models, action groups, calibration and `/etc/robot`:

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

If a command fails, the robot service may remain stopped; resolve the error and run the deployment again. This developer deployment does not update firmware, drivers or system service configuration. It can leave the device's Git checkout modified; resolve any conflicts reported by the official updater before installing a release.

## Recover from a device status read failure

If a device status read encounters a serial fault, motion control is disabled while the robot attempts to restore communication and verify each servo is released. If the fault persists, check the servo and MCU connections and power, then reopen the device status page. Contact support if recovery still fails.
