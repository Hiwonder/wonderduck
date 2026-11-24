# WonderDuck

English | [简体中文](README_zh.md)

WonderDuck provides robot control software and WonderDuck Studio. It is based on Pollen Robotics Microduck. [WonderDuck Studio on GitHub](https://github.com/Hiwonder/wonderduck_studio) · [Microduck](https://github.com/pollen-robotics/microduck)

## Update and deployment

On a configured device, open **System settings → Software version → Check for updates** in Studio. The English interface checks GitHub and the Chinese interface checks Gitee. Confirm the version to start the update. Studio shows download progress, installation, and recovery if installation or startup fails.

On a device with a configured system image, select **Rest** and wait for it to finish. Run the following command in the device terminal.

```bash
python3 <(curl -fsSL --connect-timeout 10 https://raw.githubusercontent.com/Hiwonder/wonderduck/main/scripts/update.py) --lang en
```

Select **1 (GitHub)**. Enter a version or press Enter for the latest stable release.

Installation updates the startup configuration and preserves device calibration, models, and action groups. If a local file conflicts with the new version, the update stops and keeps the original file. Resolve the conflict and run the commands again.

Studio provides control over the local network. Connect the robot to a trusted network.

## Cross-compile on a computer

When adding or changing user-facing text, maintain and verify the Simplified Chinese and English versions together. Include control labels, dialogs, status messages, and error explanations.

These steps are for an x86-64 Ubuntu computer and a robot with a configured WonderDuck image. On the computer, install Git, curl, rsync, pkg-config, Python 3 with `venv`, a C compiler and Node.js 20, then clone the repository.

```bash
git clone --branch main --recurse-submodules https://github.com/Hiwonder/wonderduck.git
cd wonderduck
```

Install Rust, the ARM64 target, cargo-zigbuild and Zig.

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
test -s extensions/wonderduck_studio/console/web/build/index.html
test -s extensions/wonderduck_studio/console/web/build/debug.html
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

## Build and deploy on the robot

On the RK3566 robot, update the existing `~/wonderduck` checkout and its Studio submodule from GitHub.

```bash
(
set -euo pipefail
cd "$HOME/wonderduck"
test -z "$(git status --porcelain --untracked-files=no --ignore-submodules=all)"
test -z "$(git -C extensions/wonderduck_studio status --porcelain --untracked-files=no)"
git remote set-url origin https://github.com/Hiwonder/wonderduck.git
git fetch --no-tags --no-recurse-submodules origin main
git merge-base --is-ancestor HEAD FETCH_HEAD
git merge --ff-only --no-overwrite-ignore FETCH_HEAD
git submodule sync --recursive
git submodule update --init --recursive --checkout
)
```

The source update stops if either repository has tracked changes or the main checkout cannot fast-forward.

Before the first build, install the system dependencies, Rust 1.99.0, and ARM64 Node.js 22 with npm. [Node.js downloads](https://nodejs.org/en/download)

```bash
sudo apt-get update && sudo apt-get install -y build-essential pkg-config libudev-dev rsync
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain 1.99.0
```

Build the programs and Studio web interface in `target/native-build`. On devices with 1 GB of RAM and no active swap, this step creates a temporary 2 GiB swap file for compilation and removes it on exit.

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

Once the build succeeds, select **Rest** and wait for it to finish. Deploy the programs and web interface with the commands below. This uses the media runtime installed by the system image.

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
install -m 0644 assets/prompt-sounds/{startup,low_battery}.wav target/prompt-sounds/
sudo systemctl restart wonderduck-prompt-sounds
sudo systemctl start robotd
systemctl is-active robotd
)
```

Open Studio to verify the deployment, then remove the build output in `target/native-build`.

## Recover from a device status read failure

If a device status read encounters a serial fault, motion control is disabled while the robot attempts to restore communication and verify each servo is released. If the fault persists, check the servo and MCU connections and power, then reopen the device status page. Contact support if recovery still fails.
