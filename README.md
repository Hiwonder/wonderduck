# WonderDuck

English | [简体中文](README_zh.md)

WonderDuck provides robot control software and [WonderDuck Studio](https://github.com/Hiwonder/wonderduck_studio).

## Update and deployment

Both update methods require a device with the system image installed and configured. Before updating, open **Network → Wi-Fi network** in Studio and connect the robot to a trusted Wi-Fi network with internet access.

### Studio 0.2.0 and later

In Studio, open **Settings → Software version → Check for updates**. Confirm the version to start the update. Studio shows download progress, installation, and recovery if installation or startup fails.

### Studio versions earlier than 0.2.0

Update from the device terminal. Select **Rest** in Studio, wait for it to finish, then run the following command in the device terminal.

```bash
python3 <(curl -fsSL --connect-timeout 10 https://raw.githubusercontent.com/Hiwonder/wonderduck/main/scripts/update.py) --lang en
```

Select **1 (GitHub)**. Enter a version or press Enter for the latest stable release.

Installation updates the startup configuration and preserves device calibration, models, and action groups. If a local file conflicts with the new version, the update stops and keeps the original file. Resolve the conflict and run the commands again.

## Cross-compile on a computer

The Gitee `develop` branch uses the same cross-compilation steps on x86-64 Ubuntu and WSL 2. [Gitee build instructions](README_zh.md#在电脑上交叉编译).

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

To deploy the local build, select **Rest** in Studio and wait for it to finish. Run the commands below in the same terminal you used to build the software.

Source synchronization copies only files tracked by Git. Deployment replaces program files and preserves device models, action groups, calibration, and `/etc/robot`.

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

If deployment fails, the robot service may remain stopped. Resolve the error and run the deployment again. Update firmware, drivers, or system service configuration separately as needed. If a later release update reports local file conflicts on the device, resolve them and retry.

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
install -m 0644 assets/prompt-sounds/*.wav target/prompt-sounds/
sudo systemctl restart wonderduck-prompt-sounds
sudo systemctl start robotd
systemctl is-active robotd
)
```

Open Studio to verify the deployment, then remove the build output in `target/native-build`.

## Recover from a device status read failure

If a device status read encounters a serial fault, motion control is disabled while the robot attempts to restore communication and verify each servo is released. If the fault persists, check the servo and MCU connections and power, then reopen the device status page. Contact support if recovery still fails.

## Acknowledgments

- [MicroDuck](https://github.com/pollen-robotics/microduck)
- [MjLab](https://github.com/mujocolab/mjlab)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [BAM](https://github.com/Rhoban/bam)
