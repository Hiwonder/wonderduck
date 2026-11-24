# UNO Q 行人检测 ncnn 后端

使用带 Vulkan 的 ncnn 20261020 和 PNNX 20261020。下面的命令在 ARM64 构建环境中运行，`NCNN_SOURCE` 指向包含 glslang 的 ncnn 完整源码，`WONDERDUCK_ROOT` 指向 WonderDuck 根目录。

```sh
cmake -S "$NCNN_SOURCE" -B build/ncnn \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/build/ncnn-install" \
  -DNCNN_VULKAN=ON -DNCNN_ARM82=OFF \
  -DNCNN_BUILD_TOOLS=OFF -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_BUILD_BENCHMARK=OFF
cmake --build build/ncnn --target install
cmake -S "$WONDERDUCK_ROOT/deploy/unoq/gpu-runner" -B build/gpu-runner \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/build/ncnn-install"
cmake --build build/gpu-runner
```

将 YOLOv6n 512×512 的行人检测 ONNX 源文件命名为 `person-yolov6n-512.onnx`。模型输入为 `[1,3,512,512]`，输出为 `person_boxes` `[1,5376,4]` 和 `person_scores` `[1,1,5376]`。

```sh
install -m 0755 build/gpu-runner/object-gpu-runner "$WONDERDUCK_ROOT/target/release/"
cd "$WONDERDUCK_ROOT/models"
pnnx person-yolov6n-512.onnx 'inputshape=[1,3,512,512]' fp16=0 \
  ncnnparam=person-yolov6n-512.param ncnnbin=person-yolov6n-512.bin
```

UNO Q 的 `run-robotd.sh` 将推理进程路径传给 Studio。行人跟随固定使用 YOLOv6n 512×512，Studio 以 `yolov6n-512x512 --cpu` 启动 ncnn 推理进程，进程使用两个 CPU 推理线程。启动成功时，日志会显示 `yolov6n-512x512 uses ncnn CPU with 2 threads`。
