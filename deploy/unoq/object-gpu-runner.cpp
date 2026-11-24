#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string_view>
#include <vector>

#include <gpu.h>
#include <layer.h>
#include <net.h>

#if !NCNN_VULKAN
#error The object GPU runner requires an ncnn build with Vulkan enabled.
#endif

namespace {

constexpr float rgb_scale = 1.f / 255.f;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

struct TensorShape {
    int width;
    int height;
    int planes;
};

struct Model {
    const char* name;
    int width;
    int height;
    std::size_t output_count;
    std::array<TensorShape, 3> outputs;
};

constexpr Model yolo26n{
    "yolo26n", 640, 640, 2, {{{8400, 80, 1}, {8400, 4, 1}}},
};
constexpr Model yolov5s_relu_wide{
    "yolov5s-relu-640x384", 640, 384, 3, {{{18, 80, 48}, {18, 40, 24}, {18, 20, 12}}},
};
constexpr Model yolov6n{
    "yolov6n-512x512", 512, 512, 2, {{{4, 5376, 1}, {5376, 1, 1}}},
};

int adreno_device()
{
    for (int index = 0; index < ncnn::get_gpu_count(); ++index) {
        const auto& device = ncnn::get_gpu_info(index);
        const std::string_view name = device.device_name();
        const auto type = device.physicalDeviceProperties().deviceType;
        if ((type == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU || type == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
            && (name.find("Adreno") != name.npos || name.find("FD702") != name.npos)
            && name.find("702") != name.npos) {
            return index;
        }
    }
    return -1;
}

bool validate_layers(const ncnn::Net& net)
{
    // ncnn's parameter format uses bit 4 to disable Vulkan for an individual layer.
    constexpr int disable_vulkan = 1 << 4;
    for (const auto* layer : net.layers()) {
        if (layer->type != "Input" && (!layer->support_vulkan || (layer->featmask & disable_vulkan))) {
            std::fprintf(stderr, "Model layer %s (%s) cannot run in Vulkan\n",
                         layer->name.c_str(), layer->type.c_str());
            return false;
        }
    }
    return true;
}

bool matches_shape(const ncnn::Mat& tensor, const TensorShape& shape)
{
    const int dims = shape.planes == 1 ? 2 : 3;
    const bool dims_match = tensor.dims == dims
        || (tensor.dims == 1 && shape.height == 1 && shape.planes == 1);
    return dims_match && tensor.w == shape.width && tensor.h == shape.height && tensor.c == shape.planes
        && tensor.elempack == 1 && tensor.elemsize == sizeof(float);
}

bool write_tensor(const ncnn::Mat& tensor)
{
    const std::size_t plane_size = static_cast<std::size_t>(tensor.w) * tensor.h;
    if (tensor.c == 1 || tensor.cstep == plane_size) {
        const std::size_t elements = plane_size * tensor.c;
        return std::fwrite(tensor.data, sizeof(float), elements, stdout) == elements;
    }

    // ncnn may align each plane. The wire format contains only the tensor elements.
    for (int plane = 0; plane < tensor.c; ++plane) {
        if (std::fwrite(tensor.channel(plane).data, sizeof(float), plane_size, stdout) != plane_size) {
            return false;
        }
    }
    return true;
}

int run(const char* param_path, const char* bin_path, const Model& model, bool use_cpu)
{
    ncnn::Net net;
    net.opt.use_vulkan_compute = !use_cpu;
    net.opt.num_threads = use_cpu ? 2 : 1;
    int gpu = -1;
    if (!use_cpu) {
        gpu = adreno_device();
        if (gpu < 0) {
            std::fputs("No physical Adreno 702 Vulkan device found\n", stderr);
            return 1;
        }
        net.set_vulkan_device(gpu);
    }
    if (net.load_param(param_path) != 0 || net.load_model(bin_path) != 0) {
        std::fprintf(stderr, "Could not load ncnn model %s\n", model.name);
        return 1;
    }
    if (!use_cpu && !net.opt.use_vulkan_compute) {
        std::fprintf(stderr, "ncnn disabled Vulkan when loading %s\n", model.name);
        return 1;
    }
    if (!use_cpu && !validate_layers(net)) {
        return 1;
    }
    const auto& inputs = net.input_indexes();
    const auto& outputs = net.output_indexes();
    if (inputs.size() != 1 || outputs.size() != model.output_count) {
        std::fprintf(stderr, "Unexpected input or output count for %s\n", model.name);
        return 1;
    }

    ncnn::Mat input(model.width, model.height, 3);
    if (input.empty()) {
        std::fputs("Could not allocate the RGB input tensor\n", stderr);
        return 1;
    }
    float* red = input.channel(0);
    float* green = input.channel(1);
    float* blue = input.channel(2);
    const int image_pixels = model.width * model.height;
    std::vector<unsigned char> pixels(image_pixels * 3);

    if (use_cpu) {
        std::fprintf(stderr, "%s uses ncnn CPU with %d threads\n", model.name, net.opt.num_threads);
    } else {
        std::fprintf(stderr, "%s uses Vulkan GPU %s\n", model.name, ncnn::get_gpu_info(gpu).device_name());
    }
    if (std::fputs("READY\n", stdout) == EOF || std::fflush(stdout) == EOF) {
        return 1;
    }

    for (;;) {
        const size_t count = std::fread(pixels.data(), 1, pixels.size(), stdin);
        if (count != pixels.size()) {
            if (count == 0 && std::feof(stdin) && !std::ferror(stdin)) {
                return 0;
            }
            std::fputs("Could not read a complete RGB frame\n", stderr);
            return 1;
        }

        for (int i = 0; i < image_pixels; ++i) {
            red[i] = pixels[i * 3] * rgb_scale;
            green[i] = pixels[i * 3 + 1] * rgb_scale;
            blue[i] = pixels[i * 3 + 2] * rgb_scale;
        }
        auto extractor = net.create_extractor();
        if (extractor.input(inputs.front(), input) != 0) {
            std::fputs("Could not set the RGB input tensor\n", stderr);
            return 1;
        }
        std::array<ncnn::Mat, 3> tensors;
        for (std::size_t index = 0; index < outputs.size(); ++index) {
            if (extractor.extract(outputs[index], tensors[index]) != 0) {
                std::fprintf(stderr, "%s ncnn inference failed\n", model.name);
                return 1;
            }
        }

        std::array<const ncnn::Mat*, 3> ordered{};
        for (std::size_t index = 0; index < outputs.size(); ++index) {
            const auto tensor = std::find_if(tensors.begin(), tensors.end(), [&](const ncnn::Mat& item) {
                return matches_shape(item, model.outputs[index]);
            });
            if (tensor == tensors.end()) {
                std::fprintf(stderr, "Unexpected output shape for %s tensor %zu\n", model.name, index);
                return 1;
            }
            ordered[index] = &*tensor;
        }
        for (std::size_t index = 0; index < outputs.size(); ++index) {
            if (!write_tensor(*ordered[index])) {
                std::fputs("Could not write an output tensor\n", stderr);
                return 1;
            }
        }
        if (std::fflush(stdout) == EOF) {
            std::fputs("Could not flush the output tensors\n", stderr);
            return 1;
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5 || (argc == 5 && std::string_view(argv[4]) != "--cpu")) {
        std::fprintf(stderr, "Usage: %s MODEL.param MODEL.bin [yolo26n|yolov5s-relu-640x384|yolov6n-512x512 [--cpu]]\n", argv[0]);
        return 1;
    }
    const std::string_view kind = argc >= 4 ? argv[3] : yolo26n.name;
    const Model* model = nullptr;
    if (kind == yolo26n.name) {
        model = &yolo26n;
    } else if (kind == yolov5s_relu_wide.name) {
        model = &yolov5s_relu_wide;
    } else if (kind == yolov6n.name) {
        model = &yolov6n;
    } else {
        std::fprintf(stderr, "Unknown model kind %s\n", argv[3]);
        return 1;
    }
    if (argc == 5) {
        if (model != &yolov6n) {
            std::fputs("The CPU mode requires yolov6n-512x512\n", stderr);
            return 1;
        }
        return run(argv[1], argv[2], *model, true);
    }
    if (ncnn::create_gpu_instance() != 0) {
        std::fputs("Could not initialize the Vulkan runtime\n", stderr);
        return 1;
    }
    const int result = run(argv[1], argv[2], *model, false);
    ncnn::destroy_gpu_instance();
    return result;
}
