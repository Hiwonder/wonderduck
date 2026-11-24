/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace libcamera {

/* A CSI-2 RAW10 group has four high bytes followed by the four low-bit pairs. */
class Imx219RawDenoise
{
public:
	static constexpr unsigned int width = 1920, height = 1080;
	static constexpr unsigned int rowBytes = width / 4 * 5;

	explicit Imx219RawDenoise(float gain)
	{
		for (unsigned int bin = 0; bin < thresholds_.size(); ++bin) {
			/* Sensor noise above the 10-bit black level, in high-byte units. */
			const float signal = std::max(0, static_cast<int>(bin * 64 + 30) - 64);
			const float sigma = std::sqrt(4.0f + 0.18f * gain * signal) / 4.0f;
			thresholds_[bin] = std::max(3L, std::lround(3.2f * sigma));
		}
	}

	void apply(uint8_t *pixels, uint8_t *previous, unsigned int stride) const
	{
		static_assert(rowBytes % 16 == 0);
		for (unsigned int y = 0; y < height; ++y) {
			uint8_t *row = pixels + y * stride;
			uint8_t *history = previous + y * stride;
			for (unsigned int x = 0; x < rowBytes; x += 16)
				filterBlock(row + x, history + x, masks_[x % 5]);
		}
	}

private:
	using Block = std::array<uint8_t, 16>;
	static constexpr auto masks_ = [] {
		std::array<Block, 5> masks{};
		for (unsigned int phase = 0; phase < masks.size(); ++phase)
			for (unsigned int i = 0; i < masks[phase].size(); ++i)
				masks[phase][i] = (phase + i) % 5 == 4 ? 0 : 0xff;
		return masks;
	}();

	/* Always keep an unfiltered history so a moving edge cannot leave a long trail. */
	void filterBlock(uint8_t *pixels, uint8_t *history, const Block &highBytes) const
	{
#ifdef __aarch64__
		const uint8x16_t current = vld1q_u8(pixels);
		const uint8x16_t old = vld1q_u8(history);
		const uint8x16_t mask = vld1q_u8(highBytes.data());
		const uint8x16_t threshold = vqtbl1q_u8(vld1q_u8(thresholds_.data()),
							vshrq_n_u8(vmaxq_u8(current, old), 4));
		vst1q_u8(history, current);

		const int change = int(vaddlvq_u8(vandq_u8(current, mask))) -
				   int(vaddlvq_u8(vandq_u8(old, mask)));
		if (4 * std::abs(change) > vaddlvq_u8(vandq_u8(threshold, mask)))
			return;

		const uint8x16_t moving = vandq_u8(vcgtq_u8(vabdq_u8(current, old), threshold), mask);
		const uint8x16_t left = vextq_u8(vdupq_n_u8(0), moving, 15);
		const uint8x16_t right = vextq_u8(moving, vdupq_n_u8(0), 1);
		const uint8x16_t allow = vbicq_u8(mask, vorrq_u8(moving, vorrq_u8(left, right)));
		/* For odd differences round the average toward the current high byte. */
		const uint8x16_t round = vandq_u8(vandq_u8(current, veorq_u8(current, old)), vdupq_n_u8(1));
		const uint8x16_t mean = vaddq_u8(vhaddq_u8(current, old), round);
		vst1q_u8(pixels, vbslq_u8(allow, mean, current));
#else
		Block old;
		std::memcpy(old.data(), history, old.size());
		std::memcpy(history, pixels, old.size());
		int change = 0, limit = 0;
		std::array<bool, 18> moving{};
		for (unsigned int i = 0; i < old.size(); ++i) {
			if (!highBytes[i])
				continue;
			const int difference = int(pixels[i]) - old[i];
			const int threshold = thresholds_[std::max(pixels[i], old[i]) >> 4];
			change += difference;
			limit += threshold;
			moving[i + 1] = std::abs(difference) > threshold;
		}
		if (4 * std::abs(change) > limit)
			return;
		for (unsigned int i = 0; i < old.size(); ++i) {
			if (highBytes[i] && !moving[i] && !moving[i + 1] && !moving[i + 2]) {
				const int current = pixels[i];
				pixels[i] = ((current + old[i]) >> 1) + ((current ^ old[i]) & current & 1);
			}
		}
#endif
	}

	Block thresholds_{};
};

} /* namespace libcamera */
