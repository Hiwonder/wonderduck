#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32u5xx_ll_bus.h>
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_rcc.h>
#include <stm32u5xx_ll_usart.h>
#include "tof_i2c.h"

using Bytes = MsgPack::bin_t<uint8_t>;

namespace {
#ifndef WONDERDUCK_BRIDGE_BAUD
#define WONDERDUCK_BRIDGE_BAUD 1000000
#endif
constexpr unsigned long bridge_baud = WONDERDUCK_BRIDGE_BAUD;
constexpr uint32_t servo_baud = 1000000;
constexpr uint32_t servo_tx = LL_GPIO_PIN_8;
constexpr uint32_t servo_rx = LL_GPIO_PIN_9;
constexpr size_t max_bus_bytes = 1024;
constexpr size_t max_i2c_bytes = 64;
constexpr uint8_t hx_ping = 0x01;
constexpr uint8_t hx_read = 0x02;
constexpr uint8_t hx_write = 0x03;
constexpr uint8_t hx_sync_read = 0x82;
constexpr uint8_t hx_sync_write = 0x83;
constexpr size_t hx_status_overhead = 6;

// The expansion board uses D18 to select the transmit path. D18 is PC1 on UNO Q.
constexpr uint32_t direction_pin = LL_GPIO_PIN_1;

void init_servo_bus() {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD | LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_GPIO_ResetOutputPin(GPIOC, direction_pin);
    LL_GPIO_SetPinMode(GPIOC, direction_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, direction_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetAFPin_8_15(GPIOD, servo_tx, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_8_15(GPIOD, servo_rx, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, servo_tx, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinMode(GPIOD, servo_tx, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOD, servo_rx, LL_GPIO_MODE_ALTERNATE);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
    LL_USART_Disable(USART3);
    LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE,
                            LL_USART_STOPBITS_1);
    LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);
    LL_USART_SetBaudRate(USART3, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
                         LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, servo_baud);
    // A byte arrives every 10 us. Buffer replies while Zephyr services LPUART and timers.
    LL_USART_EnableFIFO(USART3);
    LL_USART_Enable(USART3);
}

size_t expected_reply_size(const Bytes &request) {
    switch (request[4]) {
    case hx_ping:
    case hx_write:
        return hx_status_overhead;
    case hx_read:
        return request.size() == 8 ? hx_status_overhead + request[6] : max_bus_bytes;
    case hx_sync_read:
        if (request.size() >= 9) {
            return min(max_bus_bytes, (request.size() - 8) * (hx_status_overhead + request[6]));
        }
        break;
    }
    return max_bus_bytes;
}

Bytes servo_exchange(Bytes request) {
    Bytes reply;
    if (request.size() < 6 || request.size() > 259 || request[0] != 0xff ||
        request[1] != 0xff || request[3] + 4u != request.size()) {
        return reply;
    }

    const size_t reply_limit = expected_reply_size(request);
    reply.reserve(reply_limit);
    LL_USART_RequestRxDataFlush(USART3);
    LL_USART_ClearFlag_ORE(USART3);
    LL_GPIO_SetOutputPin(GPIOC, direction_pin);
    uint32_t started = micros();
    for (uint8_t value : request) {
        while (!LL_USART_IsActiveFlag_TXE_TXFNF(USART3)) {
            if (micros() - started >= 4000) {
                LL_GPIO_ResetOutputPin(GPIOC, direction_pin);
                return reply;
            }
        }
        LL_USART_TransmitData8(USART3, value);
    }
    while (!LL_USART_IsActiveFlag_TC(USART3) && micros() - started < 4000) {}
    LL_GPIO_ResetOutputPin(GPIOC, direction_pin);

    if (request[4] == hx_sync_write ||
        (request[2] == 0xfe && request[4] != hx_sync_read)) {
        return reply;  // Broadcast sync write has no status reply.
    }
    started = micros();
    uint32_t last_byte = started;
    while (reply.size() < reply_limit && micros() - started < 12000) {
        if (LL_USART_IsActiveFlag_ORE(USART3)) {
            LL_USART_ClearFlag_ORE(USART3);
            return {};
        }
        if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART3)) {
            reply.push_back(LL_USART_ReceiveData8(USART3));
            last_byte = micros();
        } else if (!reply.empty() && micros() - last_byte >= 1500) {
            break;
        }
    }
    return reply;
}

Bytes head_imu_transfer(uint8_t address, Bytes request, uint8_t read_length) {
    Bytes result;
    if ((address != 0x6a && address != 0x6b) || request.empty() ||
        request.size() > max_i2c_bytes || read_length > max_i2c_bytes) {
        return result;
    }
    result.resize(read_length + 1);
    if (!HeadImuI2c::transfer(address, request.data(), request.size(),
                              result.data() + 1, read_length)) {
        return {};
    }
    result[0] = 0;
    return result;
}

Bytes tof_transfer(Bytes request, uint8_t read_length) {
    if (request.size() > max_i2c_bytes || read_length > max_i2c_bytes) {
        return {};
    }
    Bytes reply(read_length + 1);
    if (!TofI2c::transfer(request.data(), request.size(), reply.data() + 1, read_length)) {
        return {};
    }
    return reply;
}
}

void setup() {
    init_servo_bus();
    HeadImuI2c::begin();
    TofI2c::begin();
    if (Bridge.begin(bridge_baud)) {
        Bridge.provide("wonderduck/version", +[]() -> int { return 1; });
        Bridge.provide("wonderduck/echo", +[](Bytes input) -> Bytes { return input; });
        Bridge.provide("wonderduck/link_test", +[](uint16_t count) -> Bytes {
            return count <= max_bus_bytes ? Bytes(count, 0xaa) : Bytes{};
        });
        Bridge.provide("wonderduck/servo", servo_exchange);
        Bridge.provide("wonderduck/head_imu", head_imu_transfer);
        Bridge.provide("wonderduck/tof", tof_transfer);
        Bridge.provide("wonderduck/i2c_probe", +[]() -> Bytes {
            Bytes result(3);
            HeadImuI2c::probe(result.data());
            result[2] = TofI2c::probe_muxes();
            return result;
        });
        Bridge.provide("wonderduck/tof_channels", +[](uint8_t address) -> int {
            return TofI2c::probe_channels(address);
        });
    }
}

void loop() {
    delay(20);
}
