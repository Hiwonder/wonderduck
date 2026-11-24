#include <Arduino_RouterBridge.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32u5xx_ll_bus.h>
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_rcc.h>
#include <stm32u5xx_ll_usart.h>
#include <cstring>
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
constexpr size_t hx_max_frame_size = 255 + 4;
constexpr uint8_t feedback_ids[] = {20, 21, 22, 23, 24, 30, 31, 32, 33, 34,
                                    10, 11, 12, 13, 14, 200};
constexpr uint32_t feedback_period_us = 10000;
constexpr uint32_t feedback_lease_us = 1000000;
constexpr uint32_t feedback_max_age_us = 20000;

k_mutex bus_mutex, feedback_mutex;
k_sem feedback_start, feedback_ready;
K_THREAD_STACK_DEFINE(feedback_stack, 4096);
k_thread feedback_thread;
Bytes feedback_request, feedback_latest;
uint32_t feedback_sequence = 0, feedback_sample_started = 0, feedback_lease_until = 0;
uint32_t feedback_generation = 0;
bool feedback_enabled = false;

// The expansion board uses D18 to select the transmit path. D18 is PC1 on UNO Q.
constexpr uint32_t direction_pin = LL_GPIO_PIN_1;

void enable_router_fifo() {
    if (bridge_baud < 2000000) return;
    ARDUINO_ROUTER_SERIAL.begin(bridge_baud);
    auto *port = reinterpret_cast<USART_TypeDef *>(DT_REG_ADDR(ARDUINO_ROUTER_PHANDLE));
    const unsigned key = irq_lock();
    const uint32_t config = port->CR1;
    LL_USART_Disable(port);
    LL_USART_EnableFIFO(port);
    port->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    port->CR1 = config | USART_CR1_FIFOEN;
    irq_unlock(key);
}

void init_servo_bus() {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB | LL_AHB2_GRP1_PERIPH_GPIOD |
                            LL_AHB2_GRP1_PERIPH_GPIOC);
    // The core's USART3 driver supplies interrupt-driven RX buffering. Its
    // default pinctrl uses PB10/PB11, while this board routes USART3 to PD8/PD9.
    const uint32_t mode = GPIOB->MODER, pull = GPIOB->PUPDR, speed = GPIOB->OSPEEDR;
    const uint32_t type = GPIOB->OTYPER, alternate = GPIOB->AFR[1];
    Serial3.begin(servo_baud);
    constexpr uint32_t pair_mask = (3u << 20) | (3u << 22);
    constexpr uint32_t pin_mask = (1u << 10) | (1u << 11);
    constexpr uint32_t af_mask = (15u << 8) | (15u << 12);
    GPIOB->MODER = (GPIOB->MODER & ~pair_mask) | (mode & pair_mask);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~pair_mask) | (pull & pair_mask);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~pair_mask) | (speed & pair_mask);
    GPIOB->OTYPER = (GPIOB->OTYPER & ~pin_mask) | (type & pin_mask);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~af_mask) | (alternate & af_mask);
    LL_GPIO_ResetOutputPin(GPIOC, direction_pin);
    LL_GPIO_SetPinMode(GPIOC, direction_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, direction_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetAFPin_8_15(GPIOD, servo_tx, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_8_15(GPIOD, servo_rx, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOD, servo_tx, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinMode(GPIOD, servo_tx, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOD, servo_rx, LL_GPIO_MODE_ALTERNATE);

    const unsigned key = irq_lock();
    const uint32_t config = USART3->CR1;
    LL_USART_Disable(USART3);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
    LL_USART_SetBaudRate(USART3, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
                         LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, servo_baud);
    LL_USART_EnableFIFO(USART3);
    USART3->CR1 = config | USART_CR1_FIFOEN;
    irq_unlock(key);
}

Bytes servo_exchange(Bytes request) {
    Bytes reply;
    if (request.size() < 6 || request.size() > 259 || request[0] != 0xff ||
        request[1] != 0xff || request[3] + 4u != request.size()) {
        return reply;
    }

    const bool sync_read = request[2] == 0xfe && request[4] == hx_sync_read;
    size_t reply_limit = sync_read && request.size() >= 9
        ? min(max_bus_bytes, (request.size() - 8) * (hx_status_overhead + request[6]))
        : hx_max_frame_size;
    reply.reserve(reply_limit);
    const unsigned key = irq_lock();
    LL_USART_RequestRxDataFlush(USART3);
    LL_USART_ClearFlag_ORE(USART3);
    irq_unlock(key);
    while (Serial3.available()) Serial3.read();
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
    while (reply.size() < reply_limit) {
        if (LL_USART_IsActiveFlag_ORE(USART3)) {
            LL_USART_ClearFlag_ORE(USART3);
            return {};
        }
        if (Serial3.available()) {
            reply.push_back(static_cast<uint8_t>(Serial3.read()));
            // ID200 calibration writes can acknowledge with data. Use the
            // unicast status header's length, including for short error replies.
            if (!sync_read && reply.size() == 4 && reply[0] == 0xff && reply[1] == 0xff &&
                reply[2] == request[2] && reply[3] >= 2) {
                reply_limit = size_t(reply[3]) + 4;
            }
            last_byte = micros();
        } else if (micros() - started >= 12000 ||
                   (!reply.empty() && micros() - last_byte >= 1500)) {
            if (Serial3.available()) continue;
            break;
        }
    }
    return reply;
}

Bytes locked_servo_exchange(Bytes request) {
    k_mutex_lock(&bus_mutex, K_FOREVER);
    Bytes reply = servo_exchange(request);
    k_mutex_unlock(&bus_mutex);
    return reply;
}

void feedback_acquire(void *, void *, void *) {
    for (;;) {
        k_sem_take(&feedback_start, K_FOREVER);
        k_mutex_lock(&feedback_mutex, K_FOREVER);
        const uint32_t generation = feedback_generation;
        k_mutex_unlock(&feedback_mutex);
        uint32_t deadline = micros();
        for (;;) {
            k_mutex_lock(&feedback_mutex, K_FOREVER);
            if (!feedback_enabled || generation != feedback_generation ||
                int32_t(micros() - feedback_lease_until) >= 0) {
                if (generation == feedback_generation) feedback_enabled = false;
                k_mutex_unlock(&feedback_mutex);
                break;
            }
            k_mutex_unlock(&feedback_mutex);

            k_mutex_lock(&bus_mutex, K_FOREVER);
            const uint32_t started = micros();
            Bytes reply = servo_exchange(feedback_request);
            k_mutex_lock(&feedback_mutex, K_FOREVER);
            if (feedback_enabled && generation == feedback_generation) {
                feedback_latest.swap(reply);
                feedback_sample_started = started;
                if (++feedback_sequence == 0) ++feedback_sequence;
                k_sem_give(&feedback_ready);
            }
            k_mutex_unlock(&feedback_mutex);
            k_mutex_unlock(&bus_mutex);

            deadline += feedback_period_us;
            const int32_t remaining = int32_t(deadline - micros());
            if (remaining > 0) {
                k_sleep(K_USEC(remaining));
            } else {
                deadline = micros();
            }
        }
    }
}

void feedback_begin() {
    k_mutex_lock(&feedback_mutex, K_FOREVER);
    feedback_lease_until = micros() + feedback_lease_us;
    if (!feedback_enabled) {
        feedback_enabled = true;
        ++feedback_generation;
        feedback_latest.clear();
        k_sem_reset(&feedback_ready);
        k_sem_give(&feedback_start);
    }
    k_mutex_unlock(&feedback_mutex);
}

Bytes feedback_read(uint32_t previous_sequence) {
    const uint32_t until = micros() + 18000;
    feedback_begin();

    for (;;) {
        k_mutex_lock(&feedback_mutex, K_FOREVER);
        const uint32_t age = micros() - feedback_sample_started;
        if (!feedback_latest.empty() && feedback_sequence != previous_sequence &&
            age <= feedback_max_age_us) {
            const uint32_t header[] = {feedback_sequence, age};
            Bytes result(sizeof(header) + feedback_latest.size());
            std::memcpy(result.data(), header, sizeof(header));
            std::memcpy(result.data() + sizeof(header), feedback_latest.data(), feedback_latest.size());
            k_mutex_unlock(&feedback_mutex);
            return result;
        }
        k_mutex_unlock(&feedback_mutex);
        const int32_t remaining = int32_t(until - micros());
        if (remaining <= 0 || k_sem_take(&feedback_ready, K_USEC(remaining)) != 0) return {};
    }
}

int feedback_stop() {
    k_mutex_lock(&feedback_mutex, K_FOREVER);
    feedback_enabled = false;
    ++feedback_generation;
    feedback_latest.clear();
    k_mutex_unlock(&feedback_mutex);
    // A direct maintenance transaction must follow any scan already in progress.
    k_mutex_lock(&bus_mutex, K_FOREVER);
    k_mutex_unlock(&bus_mutex);
    return 0;
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
    k_mutex_init(&bus_mutex);
    k_mutex_init(&feedback_mutex);
    k_sem_init(&feedback_start, 0, 1);
    k_sem_init(&feedback_ready, 0, 1);
    feedback_request = {0xff, 0xff, 0xfe, uint8_t(sizeof(feedback_ids) + 4),
                        hx_sync_read, 56, 12};
    feedback_request.insert(feedback_request.end(), feedback_ids, feedback_ids + sizeof(feedback_ids));
    uint8_t sum = 0;
    for (size_t i = 2; i < feedback_request.size(); ++i) sum += feedback_request[i];
    feedback_request.push_back(~sum);
    k_thread_create(&feedback_thread, feedback_stack, K_THREAD_STACK_SIZEOF(feedback_stack),
                    feedback_acquire, nullptr, nullptr, nullptr, 6, 0, K_NO_WAIT);
    HeadImuI2c::begin();
    TofI2c::begin();
    enable_router_fifo();
    if (Bridge.begin(bridge_baud)) {
        Bridge.provide("wonderduck/version", +[]() -> int { return 1; });
        Bridge.provide("wonderduck/echo", +[](Bytes input) -> Bytes { return input; });
        Bridge.provide("wonderduck/link_test", +[](uint16_t count) -> Bytes {
            return count <= max_bus_bytes ? Bytes(count, 0xaa) : Bytes{};
        });
        Bridge.provide("wonderduck/servo", locked_servo_exchange);
        // Version 2 provides feedback only in replies to Linux requests.
        Bridge.provide("wonderduck/feedback_version", +[]() -> int { return 2; });
        Bridge.provide("wonderduck/feedback", feedback_read);
        Bridge.provide("wonderduck/feedback_stop", feedback_stop);
        Bridge.provide("wonderduck/feedback_sequence", +[]() -> uint32_t {
            k_mutex_lock(&feedback_mutex, K_FOREVER);
            const uint32_t sequence = feedback_enabled ? feedback_sequence : 0;
            k_mutex_unlock(&feedback_mutex);
            return sequence;
        });
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
    asm volatile("nop" ::: "memory");
}

void loop() {
    k_msleep(20);
}
