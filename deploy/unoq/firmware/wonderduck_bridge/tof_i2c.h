#pragma once

class ExpansionSoftI2c {
public:
    ExpansionSoftI2c(GPIO_TypeDef *port, uint32_t scl, uint32_t sda)
        : port_(port), scl_(scl), sda_(sda) {}

    void begin() const {
        for (uint32_t line : {scl_, sda_}) {
            release(line);
            LL_GPIO_SetPinOutputType(port_, line, LL_GPIO_OUTPUT_OPENDRAIN);
            LL_GPIO_SetPinMode(port_, line, LL_GPIO_MODE_OUTPUT);
        }
    }

    bool transfer(uint8_t address, const uint8_t *output, size_t written,
                  uint8_t *input, size_t read) const {
        bool okay = start();
        if (written || !read) {
            okay = okay && write_byte(address << 1);
            for (size_t index = 0; index < written && okay; ++index) {
                okay = write_byte(output[index]);
            }
        }
        if (read) {
            okay = okay && (!written || start()) && write_byte((address << 1) | 1);
            for (size_t index = 0; index < read && okay; ++index) {
                okay = read_byte(input[index], index + 1 == read);
            }
        }
        stop();
        return okay;
    }

private:
    GPIO_TypeDef *port_;
    uint32_t scl_;
    uint32_t sda_;

    void release(uint32_t line) const { LL_GPIO_SetOutputPin(port_, line); }
    void low(uint32_t line) const { LL_GPIO_ResetOutputPin(port_, line); }
    static void pause() { k_busy_wait(5); }

    bool raise_clock() const {
        release(scl_);
        uint32_t started = micros();
        while (!LL_GPIO_IsInputPinSet(port_, scl_)) {
            if (micros() - started >= 1000) {
                return false;
            }
        }
        pause();
        return true;
    }

    bool start() const {
        release(sda_);
        pause();
        if (!raise_clock() || !LL_GPIO_IsInputPinSet(port_, sda_)) {
            return false;
        }
        low(sda_);
        pause();
        low(scl_);
        return true;
    }

    void stop() const {
        low(sda_);
        pause();
        raise_clock();
        release(sda_);
        pause();
    }

    bool clock_bit(bool send_high, bool &read_high) const {
        if (send_high) {
            release(sda_);
        } else {
            low(sda_);
        }
        pause();
        bool okay = raise_clock();
        read_high = LL_GPIO_IsInputPinSet(port_, sda_);
        low(scl_);
        return okay;
    }

    bool write_byte(uint8_t value) const {
        bool high;
        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            if (!clock_bit(value & mask, high)) {
                return false;
            }
        }
        return clock_bit(true, high) && !high;
    }

    bool read_byte(uint8_t &value, bool last) const {
        value = 0;
        bool high;
        for (unsigned index = 0; index < 8; ++index) {
            if (!clock_bit(true, high)) {
                return false;
            }
            value = (value << 1) | high;
        }
        return clock_bit(last, high);
    }
};

// JMISC pins 22 and 24 carry SCL0 and SDA0 to the TCA9546 at 0x71.
class TofI2c {
public:
    static void begin() {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        bus().begin();
    }

    static bool transfer(const uint8_t *output, size_t written, uint8_t *input, size_t read) {
        constexpr uint8_t mux = 0x71;
        constexpr uint32_t probe_interval_ms = 1000;
        static uint8_t channel = 0;
        static uint32_t last_probe = millis() - probe_interval_ms;
        if (channel == 0 && millis() - last_probe >= probe_interval_ms) {
            last_probe = millis();
            const uint8_t detected = probe_channels(mux);
            for (uint8_t mask = 1; mask <= 8; mask <<= 1) {
                if (detected & mask) {
                    channel = mask;
                    break;
                }
            }
        }
        if (channel == 0) {
            return false;
        }
        if (bus().transfer(mux, &channel, 1, nullptr, 0) &&
            bus().transfer(0x29, output, written, input, read)) {
            return true;
        }
        channel = 0;
        return false;
    }

    static uint8_t probe_muxes() {
        uint8_t present = 0;
        for (uint8_t index = 0; index < 8; ++index) {
            uint8_t control;
            if (bus().transfer(0x70 + index, nullptr, 0, &control, 1)) {
                present |= 1u << index;
            }
        }
        return present;
    }

    static uint8_t probe_channels(uint8_t mux) {
        // Keep the previous channel selection after probing device identities.
        uint8_t previous;
        if (mux < 0x70 || mux > 0x77 || !bus().transfer(mux, nullptr, 0, &previous, 1)) {
            return 0;
        }
        uint8_t present = 0;
        for (uint8_t index = 0; index < 4; ++index) {
            const uint8_t channel = 1u << index;
            const uint8_t identity_register = 0xc0;
            uint8_t identity;
            if (bus().transfer(mux, &channel, 1, nullptr, 0) &&
                bus().transfer(0x29, &identity_register, 1, &identity, 1)) {
                present |= 1u << index;
            }
        }
        bus().transfer(mux, &previous, 1, nullptr, 0);
        return present;
    }

private:
    static ExpansionSoftI2c bus() {
        return {GPIOA, LL_GPIO_PIN_0, LL_GPIO_PIN_1};
    }
};

class HeadImuI2c {
public:
    static void begin() {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
        bus().begin();
    }

    static bool transfer(uint8_t address, const uint8_t *output, size_t written,
                         uint8_t *input, size_t read) {
        return bus().transfer(address, output, written, input, read);
    }

    static void probe(uint8_t *data) {
        const uint8_t identity_register = 0x00;
        for (uint8_t index = 0; index < 2; ++index) {
            if (!transfer(0x6a + index, &identity_register, 1, data + index, 1)) {
                data[index] = 0xff;
            }
        }
    }

private:
    static ExpansionSoftI2c bus() {
        return {GPIOF, LL_GPIO_PIN_14, LL_GPIO_PIN_15};
    }
};
