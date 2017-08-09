#include "i2c_slave_interface.h"
#include "helpers.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>

#define TOSHIBALED_I2C_ADDRESS 0x55
#define AK09916_I2C_ADDRESS 0x0C
#define LEN(x) (sizeof(x)/sizeof(x[0]))

void i2c_slave_init(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_I2C2);
    i2c_reset(I2C2);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9|GPIO10);
    i2c_peripheral_disable(I2C2);
    i2c_enable_analog_filter(I2C2);
    i2c_set_digital_filter(I2C2, I2C_CR1_DNF_DISABLED);
    rcc_set_i2c_clock_sysclk(I2C2);
    i2c_set_prescaler(I2C2,8);
    i2c_set_data_setup_time(I2C2,9);
    i2c_set_data_hold_time(I2C2,11);
    i2c_enable_stretching(I2C2);
    i2c_set_7bit_addr_mode(I2C2);
    I2C_OAR1(I2C2) = (TOSHIBALED_I2C_ADDRESS&0xFF) << 1;
    I2C_OAR1(I2C2) |= (1<<15);
    I2C_OAR2(I2C2) = (AK09916_I2C_ADDRESS&0xFF) << 1;
    I2C_OAR2(I2C2) |= (1<<15);
    nvic_enable_irq(NVIC_I2C2_EV_EXTI24_IRQ);
    I2C_CR1(I2C2) |= (1<<1); // TXIE
    I2C_CR1(I2C2) |= (1<<2); // RXIE
    I2C_CR1(I2C2) |= (1<<3); // ADDRIE
    i2c_peripheral_enable(I2C2);
}

struct compass_register_s {
    const uint8_t address;
    const bool writeable;
    const bool read_locks_data;
    const int8_t next_address;
    uint8_t value;
};

static struct compass_register_s compass_registers[] = {
    {0x00, false, false, 0x01, 0x48},
    {0x01, false, false, 0x02, 0x09},
    {0x02, false, false, 0x03, 0x00},
    {0x03, false, false, 0x10, 0x00},
    {0x10, false, false, 0x11, 0x00},
    {0x11, false,  true, 0x12, 0x00},
    {0x12, false,  true, 0x13, 0x00},
    {0x13, false,  true, 0x14, 0x00},
    {0x14, false,  true, 0x15, 0x00},
    {0x15, false,  true, 0x16, 0x00},
    {0x16, false,  true, 0x17, 0x00},
    {0x17, false,  true, 0x18, 0x00},
    {0x18, false, false, 0x00, 0x00},
    {0x30,  true, false, 0x31, 0x00},
    {0x31,  true, false, 0x32, 0x00},
    {0x32,  true, false, 0x30, 0x00},
};

static bool compass_update_lock;
static struct compass_register_s* compass_curr_register = &compass_registers[0];
static uint8_t compass_new_data_counter;

static struct compass_register_s* get_compass_register(uint8_t address) {
    for(uint8_t i=0; i<LEN(compass_registers); i++) {
        if (compass_registers[i].address == address) {
            return &compass_registers[i];
        }
    }
    return 0;
}

// for now, it is assumed that this is called at 100hz
void i2c_slave_new_compass_data(float x, float y, float z, bool hofl) {
    struct compass_register_s* st1_reg = get_compass_register(0x10);
    struct compass_register_s* st2_reg = get_compass_register(0x18);
    struct compass_register_s* cntl2_reg = get_compass_register(0x31);

    bool update_now = false;

    switch(cntl2_reg->value&0x1F) {
        case 0b00001:
            update_now = true;
            break;
        case 0b00010: // 10Hz
            update_now = compass_new_data_counter%10 == 0;
            break;
        case 0b00100: // 20Hz
            update_now = compass_new_data_counter%5 == 0;
            break;
        case 0b00110: // 50Hz
            update_now = compass_new_data_counter%2 == 0;
            break;
        case 0b01000: // 100Hz
            update_now = true;
            break;
    }

    if (compass_update_lock) {
        st1_reg->value |= (1<<1);
        update_now = false;
    }

    if (update_now) {
        x = constrain_float(x*32752.0/49120.0, -32752, 32752);
        y = constrain_float(-y*32752.0/49120.0, -32752, 32752);
        z = constrain_float(-z*32752.0/49120.0, -32752, 32752);

        struct compass_register_s* meas_reg = get_compass_register(0x11);
        (meas_reg++)->value = (int16_t)x & 0xff;
        (meas_reg++)->value = ((int16_t)x >> 8) & 0xff;
        (meas_reg++)->value = (int16_t)y & 0xff;
        (meas_reg++)->value = ((int16_t)y >> 8) & 0xff;
        (meas_reg++)->value = (int16_t)z & 0xff;
        (meas_reg++)->value = ((int16_t)z >> 8) & 0xff;
        if (hofl) {
            st2_reg->value |= 1<<3; // HOFL
        }
        st1_reg->value |= 1; // DRDY

        if ((cntl2_reg->value&0x1F) == 1) {
            cntl2_reg->value = 0;
        }
    }

    compass_new_data_counter = (compass_new_data_counter+1) % 10;
}

static void set_compass_register_address(uint8_t address) {
    struct compass_register_s* next_compass_register = get_compass_register(address);
    if (next_compass_register) {
        compass_curr_register = next_compass_register;
    }
}

static void ak09916_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte) {
    if (recv_byte_idx == 0) {
        set_compass_register_address(recv_byte);
    } else {
        if (compass_curr_register->writeable) {
            compass_curr_register->value = recv_byte;
        }

        set_compass_register_address(compass_curr_register->next_address);
    }
}

static uint8_t ak09916_interface_send_byte(uint8_t transmit_byte_idx) {
    UNUSED(transmit_byte_idx);

    struct compass_register_s* st1_reg = get_compass_register(0x10);
    struct compass_register_s* accessed_reg = compass_curr_register;

    if (accessed_reg->read_locks_data && (st1_reg->value&(1<<1)) != 0) {
        compass_update_lock = true;
        st1_reg->value &= ~0b11;
    }

    if (accessed_reg->address == 0x18) {
        compass_update_lock = false;
    }

    set_compass_register_address(accessed_reg->next_address);

    return accessed_reg->value;
}

static uint32_t led_color_hex;
static uint8_t led_reg;

static void toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte) {
    if (recv_byte_idx == 0 || ((recv_byte&(1<<7)) != 0)) {
        led_reg = recv_byte & ~(1<<7);
    } else {
        switch(led_reg) {
            case 1:
                led_color_hex &= ~((uint32_t)0xff<<0);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<0;
                break;
            case 2:
                led_color_hex &= ~((uint32_t)0xff<<8);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<8;
                break;
            case 3:
                led_color_hex &= ~((uint32_t)0xff<<16);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<16;
                break;
        }
        led_reg++;
    }
}

uint32_t i2c_slave_retrieve_led_color_hex(void) {
    return led_color_hex;
}

static uint8_t i2c2_transfer_byte_idx;
static uint8_t i2c2_transfer_address;
static uint8_t i2c2_transfer_direction;

void i2c2_ev_exti24_isr(void) {
    if (I2C_ISR(I2C2) & (1<<3)) { // ADDR
        i2c2_transfer_address = (I2C_ISR(I2C2) >> 17) & 0x7FU; // ADDCODE
        i2c2_transfer_direction = (I2C_ISR(I2C2) >> 16) & 1; // direction
        i2c2_transfer_byte_idx = 0;
        if (i2c2_transfer_direction) {
            I2C_ISR(I2C2) |= (1<<0); // TXE
        }
        I2C_ICR(I2C2) |= (1<<3); // ADDRCF
    }

    if (i2c_received_data(I2C2)) {
        uint8_t recv_byte = i2c_get_data(I2C2); // reading clears our interrupt flag
        switch(i2c2_transfer_address) {
            case TOSHIBALED_I2C_ADDRESS:
                toshibaled_interface_recv_byte(i2c2_transfer_byte_idx, recv_byte);
                break;
            case AK09916_I2C_ADDRESS:
                ak09916_interface_recv_byte(i2c2_transfer_byte_idx, recv_byte);
                break;
        }
        i2c2_transfer_byte_idx++;
    }

    if (i2c_transmit_int_status(I2C2)) {
        uint8_t send_byte = 0;
        switch(i2c2_transfer_address) {
            case AK09916_I2C_ADDRESS:
                send_byte = ak09916_interface_send_byte(i2c2_transfer_byte_idx);
                break;
        }

        i2c_send_data(I2C2, send_byte);

        i2c2_transfer_byte_idx++;
    }
}
