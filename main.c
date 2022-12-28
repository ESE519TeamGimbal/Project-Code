#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico_servo.pio.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "MPU6050.h"

//offsets x=2, y=0 z=0

#define MIN_DC 1200 //MIN SEEMS TO BE AROUND 1K MAX MAYBE AROUND 2.8K-3K?
#define MAX_DC 2400
const uint SERVO_PIN = 29;
const uint SERVO_PIN2 = 28;

#define SCL 23
#define SDA 22

int gyro0; //X READING
int gyro1; //Y READING
int gyro2; //Z READING


// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 100*1000);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
    int16_t accelerometer[3], gyro[3], temp;
    MPU6050_Reset();


    PIO pio = pio0;
    int sm = 0;
    int sm1 = 1;
    uint offset = pio_add_program(pio, &pico_servo_pio_program);

    float freq = 50.0f; // servo except 50Hz
    uint clk_div = 64;  // make the clock slower

    pico_servo_pio_program_init(pio, sm, offset, clk_div, SERVO_PIN);
    pico_servo_pio_program_init(pio, sm1, offset, clk_div, SERVO_PIN2);

    uint cycles = clock_get_hz(clk_sys) / (freq * clk_div);
    uint32_t period = (cycles -3) / 3;

    pio_pwm_set_period(pio, sm, period);
    pio_pwm_set_period(pio, sm1, period);

    uint level;
    uint level1;
    int ms = (MAX_DC - MIN_DC) / 2;
    int ms1 = (MAX_DC - MIN_DC) / 2;
    bool clockwise = false;
    bool clockwise1 = false;

  while (true) {
    level = (ms / 20000.f) * period;
    level1 = (ms1 / 20000.f) * period;
    pio_pwm_set_level(pio, sm, level);
    pio_pwm_set_level(pio, sm1, level1);
    MPU6050_ReadData(accelerometer, gyro, &temp);
    gyro0 = gyro[0]/131;
    gyro1 = gyro[1]/131;
    gyro2 = gyro[2]/131;

    if (ms <= MIN_DC) {
        MPU6050_ReadData(accelerometer, gyro, &temp);
        printf("Gyro X= %d  Y= %d   Z= %d ms= %d ms1= %d\r\n",gyro0, gyro1, gyro2, ms, ms1);
        ms = MIN_DC + 10;
        ms1 += gyro2/4;
    }
    if (ms >= MAX_DC) {
        MPU6050_ReadData(accelerometer, gyro, &temp);
        printf("Gyro X= %d  Y= %d   Z= %d ms= %d ms1= %d\r\n",gyro0, gyro1, gyro2, ms, ms1);
        ms = MAX_DC - 10;
        ms1 += gyro2/4;
    }
    if (ms1 <= MIN_DC) {
        MPU6050_ReadData(accelerometer, gyro, &temp);
        printf("Gyro X= %d  Y= %d   Z= %d ms= %d ms1= %d\r\n",gyro0, gyro1, gyro2, ms, ms1);
        ms += gyro0/4;
        ms1 = MIN_DC + 10;
    }
    if (ms1 >= MAX_DC) {
        MPU6050_ReadData(accelerometer, gyro, &temp);
        printf("Gyro X= %d  Y= %d   Z= %d ms= %d ms1= %d\r\n",gyro0, gyro1, gyro2, ms, ms1);
        ms += gyro0/4;
        ms1 = MAX_DC - 10;
    }
    else {
        MPU6050_ReadData(accelerometer, gyro, &temp);
        printf("Gyro X= %d  Y= %d   Z= %d ms= %d ms1= %d\r\n",gyro0, gyro1, gyro2, ms, ms1);
        ms += gyro0/4; //100; //gyro0;
        ms1 += gyro2/4;
    }

    sleep_ms(5);
  }
}