#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define SAMPLE_PERIOD (0.01f)  // Período de amostragem ajustado

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // Inicializa o I2C e o MPU6050
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    
    // Inicializa a biblioteca Fusion para fusão de dados
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        // Lê os dados brutos do MPU6050
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converte os dados para as unidades adequadas para a fusão de dados
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,  // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,  // Conversão para gravidade (g)
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        // Atualiza a fusão de dados (sem magnetômetro)
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Converte os dados fundidos para ângulos de Euler
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Imprime os ângulos de Roll, Pitch e Yaw
        printf("Roll: %0.1f, Pitch: %0.1f, Yaw: %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        // Verifica se houve um movimento brusco para simular um "mouse click"
        if (accelerometer.axis.x > 1.5) {  // Threshold para detecção de movimento brusco
            printf("Mouse Click Detectado!\n");
        }

        // Atraso de 10ms entre as leituras
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    // Cria a tarefa para ler a IMU e processar os dados
    xTaskCreate(mpu6050_task, "mpu6050_task", 8192, NULL, 1, NULL);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    while (true)
        ;
}
