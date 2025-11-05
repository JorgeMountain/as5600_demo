#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "as5600_lib.h"
#include "init.h"
#include "bldc_pwm.h"
#include "bno055.h"

static const char *TAG = "MAIN";

static bldc_pwm_motor_t motor1;
static bldc_pwm_motor_t motor2;
static bldc_pwm_motor_t motor3;

#define M1_PWM_GPIO   16
#define M1_REV_GPIO   17

#define M2_PWM_GPIO   18
#define M2_REV_GPIO   8

#define M3_PWM_GPIO   20
#define M3_REV_GPIO   21

#define MOTOR_PWM_FREQ_HZ      50        // ESC tipico
#define MOTOR_PWM_RES_HZ       1000000   // resolucion de 1 MHz -> 1 tick = 1 us
#define MOTOR_PWM_BOTTOM_DUTY  55        // ~= 1100us -> minimo del ESC (0..1000)
#define MOTOR_PWM_TOP_DUTY     97        // ~= 1940us -> maximo del ESC (0..1000)
#define MOTOR_PATTERN_MAX_DUTY 20.0f

#define BNO055_I2C_PORT     I2C_NUM_0
#define BNO055_SCL_GPIO     GPIO_NUM_14
#define BNO055_SDA_GPIO     GPIO_NUM_13
#define BNO055_RST_GPIO     GPIO_NUM_NC

static void motor_apply_duty(bldc_pwm_motor_t *motor, float duty, const char *name)
{
    esp_err_t err = bldc_set_duty_motor(motor, duty);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo aplicar duty %.1f%% a %s (err=0x%x)", duty, name, err);
    }
}

static void motors_drive_vector(float vx, float vy, float omega, float duty_limit, const char *label)
{
    const float sin60 = 0.8660254f;
    const float cos60 = 0.5f;

    // Modelo cinemático para robot tipo "kiwi" con tres ruedas omni separadas 120°
    // vx: avance lateral (derecha +), vy: avance frontal (adelante +), omega: giro CCW
    float w1 = vy + omega;
    float w2 = (-cos60 * vy + sin60 * vx) + omega;
    float w3 = (-cos60 * vy - sin60 * vx) + omega;

    float max_mag = fmaxf(fabsf(w1), fmaxf(fabsf(w2), fabsf(w3)));
    float scale = 0.0f;
    if (max_mag > 0.0f) {
        scale = duty_limit / max_mag;
    }

    float duty1 = w1 * scale;
    float duty2 = w2 * scale;
    float duty3 = w3 * scale;

    motor_apply_duty(&motor1, duty1, "motor1");
    motor_apply_duty(&motor2, duty2, "motor2");
    motor_apply_duty(&motor3, duty3, "motor3");

    ESP_LOGI(TAG, "Patrón motores: %s -> duty [%.1f, %.1f, %.1f]",
             label, duty1, duty2, duty3);
}

typedef struct {
    float vx;
    float vy;
    float omega;
    const char *label;
} motor_pattern_cmd_t;

static void motor_demo_step(void)
{
    static const motor_pattern_cmd_t pattern[] = {
        {  0.577350f,  1.000000f,  0.000000f, "Avance (M1 opuesto a M3)" },
        { -0.577350f, -1.000000f,  0.000000f, "Retroceso (M1 opuesto a M3)" },
        { 0.0f,        0.0f,        0.6f,      "Giro eje" },
    };
    static size_t idx = 0;

    const motor_pattern_cmd_t *cmd = &pattern[idx];
    motors_drive_vector(cmd->vx, cmd->vy, cmd->omega, MOTOR_PATTERN_MAX_DUTY, cmd->label);

    idx = (idx + 1) % (sizeof(pattern) / sizeof(pattern[0]));
}

void app_main(void)
{
    ESP_LOGI(TAG, "==== AS5600 DEMO START ====");

    AS5600_t encoder;
    BNO055_t imu;

    // --- Inicializar bus I2C ---
    if (!as5600_init_bus(&encoder, AS5600_I2C_PORT, AS5600_I2C_SCL_PIN, AS5600_I2C_SDA_PIN, AS5600_I2C_FREQ_HZ)) {
        ESP_LOGE(TAG, "Fallo al iniciar bus I2C");
        return;
    }
    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");

    // --- Inicializar ADC para encoders analógicos ---
    bool analog_available = as5600_init_analog_encoders();
    if (!analog_available) {
        ESP_LOGW(TAG, "No se pudo inicializar los encoders analógicos (ADC)");
    } else {
        ESP_LOGI(TAG, "Encoders analógicos inicializados");
    }

    // --- Inicializar IMU BNO055 en bus independiente ---
    bool imu_available = (BNO055_Init(&imu, BNO055_I2C_PORT, BNO055_SCL_GPIO, BNO055_SDA_GPIO, BNO055_RST_GPIO) == BNO055_SUCCESS);
    ESP_LOGI(TAG, "IMU status: %s", imu_available ? "OK" : "FALLÓ");

    // --- Calibración volátil completa (0..4095) ---
    if (!as5600_calibrate_full_range(&encoder)) {
        ESP_LOGW(TAG, "No se pudo calibrar el rango completo, revisa el imán o conexiones");
    } else {
        ESP_LOGI(TAG, "Calibración completada (ventana 0..4095)");
    }

    // --- Configurar parámetros opcionales (CONF) ---
    AS5600_config_t conf = { .WORD = 0 };
    conf.PM   = AS5600_POWER_MODE_NOM;
    conf.HYST = AS5600_HYSTERESIS_2LSB;
    conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_FR;
    conf.PWMF = AS5600_PWM_FREQUENCY_115HZ;
    conf.SF   = AS5600_SLOW_FILTER_8X;
    conf.FTH  = AS5600_FF_THRESHOLD_6LSB;
    conf.WD   = AS5600_WATCHDOG_OFF;

    if (AS5600_Calibrate(&encoder, conf, 0x0000, 0x0FFF)) {
        ESP_LOGI(TAG, "Configuración CONF escrita y verificada correctamente");
    } else {
        ESP_LOGW(TAG, "Error al aplicar configuración CONF");
    }

    // --- Inicializar PWM de los tres ESC ---
    ESP_LOGI(TAG, "Inicializando motores...");

    if (bldc_init(&motor1, M1_PWM_GPIO, M1_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 1");
        return;
    }

    if (bldc_init(&motor2, M2_PWM_GPIO, M2_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 2");
        return;
    }

    if (bldc_init(&motor3, M3_PWM_GPIO, M3_REV_GPIO, MOTOR_PWM_FREQ_HZ,
                  0, MOTOR_PWM_RES_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init motor 3");
        return;
    }

    if (bldc_enable(&motor1) != ESP_OK ||
        bldc_enable(&motor2) != ESP_OK ||
        bldc_enable(&motor3) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo habilitar MCPWM para todos los motores");
        return;
    }

    bldc_set_duty(&motor1, MOTOR_PWM_BOTTOM_DUTY);
    bldc_set_duty(&motor2, MOTOR_PWM_BOTTOM_DUTY);
    bldc_set_duty(&motor3, MOTOR_PWM_BOTTOM_DUTY);
    vTaskDelay(pdMS_TO_TICKS(3000));  // esperar a que el ESC arme

    // --- Loop principal ---
    float analog_deg[AS5600_ANALOG_COUNT] = {0};

    while (1)
    {
        uint8_t status = 0, agc = 0;
        uint16_t mag = 0, raw = 0;
        float deg = 0;
        float yaw = 0, pitch = 0, roll = 0;

        bool ok = true;

        ok &= as5600_get_status(&encoder, &status);
        ok &= as5600_get_agc(&encoder, &agc);
        ok &= as5600_get_magnitude(&encoder, &mag);
        ok &= as5600_get_raw_angle(&encoder, &raw);
        ok &= as5600_get_angle_deg(&encoder, &deg);

        if (!ok) {
            ESP_LOGE(TAG, "Error leyendo el AS5600 por I2C");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        for (size_t i = 0; analog_available && i < AS5600_ANALOG_COUNT; ++i) {
            float adc_deg = analog_deg[i];
            if (as5600_read_analog_deg(i, &adc_deg)) {
                analog_deg[i] = adc_deg;
            } else {
            ESP_LOGW(TAG, "No se pudo leer encoder analógico %zu", i);
            }
        }

        if (imu_available) {
            BNO055_GetEulerAngles(&imu, &yaw, &pitch, &roll);
        }

        printf("STATUS=0x%02X  AGC=%3u  MAG=%4u  RAW=%4u  ANG=%.2f deg",
               status, agc, mag, raw, deg);
        for (size_t i = 0; analog_available && i < AS5600_ANALOG_COUNT; ++i) {
            printf("  ANA%zu=%.2f deg", i, analog_deg[i]);
        }
        if (imu_available) {
            printf("  IMU[YPR]=%.2f/%.2f/%.2f deg", yaw, pitch, roll);
        }
        printf("\n");

        static int loop_count = 0;
        if (++loop_count >= 10) {  // cada ~5 s avanza un paso de la demo
            motor_demo_step();
            loop_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // 2 Hz
    }

    // Nunca llega aquí, pero si quisieras limpiar:
    // as5600_deinit_bus(&encoder);
}
