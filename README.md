# AS5600 & BNO055 ESP32-S3 Demo

Proyecto de ejemplo para ESP32-S3 que integra:

- Lectura de un codificador magnético **AS5600** por I²C (GPIO4/GPIO5).
- Lectura de dos AS5600 adicionales usando su salida analógica conectada al ADC.
- Control de **tres motores** BLDC/ESC vía MCPWM con un patrón simple de avance/retroceso/giro.
- Lectura opcional de orientación (**BNO055**) en un bus I²C independiente (GPIO13/GPIO14).

> Probado con ESP-IDF v5.x. Ajusta pines y parámetros según tu hardware.

---

## Requerimientos

- ESP32-S3 con ESP-IDF instalado (`idf.py` accesible en la terminal).
- Tres ESC/motores conectados a los GPIO indicados.
- Sensor AS5600 con bus I²C (SDA=GPIO4, SCL=GPIO5) y dos sensores adicionales por salida analógica.
- IMU BNO055 con ADR a GND (dirección 0x28), SDA=GPIO13, SCL=GPIO14 (pull-ups a 3V3), reset fijo a 3V3.

### Pines por defecto

| Componente | Función                        | GPIO |
|------------|--------------------------------|------|
| AS5600 (I²C) | SCL                           | 5    |
|            | SDA                            | 4    |
| AS5600 (analógico) | OUT #0 -> ADC1_CH1       | 2    |
|            | OUT #1 -> ADC1_CH2             | 3    |
| Motores    | Motor 1 PWM / REV              | 16 / 17 |
|            | Motor 2 PWM / REV              | 18 / 8 |
|            | Motor 3 PWM / REV              | 20 / 21 |
| BNO055     | SCL                            | 14   |
|            | SDA                            | 13   |

> Cambia los defines en `main/init.h` y `main/main.c` si tus conexiones son distintas.

---

## Compilación y carga

1. Abre una terminal con el entorno ESP-IDF configurado (`. $IDF_PATH/export.sh`).
2. Navega al proyecto y selecciona el target si es necesario:
   ```bash
   idf.py set-target esp32s3
   ```
3. Compila y flashea:
   ```bash
   idf.py build flash monitor
   ```
4. Observa por consola los datos de sensores (`STATUS`, `ANG`, `ANA0/ANA1`, `IMU[YPR]`) y el patrón de motores.

---

## Estructura del proyecto

- `main/main.c`: lógica principal, inicialización de sensores, lectura y patrón de motores.
- `main/init.[ch]`: configuración del AS5600, ADC y utilidades.
- `main/as5600_lib.[ch]`: funciones auxiliares para el AS5600.
- `main/bno055.[ch]`: driver I²C para la IMU BNO055.
- `main/bldc_pwm.[ch]`: control MCPWM para los tres ESC.

---

## Personalización rápida

- **Ganancias motores**: ajusta `MOTOR_PATTERN_MAX_DUTY` para más/menos PWM (por defecto 12%). Modifica `pattern[]` en `motor_demo_step` para otros movimientos.
- **Encoders analógicos**: si usas otros pines, edita `AS5600_ANALOG*_GPIO` y `*_CHANNEL` en `main/init.h`.
- **IMU opcional**: si no usas el BNO055, desactiva la inicialización correspondiente en `main/main.c`.
- **Dirección AS5600**: la biblioteca usa 0x36 (fija). Si combinas con otros dispositivos en el mismo bus, añade un multiplexor o un expander.

---

## Notas

- No se realiza **burn** en los AS5600; las configuraciones se vuelven a enviar en cada arranque.
- El patrón de motores supone un robot triangular (“kiwi”). Ajusta signos y magnitudes según tu chasis real.
- Si el BNO055 no responde, revisa tensiones, pull-ups y que el pin ADR esté a GND. El driver intenta leer `CHIP_ID=0xA0` antes de configurar la IMU.

---

¡Disfruta adaptando el demo a tu plataforma! Contribuciones y mejoras son bienvenidas. ***
