# AS5600 & BNO055 ESP32-S3 Demo

Este proyecto para ESP32-S3 muestra cómo:

- inicializar un encoder magnético **AS5600** vía I²C (GPIO4/GPIO5) y leer sus registros principales;
- leer dos AS5600 adicionales por la salida analógica utilizando el ADC interno;
- controlar tres motores BLDC con MCPWM aplicando un patrón de avance, retroceso y giro;
- y, de forma opcional, inicializar una IMU **BNO055** en un bus I²C secundario (GPIO13/GPIO14) y recuperar sus ángulos de Euler.

Todo está escrito con ESP-IDF v5.x.

---

## Funcionamiento del código

1. **`main/main.c`**
   - Configura el bus I²C principal (GPIO5–SCL / GPIO4–SDA) y el ADC para las salidas analógicas.
   - Ejecuta una calibración volátil completa del AS5600 principal y carga un registro `CONF` con hysteresis y modo analógico.
   - Inicializa el BNO055 (dirección 0x28) en un segundo bus I²C usando GPIO14 (SCL) y GPIO13 (SDA); al arrancar se verifica el `CHIP_ID` y se deja el sensor en modo NDOF.
   - Prepara tres motores (`motor1`, `motor2`, `motor3`) con MCPWM. Cada ciclo de 500 ms aplica un paso del patrón: avance, retroceso o giro.
   - Mide continuamente: estado del AS5600 (status, AGC, magnitud, ángulo), dos lecturas analógicas, y si la IMU está disponible, los ángulos yaw/pitch/roll. Todo se imprime en consola.

2. **`main/init.[ch]`**
   - Expone `as5600_init_bus`, `as5600_init_analog_encoders`, `as5600_read_analog_deg` y un arreglo con la configuración de los canales ADC (GPIO2→ADC1_CH1 y GPIO3→ADC1_CH2).
   - Guarda el handle global del AS5600 principal para usos futuros.

3. **`main/as5600_lib.[ch]`**
   - Implementa la comunicación I²C con el AS5600 (lecturas u8/u16, escritura de ventanas ZPOS/MPOS, lecturas de ángulo y magnitud).
   - Incluye utilidades para calibración volátil, lectura en grados y escritura del registro `CONF`.

4. **`main/bno055.[ch]`**
   - Implementa helpers I²C para el BNO055 (escritura/lectura de registros y conversión de aceleración, giro, Euler y magnetómetro).
   - `BNO055_Init` asegura la configuración del bus, verifica el chip ID, selecciona página 0, define unidades, normaliza modo de potencia y entra en NDOF.
   - Provee funciones para obtener Euler, aceleraciones, gyro, magnetómetro y perfiles de calibración.

5. **`main/bldc_pwm.[ch]`**
   - Configura MCPWM para cada motor, crea comparadores, genera PWM para la señal principal y la reversa.
   - Expone `bldc_set_duty_motor`, que recibe un porcentaje (±100) y lo convierte al duty permitido por el ESC (entre `MOTOR_PWM_BOTTOM_DUTY` y `MOTOR_PWM_TOP_DUTY`).

---

## Pines utilizados

| Componente            | Función                | GPIO |
|-----------------------|------------------------|------|
| AS5600 (I²C)          | SCL                    | 5    |
|                       | SDA                    | 4    |
| AS5600 analógico #0   | OUT (ADC1_CH1)         | 2    |
| AS5600 analógico #1   | OUT (ADC1_CH2)         | 3    |
| Motor 1               | PWM / REV              | 16 / 17 |
| Motor 2               | PWM / REV              | 18 / 8 |
| Motor 3               | PWM / REV              | 20 / 21 |
| BNO055 (I²C opcional) | SCL / SDA              | 14 / 13 |

---

## Ejecución

```
idf.py set-target esp32s3
idf.py build flash monitor
```

La consola mostrará entradas con el formato:

```
STATUS=0x20  AGC=123  MAG=2048  RAW=1024  ANG=180.00 deg  ANA0=170.50 deg  ANA1=189.75 deg  IMU[YPR]=12.34/1.23/0.45 deg
```

En cada iteración se alterna el patrón de motores (avance → retroceso → giro) y se repite.

---

## Consideraciones destacadas

- El AS5600 no recibe comandos de **burn**; la calibración es volátil y se repite cada vez que arranca el firmware.
- El patrón de motores está diseñado como ejemplo para una plataforma triangular (“kiwi”) y calcula los tres deberes a partir de un vector `(vx, vy, ω)`.
- El driver de la IMU valida el `CHIP_ID` antes de modificar registros; si la lectura falla se avisa en consola y las lecturas IMU no se imprimen.

---

## Árbol de archivos

```
.
├── CMakeLists.txt
├── README.md
└── main
    ├── CMakeLists.txt
    ├── as5600_lib.c / as5600_lib.h
    ├── bldc_pwm.c / bldc_pwm.h
    ├── bno055.c / bno055.h
    ├── init.c / init.h
    └── main.c
```
