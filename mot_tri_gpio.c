Programa en C para el Control de un Motor Trifásico con Microcontrolador XC164 y Memoria EEPROM 24FC128

A continuación, se presenta un programa en C diseñado para controlar un motor trifásico utilizando el microcontrolador XC164 de Infineon y una memoria EEPROM 24FC128 para el almacenamiento de datos. El programa se centra en la generación de señales PWM (Modulación por Ancho de Pulso) trifásicas y la comunicación I2C para interactuar con la memoria externa.

Descripción General del Funcionamiento:

El control del motor trifásico se logra mediante la generación de tres señales PWM con un desfase de 120 grados entre sí. Estas señales son generadas por la unidad CAPCOM6 del microcontrolador XC164, un periférico diseñado específicamente para aplicaciones de control de motor.[1][2][3] La velocidad y el sentido de giro del motor se pueden ajustar modificando la frecuencia y el ciclo de trabajo de estas señales PWM.

Para almacenar parámetros de configuración del motor, como la velocidad deseada, se utiliza la memoria EEPROM 24FC128. La comunicación con esta memoria se realiza a través del protocolo I2C, que se implementa por software ("bit-banging") en los pines de propósito general (GPIO) del microcontrolador, ya que el XC164 no cuenta con un periférico I2C por hardware dedicado.[4][5][6]

A continuación, se detallan los componentes clave del programa:

1. Configuración del Hardware y Definiciones

Es fundamental definir los pines del microcontrolador que se utilizarán para la generación de las señales PWM y la comunicación I2C.

Generated c
// Definiciones de pines para I2C (Bit-banging)
#define I2C_SCL_PIN P1L_1 // Pin para la línea de reloj I2C
#define I2C_SDA_PIN P1L_0 // Pin para la línea de datos I2C

// Definiciones de pines para las salidas PWM (CAPCOM6)
// Estos pines corresponden a las salidas del módulo CCU6
// que es parte de CAPCOM6 en algunas variantes del XC164.
// La asignación exacta de pines puede variar según el encapsulado
// y la configuración específica del microcontrolador.
// Consultar la hoja de datos del dispositivo específico.
// Ejemplo de asignación:
// Fase U: COUT60
// Fase V: COUT61
// Fase W: COUT62

2. Generación de PWM Trifásico con CAPCOM6

La unidad CAPCOM6 del XC164 es idónea para generar las señales PWM necesarias para controlar un motor trifásico.[7][8] El siguiente código muestra una posible inicialización y configuración para generar tres señales PWM con desfase.

Generated c
#include <xc164cs.h>

// Periodo del Timer12 para la frecuencia PWM
#define T12_PERIOD 2000 // Para una frecuencia de 20kHz con un reloj de 40MHz

void init_3phase_pwm(void) {
    // --- Configuración del Módulo CCU6 ---
    // Habilitar el reloj para el CCU6
    CCU6_CLC = 0x0000;

    // --- Configuración del Temporizador T12 ---
    // Configurar T12 para operar en modo de conteo ascendente/descendente
    // para generar PWM centrado en el borde.
    T12MSEL = 0x0001;
    T12CTL_T12M = 2; // Modo de conteo ascendente/descendente

    // Establecer el período del temporizador T12
    T12PR = T12_PERIOD / 2;

    // --- Configuración de los Canales de Comparación ---
    // Configurar los canales CC60, CC61 y CC62 en modo de comparación
    CC6_MCMOF = 0x0000;

    // Habilitar las salidas de los comparadores
    CC6_PSL = 0x0155; // Nivel de salida activo alto

    // Establecer valores de comparación iniciales (ciclo de trabajo)
    CC60_CC6R = T12_PERIOD / 4; // Fase U - Ciclo de trabajo del 50%
    CC61_CC6R = T12_PERIOD / 4; // Fase V - Ciclo de trabajo del 50%
    CC62_CC6R = T12_PERIOD / 4; // Fase W - Ciclo de trabajo del 50%

    // Iniciar el temporizador T12
    T12CTL_T12R = 1;

    // Transferir valores de sombra a los registros principales
    CC6_TCTR4 = 0x0004;
}

void set_duty_cycle(unsigned int duty_u, unsigned int duty_v, unsigned int duty_w) {
    // Actualizar los valores de comparación para cambiar el ciclo de trabajo
    CC60_CC6SR = duty_u;
    CC61_CC6SR = duty_v;
    CC62_CC6SR = duty_w;
    // Transferir valores de sombra a los registros principales
    CC6_TCTR4 = 0x0004;
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END
3. Implementación de I2C por Software (Bit-Banging)

Dado que el XC164 carece de un periférico I2C dedicado, la comunicación con la EEPROM 24FC128 se realiza manipulando directamente los pines de E/S.[4][9]

Generated c
// --- Funciones de bajo nivel para I2C ---
void i2c_start(void) {
    I2C_SDA_PIN_DIR = 0; // SDA como salida
    I2C_SDA_PIN = 1;
    I2C_SCL_PIN = 1;
    I2C_SDA_PIN = 0;
    I2C_SCL_PIN = 0;
}

void i2c_stop(void) {
    I2C_SDA_PIN_DIR = 0; // SDA como salida
    I2C_SDA_PIN = 0;
    I2C_SCL_PIN = 1;
    I2C_SDA_PIN = 1;
}

void i2c_write_byte(unsigned char data) {
    unsigned char i;
    I2C_SDA_PIN_DIR = 0; // SDA como salida
    for (i = 0; i < 8; i++) {
        I2C_SDA_PIN = (data & 0x80) ? 1 : 0;
        data <<= 1;
        I2C_SCL_PIN = 1;
        I2C_SCL_PIN = 0;
    }
    // Esperar ACK
    I2C_SDA_PIN_DIR = 1; // SDA como entrada
    I2C_SCL_PIN = 1;
    // Aquí se debería verificar el ACK
    I2C_SCL_PIN = 0;
}

unsigned char i2c_read_byte(void) {
    unsigned char i, data = 0;
    I2C_SDA_PIN_DIR = 1; // SDA como entrada
    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_PIN = 1;
        if (I2C_SDA_PIN) {
            data |= 1;
        }
        I2C_SCL_PIN = 0;
    }
    return data;
}

// --- Funciones de alto nivel para la EEPROM 24FC128 ---
void eeprom_write(unsigned int address, unsigned char data) {
    i2c_start();
    i2c_write_byte(0xA0); // Dirección del dispositivo EEPROM
    i2c_write_byte((unsigned char)(address >> 8)); // Dirección alta
    i2c_write_byte((unsigned char)address);       // Dirección baja
    i2c_write_byte(data);
    i2c_stop();
}

unsigned char eeprom_read(unsigned int address) {
    unsigned char data;
    i2c_start();
    i2c_write_byte(0xA0); // Dirección del dispositivo para escritura
    i2c_write_byte((unsigned char)(address >> 8));
    i2c_write_byte((unsigned char)address);
    i2c_start(); // Inicio repetido
    i2c_write_byte(0xA1); // Dirección del dispositivo para lectura
    data = i2c_read_byte();
    i2c_stop();
    return data;
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END
4. Lógica de Control del Motor y Programa Principal

El programa principal inicializa los periféricos y entra en un bucle donde se podría leer la velocidad deseada desde la EEPROM y ajustar las señales PWM en consecuencia. Se implementa una tabla de seno para generar las formas de onda sinusoidales para un control más suave del motor (SPWM).

Generated c
// Tabla de seno para la generación de SPWM (Ejemplo con 32 pasos)
const unsigned int sine_wave[32] = {
    1000, 1195, 1383, 1558, 1713, 1841, 1938, 1995,
    2000, 1995, 1938, 1841, 1713, 1558, 1383, 1195,
    1000, 805,  617,  442,  287,  159,   62,   5,
    0,    5,    62,   159,  287,  442,  617,  805
};

int main(void) {
    unsigned int step = 0;
    unsigned int desired_speed;
    unsigned int phase_u, phase_v, phase_w;

    // Inicialización del sistema
    init_3phase_pwm();

    // Leer la velocidad deseada desde la EEPROM (ejemplo)
    desired_speed = eeprom_read(0x00);

    while(1) {
        // Calcular los valores de ciclo de trabajo para cada fase
        // usando la tabla de seno.
        phase_u = (sine_wave[step] * desired_speed) / 255;
        phase_v = (sine_wave[(step + 10) % 32] * desired_speed) / 255; // Desfase de ~120 grados
        phase_w = (sine_wave[(step + 21) % 32] * desired_speed) / 255; // Desfase de ~240 grados

        // Establecer el nuevo ciclo de trabajo
        set_duty_cycle(phase_u, phase_v, phase_w);

        // Incrementar el paso para la siguiente iteración de la onda sinusoidal
        step++;
        if (step >= 32) {
            step = 0;
        }

        // Se puede agregar un retardo para controlar la frecuencia de la onda (velocidad del motor)
    }

    return 0;
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END

Consideraciones Adicionales:

Hardware de Potencia: Este código se enfoca en la lógica de control del microcontrolador. Para mover un motor trifásico real, se necesita una etapa de potencia (inversor trifásico) con drivers de compuerta para los transistores (MOSFETs o IGBTs).

Control de Lazo Cerrado: Para un control de velocidad preciso, se requeriría un sistema de lazo cerrado que utilice sensores (como sensores de efecto Hall o un codificador) para medir la velocidad real del motor y ajustar las señales PWM en consecuencia. Algoritmos de control más avanzados como el Control Vectorial (FOC) o el control V/Hz (Voltaje/Frecuencia) son comunes para este propósito.[10][11]

Compilador y Entorno de Desarrollo: Este código es un ejemplo conceptual y puede requerir adaptaciones para un compilador específico como Keil C166 o Tasking. Los nombres de los registros y las bibliotecas pueden variar.

Seguridad: Es crucial implementar medidas de seguridad, como la protección contra sobrecorriente y sobretemperatura, en el diseño del hardware. El control de tiempo muerto (dead-time) en la generación de PWM es fundamental para evitar cortocircuitos en el inversor. La unidad CAPCOM6 del XC164 a menudo incluye hardware para la inserción de tiempo muerto.

Sources
help
infineon.com
infineon.com
keil.com
stackexchange.com
nxp.com
ti.com
gerbo.nl
infineon.com
youtube.com
github.io
microchip.com
Google Search Suggestions
Display of Search Suggestions is required when using Grounding with Google Search. Learn more
Infineon XC164 CAPCOM6 three-phase PWM C code example
XC164 bit-bang I2C C code for EEPROM
XC164 microcontroller motor control application note
V/Hz control algorithm C implementation for microcontroller
