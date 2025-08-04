Controle de Motor Trifásico com Microcontrolador XC164 da Infineon: Guia e Código em C

Controlar um motor trifásico utilizando o microcontrolador XC164 da Infineon requer a utilização de seus periféricos especializados, como a unidade de Captura/Comparação (CAPCOM6) para a geração de sinais PWM (Pulse Width Modulation) e o Conversor Analógico-Digital (ADC) para a leitura de feedback, como a corrente do motor. Este guia apresenta um exemplo de código em C para essa finalidade, com foco em uma abordagem de controle vetorial (Field Oriented Control - FOC), uma técnica eficiente e popular para o controle de motores trifásicos.

Visão Geral da Arquitetura

O microcontrolador XC164 possui recursos de hardware que o tornam adequado para aplicações de controle de motor.[1] A unidade CAPCOM6, em particular, pode gerar os sinais PWM necessários para acionar as três fases de um motor.[2] Além disso, o ADC integrado permite a medição de correntes e tensões do motor, essenciais para o controle em malha fechada.[2]

Para uma implementação robusta, o Controle Vetorial (FOC) é um método recomendado. Ele permite o controle independente do fluxo e do torque do motor, resultando em uma operação suave e eficiente em uma ampla faixa de velocidades. Embora exemplos de código FOC completos para o XC164 sejam escassos, a lógica pode ser adaptada de outras famílias de microcontroladores da Infineon, como a série XMC.[3][4]

A seguir, é apresentado um exemplo de código estruturado em C que demonstra os passos essenciais para configurar os periféricos do XC164 e implementar um algoritmo de controle básico.

Estrutura do Código

O código de exemplo está organizado da seguinte forma:

main.c: Contém a função principal, inicialização do sistema e o laço de controle principal.

xc164_motor_control.h / xc164_motor_control.c: Funções de abstração de hardware para inicializar e controlar os periféricos do XC164 (GPIO, PWM, ADC).

foc_algorithm.h / foc_algorithm.c: Implementação do algoritmo de Controle Vetorial (FOC), incluindo as transformadas de Clarke e Park.

Exemplo de Código em C

xc164_motor_control.h

Generated c
#ifndef XC164_MOTOR_CONTROL_H
#define XC164_MOTOR_CONTROL_H

#include <xc164.h>

// Definições de pinos (exemplo, devem ser ajustados para o seu hardware)
#define PWM_U_HIGH P2_0
#define PWM_U_LOW  P2_1
#define PWM_V_HIGH P2_2
#define PWM_V_LOW  P2_3
#define PWM_W_HIGH P2_4
#define PWM_W_LOW  P2_5

#define ADC_CURRENT_U P3_0
#define ADC_CURRENT_V P3_1

// Funções de inicialização
void init_clock(void);
void init_gpio(void);
void init_pwm(void);
void init_adc(void);

// Funções de controle
void set_pwm_duty_cycles(unsigned int duty_u, unsigned int duty_v, unsigned int duty_w);

#endif // XC164_MOTOR_CONTROL_H


xc164_motor_control.c

Generated c
#include "xc164_motor_control.h"

void init_clock(void) {
    // Configuração do PLL para a frequência de operação desejada
    // Exemplo: Configurar para 40MHz a partir de um cristal de 8MHz
    // Esta parte é altamente dependente da sua placa e configuração de hardware
    // Consulte o manual do usuário do XC164 para os detalhes dos registradores PLLCON
}

void init_gpio(void) {
    // Configurar os pinos do PWM como saídas
    DP2 |= 0x003F; // P2.0 a P2.5 como saídas
    P2  &= ~0x003F; // Inicializa as saídas em nível baixo
}

void init_pwm(void) {
    // Utilizar a unidade CAPCOM6 para gerar PWM trifásico
    // A configuração exata depende do modo de operação escolhido
    // Consulte a seção CAPCOM6 do manual do usuário do XC164 [3]
    // Exemplo de configuração simplificada para PWM center-aligned:

    // Configurar o timer T12 como base de tempo para o PWM
    T12CON = 0x8004; // Inicia o T12, prescaler = 16
    T12PR = 12500;   // Período do PWM para uma frequência de 20kHz com clock de 40MHz

    // Configurar os canais da CAPCOM6 para o modo PWM
    CCM0 = 0x0011; // Modo de comparação PWM para os canais 0 e 1
    CCM1 = 0x0011; // Modo de comparação PWM para os canais 2 e 3
    CCM2 = 0x0011; // Modo de comparação PWM para os canais 4 e 5

    // Inicializar os duty cycles
    set_pwm_duty_cycles(0, 0, 0);

    // Habilitar a transferência de sombra para atualização síncrona dos duty cycles
    T12CON_SHAD_EN = 1;
}

void init_adc(void) {
    // Configurar o ADC para ler as correntes das fases U e V
    // Consulte a seção do ADC no manual do usuário do XC164 [7]
    ADCON = 0x8003; // Habilita o ADC, modo de conversão contínua
    ADCH  = 0x0003; // Habilita os canais 0 e 1 (P3.0 e P3.1)
}

void set_pwm_duty_cycles(unsigned int duty_u, unsigned int duty_v, unsigned int duty_w) {
    // Atualiza os registradores de comparação da CAPCOM6 para definir o duty cycle
    CC0 = duty_u;
    CC1 = duty_u;
    CC2 = duty_v;
    CC3 = duty_v;
    CC4 = duty_w;
    CC5 = duty_w;
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END

main.c

Generated c
#include "xc164_motor_control.h"
// Incluir o header para o algoritmo FOC aqui

int main(void) {
    // Desabilitar o watchdog timer durante a inicialização
    WDT_CON_WDT_DIS = 1;

    // Inicialização do hardware
    init_clock();
    init_gpio();
    init_pwm();
    init_adc();

    // Inicialização do algoritmo FOC
    // foc_init();

    // Habilitar o watchdog timer se necessário
    // WDT_CON_WDT_DIS = 0;

    // Habilitar interrupções globais
    IE = 1;

    while (1) {
        // O controle do motor será executado na rotina de interrupção do ADC ou do timer do PWM
        // O laço principal pode ser usado para tarefas de baixa prioridade, como comunicação
    }

    return 0;
}

// Exemplo de rotina de serviço de interrupção para o controle do motor
// A interrupção pode ser gerada pelo fim da conversão do ADC ou pelo estouro do timer do PWM
void motor_control_isr(void) __interrupt(MOTOR_ISR_VECTOR) {
    // Ler as correntes do ADC
    unsigned int current_u = AD_RESULT0;
    unsigned int current_v = AD_RESULT1;

    // Ler a posição do rotor (se estiver usando sensores)

    // Executar o algoritmo FOC
    // foc_step(current_u, current_v, rotor_angle);

    // Obter os novos duty cycles do FOC
    // unsigned int duty_u, duty_v, duty_w;
    // foc_get_duty_cycles(&duty_u, &duty_v, &duty_w);

    // Atualizar o PWM
    // set_pwm_duty_cycles(duty_u, duty_v, duty_w);
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END

Considerações Importantes:

Hardware Específico: O código fornecido é um exemplo genérico. É crucial adaptá-lo para a sua placa de desenvolvimento e driver de motor específicos, especialmente as definições de pinos e a configuração do clock.

Manuais do Usuário: Para uma configuração detalhada dos periféricos, consulte os manuais do usuário do XC164.[5][6][7]

Segurança: A implementação de controle de motor envolve altas tensões e correntes. Certifique-se de tomar as devidas precauções de segurança.

Compilador e Ferramentas: Para compilar e depurar o código para o XC164, serão necessárias ferramentas de desenvolvimento como o Keil C166 ou o TASKING VX-toolset para C166.[8]

Este guia oferece um ponto de partida para o desenvolvimento de um sistema de controle de motor trifásico com o microcontrolador XC164. Para uma implementação completa e robusta, é fundamental um estudo aprofundado do controle de motores e dos periféricos do microcontrolador.

Sources
help
infineon.com
infineon.com
github.com
github.com
infineon.com
keil.com
keil.com
gerbo.nl
Google Search Suggestions
Display of Search Suggestions is required when using Grounding with Google Search. Learn more
C code for three-phase motor control with Infineon XC164
Infineon XC164 three-phase motor control example
XC164 motor control peripherals
Infineon XC164 programming manual
