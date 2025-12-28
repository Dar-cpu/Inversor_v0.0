/* 
 * File: main  
 * Author: cruzado
 * Comments: v 0.0
 * Revision history: N.A
 * Created on 9 de noviembre de 2025, 23:45
 */

#include <xc.h>
#include "init.h" 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <libpic30.h>
#include <math.h>
#include "lcd.h"
#include <p33FJ64GS606.h>

const char firmware_version[] = "v0.0";   //versión de firmware
const char build_date[]       = __DATE__; // fecha de compilación
const char build_time[]       = __TIME__;  // hora de compilación 

// TABLA SINUSOIDAL        
const uint16_t sine_table[SPWM_TABLE_SIZE] = {
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594,
    2642, 2690, 2737, 2784, 2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143,
    3185, 3227, 3267, 3307, 3346, 3385, 3423, 3459, 3495, 3530, 3565, 3598,
    3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4033, 4045, 4055, 4064, 4072, 4078,
    4084, 4088, 4091, 4092, 4093, 4092, 4091, 4088, 4084, 4078, 4072, 4064,
    4055, 4045, 4033, 4021, 4007, 3992, 3975, 3958, 3939, 3919, 3898, 3876,
    3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3227, 3185, 3143, 3100, 3057,
    3013, 2968, 2923, 2877, 2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496,
    2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098, 2048, 1998, 1948, 1898,
    1848, 1798, 1748, 1698, 1649, 1600, 1551, 1502, 1454, 1406, 1359, 1312,
    1265, 1219, 1173, 1128, 1083, 1039, 996, 953, 911, 869, 829, 789,
    750, 711, 673, 637, 601, 566, 531, 498, 466, 434, 404, 374,
    346, 319, 292, 267, 243, 220, 198, 177, 157, 138, 121, 104,
    89, 75, 63, 51, 41, 32, 24, 18, 12, 8, 5, 4,
    3, 4, 5, 8, 12, 18, 24, 32, 41, 51, 63, 75,
    89, 104, 121, 138, 157, 177, 198, 220, 243, 267, 292, 319,
    346, 374, 404, 434, 466, 498, 531, 566, 601, 637, 673, 711,
    750, 789, 829, 869, 911, 953, 996, 1039, 1083, 1128, 1173, 1219,
    1265, 1312, 1359, 1406, 1454, 1502, 1551, 1600, 1649, 1698, 1748, 1798,
    1848, 1898, 1948, 1998
};

volatile SystemState_t sys_state = {0};
uint16_t adc_buffer[5][ADC_SAMPLES] = {{0}};
uint8_t adc_index = 0;
uint16_t debounce_counter = 0;

ButtonState_t btn_up_state = {1, 1, 1, 0};
ButtonState_t btn_down_state = {1, 1, 1, 0};
ButtonState_t btn_enter_state = {1, 1, 1, 0};

// FUNCIÓN PRINCIPAL
int main(void) {
    System_Init(); 
    __delay_ms(100);
    TRISDbits.TRISD11 = 0; 
    while(1) {
                
        ADC_ReadAll();
        Protection_Check();
       
        if(sys_state.inverter_enabled && !sys_state.soft_start_complete) {
            SoftStart_Process();
        }
   
        else if (!sys_state.inverter_enabled) {
            // Apagado seguro
            IOCON3bits.OVRENH = 1;
            IOCON3bits.OVRENL = 1;
            IOCON3bits.OVRDAT = 0;
            IOCON3bits.PENH = 0;
            IOCON3bits.PENL = 0;
        }
        
        if (sys_state.inverter_enabled && sys_state.soft_start_complete) {

            // Quitar override
            IOCON3bits.OVRENH = 0;
            IOCON3bits.OVRENL = 0;

            // Devolver control al PWM
            IOCON3bits.PENH = 1;
            IOCON3bits.PENL = 1;
        }

        Cooler_Control();
        Button_Process();
        LATDbits.LATD11 = 1; // led indica sistema encendido 
        
        #if !SIMULATE
        if (menu_active) {
            // Si el menú está activo, el menú ya se encarga de display
        } else {
            // Mostrar pantalla principal
            oled_show_main_screen(
                sys_state.battery_voltage,
                sys_state.output_voltage,
                sys_state.battery_current,
                sys_state.temperature
            );
        }
        #endif
        
        __delay_ms(10); 
    }
    return 0; 
}

// INICIALIZACIÓN DEL SISTEMA
void System_Init(void) {
    PLL_Init();
    GPIO_Init();
    ADC_Init();
    PWM_Init();
    Timer_Init();
    I2C_Init();
    Interrupts_Init();
    ZeroCross_Init();
    
    // Estado inicial
    sys_state.inverter_enabled = false;
    sys_state.soft_start_complete = false;
    sys_state.soft_start_counter = 0;
    sys_state.spwm_index = 0;
    sys_state.fault_flags = 0;
    sys_state.battery_current = 0.0f;
    
    SSR_PIN = 0;  // SSR apagado

    oled_init();
    oled_show_boot_sequence();  // animación de inicio
    menu_init();
    menu_active = true;   
    oled_show_main_screen(0.0f, 0.0f, 0.0f, 0.0f); 
}

// CONFIGURACIÓN PLL 
void PLL_Init(void) {
    // PLL para 48.75 MIPS desde 26 MHz
    // M = PLLDIV/2   // PLLDIV = PLL Feedback Divider bits 
    // FOSC=(Fin*M)/N1*N2
    // VCO = Voltage-Controlled Oscillator (Oscilador Controlado por Voltaje)   
    // CLKDIV = Clock divider register
    
    PLLFBD = 28;  // M = 30 (PLLDIV + 2)
    CLKDIVbits.PLLPRE = 0;   // N1 = 2  // Determina cuanto se divide la frecuencia antes de entrar al VCO
    CLKDIVbits.PLLPOST = 01;  // N2 = 4  // Determina cuanto se divide la frecuencia después del VCO
    
    // Iniciar oscilador
    __builtin_write_OSCCONH(0x03);  // Nuevo oscilador = Primary con PLL // solo se toman los bits menos significativos (0b011)
    __builtin_write_OSCCONL(OSCCON | 0x01);  // Iniciar cambio
    
    // Esperar estabilización PLL
    while(OSCCONbits.COSC != 0b011); // Current Oscillator Selection bits 011 = Osc externo con PLL
    while(OSCCONbits.LOCK != 1);  // se bloquea el oscilador, entiendase bloqueo
}

// CONFIGURACIÓN GPIO
void GPIO_Init(void) {
    SSR_TRIS = 0;  // SSR como salida
    SSR_PIN = 0;
    ADPCFG = 0xFFFF; // Todos digitales inicialmente
    TRISB |= 0x003F;  // 0000 0000 0011 1111 RB0?RB5 entradas (Uso de OR para no modificar demás bits)
    
    TRISBbits.TRISB9 = 1;  // RB9 como entrada UP
    TRISBbits.TRISB10 = 1;  //RB10 como entrada DOWN
    TRISBbits.TRISB11 = 1;  //RB11 como entrada ENTER
    
    ADPCFGbits.PCFG1 = 0;  // AN1 analógico (Vbat)
    ADPCFGbits.PCFG2 = 0;  // AN2 analógico (Vout AC)
    ADPCFGbits.PCFG3 = 0;  // AN3 analógico (Cutoff)
    ADPCFGbits.PCFG4 = 0;  // AN4 analógico (NTC puente H)
    ADPCFGbits.PCFG5 = 0;  // AN5 analógico (NTC push-pull)
    
    ADPCFGbits.PCFG9 = 1;   // RB9 digital
    ADPCFGbits.PCFG10 = 1;  // RB10 digital
    ADPCFGbits.PCFG11 = 1;  // RB11 digital

    __delay_ms(10);
}

// CONFIGURACIÓN PWM
void PWM_Init(void) {
    // Configurar PWM para puente H (SPWM), SPWM = 20 kHz
    // Para puente H se usa el modo de "Center Alligned": (Fpwm = Fclk_pwm)/2*(PHASE+1) 
    PTCONbits.PTEN = 0;  // Apagar modulo PWM 
    PTCON2bits.PCLKDIV = 0b000; // Maxima resolución de PWM, entiendase que esta sin "Input Clock Prescaler (Divider)"
    PTCONbits.PTSIDL  = 0; // cONTADOR PWM SIGUE EN idle, BAJO CONSUMO 
    PTCONbits.EIPU    = 0; // no se usa ptper, asi que no se usa este modo
    PTCONbits.SYNCOEN = 0; // sin sincronización de tipo Master - Slave 
    PTCONbits.SYNCEN  = 0;  // Sin sincronizacion externa 
    PTCONbits.SEVTPS  = 0; // Sin sync de Special event que tampoco se usa
    
    //PWM1 puente H 
    PWMCON1bits.CAM = 1; // Modo "Center alligned" 
    PWMCON1bits.ITB = 1; // Base de tiempo independiente
    PWMCON1bits.MDCS = 0; // Se emplean los registros PDCx y SDCx para info del duty de este generador PWM 
    
    PHASE1  = PHASE_BRIDGE;  // 20 kHz
    SPHASE1 = 0;             // No usar fase secundaria

    IOCON1bits.PMOD = 0b00;  // Modo complementario
    IOCON1bits.PENH = 1;     // Habilitar PWM1H
    IOCON1bits.PENL = 1;     // Habilitar PWM1L

    DTR1    = 0;             // En center+complementary, solo usar ALTDTR
    ALTDTR1 = DEAD_TICKS;    // Dead-time  500 ns
    PDC1 = (PHASE1 + 1) / 2;   // Duty inicial 50%
    
    //PWM2 puente H
    PWMCON2bits.CAM  = 1;    // Modo "Center alligned" 
    PWMCON2bits.ITB  = 1;    //Base de tiempo independiente porque usa PHASE2
    PWMCON2bits.MDCS = 0;    // Usa PDC2

    PHASE2  = PHASE_BRIDGE;  //  20 kHz
    SPHASE2 = 0;

    IOCON2bits.PMOD = 0b00;  // Complementario
    IOCON2bits.PENH = 1;     // Habilitar PWM2H
    IOCON2bits.PENL = 1;     // Habilitar PWM2L

    DTR2    = 0;   //  En center+complementary, solo usar ALTDTR
    ALTDTR2 = DEAD_TICKS;   // Dead-time  500 ns
    PDC2 = (PHASE2 + 1) / 2;  
    
    // PWM3 Push Pull 
    STCON2bits.PCLKDIV = 0b000;    // Clock = FOSC = 97.5 MHz
    STPER = STPER_PUSH;             // 50 kHz
    
    PWMCON3bits.CAM  = 0;     // Edge-aligned, alineado por flanco 
    PWMCON3bits.ITB  = 0;     // Usa master secondary timebase
    PWMCON3bits.MTBS = 1;     // Usa (STPER) base de tiempo maestra secundaria sincronización y fuente de reloj para generación PWM.
    PWMCON3bits.MDCS = 0;     // PDCx y SDCx proporcionan información sobre el duty
    // Modo complementario como push-pull
    IOCON3bits.PMOD = 0b00;   // Complementario
    IOCON3bits.PENH = 1;    //  modulo PWM controla el pin PWM3H
    IOCON3bits.PENL = 1;    //  modulo PWM controla el pin PWM3L

    DTR3    = 0;   // Dead-time
    ALTDTR3 = DEAD_TICKS;  //Dead time 500 ns 
    PDC3 = (uint16_t)(0.80 * STPER);     

    //  PWM4 Cooler    
    PWMCON4bits.CAM  = 0;   // Edge aligned
    PWMCON4bits.ITB  = 1;   // Base independiente
    PWMCON4bits.MDCS = 0;   //PDCx y SDCx proporcionan información sobre el duty
    // Solo salida PWM4H
    IOCON4bits.PMOD = 0b10; // "independiente"
    IOCON4bits.PENH = 1;   //  modulo PWM controla el pin PWM3H
    IOCON4bits.PENL = 0;

    PHASE4 = (F_PWM_CLOCK / 10000UL) - 1;    // Frecuencia inicial cooler 10 kHz
    PDC4   = 0;
    
    PTCONbits.PTEN = 1; // Activar módulo PWM
}

// INICIALIZACIÓN ZERO-CROSS
void ZeroCross_Init(void) {
    // Pines ADC como analógicos
   // TRISB |= 0x003F;   // 0000 0000 0011 1111 RB0?RB5 entradas (Uso de OR para no modificar demás bits)
    // ADPCFG = 0xFFC1;    // AN1 - AN5 analógicos, demmás digital (1111 1111 1100 0000)   Por redundancia 
    
    TRISBbits.TRISB8 = 1;  //RB8 como entrada (Zero Cross)
    CNPU1bits.CN8PUE = 0;  // sin pull up interno 
    
    ADPCFGbits.PCFG8 = 1;  // Modo digital y la lectura del puerto esta habilitada, por ende multiplexor de entrada A a D esta a AVSS (GND))

    CNEN2bits.CN22IE = 1; // habilita las notificaciones CN en RB8
    
    // Configurar interrupción con prioridad alta
    IPC4bits.CNIP = 5;   // prioridad medi alta 
    IFS1bits.CNIF = 0; // limpiar bandera
    IEC1bits.CNIE = 1;  // habilita interrupcion de pines de forma global 
}

// INTERRUPCIÓN ZERO-CROSS
void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void) {
    static uint8_t prev_state = 0;
    IFS1bits.CNIF = 0;  // Limpiar flag
    
    if(ZEROCROSS_PIN == 1 && prev_state == 0) {  // detecta pulso positivo
        
        if(sys_state.inverter_enabled) { // reiniciar tabla SPWM en cruce por cero, sincronizar 
            sys_state.spwm_index = 0;          
        }                       
    }                               
    prev_state = ZEROCROSS_PIN;
}   

// CONFIGURACIÓN ADC
void ADC_Init(void) {
    ADPCFG = 0xFFC1; // AN1 a 5 analogicos
    
    //Configuracion de ADC, registro: ADCON 
    ADCONbits.ADON      = 0;      // Apagar ADC mientras se configura
    ADCONbits.FORM      = 0;      // Salida entera (0000 00dd dddd dddd)
    ADCONbits.EIE       = 0;      // IRQ después de 2ª conversión 
    ADCONbits.ORDER     = 0;      // Convierte primero canal PAR y luego IMPAR 
    ADCONbits.SEQSAMP   = 0;      // S&H compartidos muestreados al mismo tiempo 
    ADCONbits.ASYNCSAMP = 0;      // Muestreo síncrono (no asíncrono) 
    ADCONbits.SLOWCLK   = 0;      // ADC clock desde PLL principal (no APLL)
    ADCONbits.ADCS      = 0b011;  // FADC/4 
    
    ADSTAT = 0x0000; // Limpiar flags de listo
    
    // Par 0: AN1 / AN0   -> batería (AN1), AN0 no usado por error en pcb
    // Par 1: AN3 / AN2   -> cutoff / Vout AC
    // Par 2: AN5 / AN4   -> NTC push-pull / NTC puente H
    // Disparados por TRGSRCx = 00010

    ADCPC0bits.TRGSRC0 = 0b00010;   // Trigger por GSWTRG  // Par 0: AN1/AN0
    ADCPC0bits.IRQEN0  = 0;         // No IRQ de par0, IRQ es solicitud de interupción 

    ADCPC0bits.TRGSRC1 = 0b00010;   // Trigger por GSWTRG  // Par 1: AN3/AN2
    ADCPC0bits.IRQEN1  = 0;

    ADCPC1bits.TRGSRC2 = 0b00010;   // Trigger por GSWTRG   // Par 2: AN5/AN4 
    ADCPC1bits.IRQEN2  = 0;

    ADCONbits.ADON = 1;  // Encender ADC
    
    // TAD = 125ns mínimo  TAD = TCY × (ADCS + 1)
    // ADCS = (TAD / TCY) - 1 = (125ns / 25ns) - 1 = 4
}

// LECTURA ADC 
void ADC_ReadAll(void) {
    uint16_t an1, an2, an3, an4, an5; 
    ADCONbits.GSWTRG = 1; // Dispara conversión global de los pares 0,1,2
    
    while ((ADSTATbits.P0RDY == 0) ||   // Par0: AN1/AN0
           (ADSTATbits.P1RDY == 0) ||   // Par1: AN3/AN2
           (ADSTATbits.P2RDY == 0));    // Par2: AN5/AN4   //Que terminen los tres pares
    
    an1 = ADCBUF1;   // AN1 - Vbat// LECTURA ADC 
void ADC_ReadAll(void) {
    uint16_t an1, an2, an3, an4, an5; 
    ADCONbits.GSWTRG = 1; // Dispara conversión global de los pares 0,1,2
    
    while ((ADSTATbits.P0RDY == 0) ||   // Par0: AN1/AN0
           (ADSTATbits.P1RDY == 0) ||   // Par1: AN3/AN2
           (ADSTATbits.P2RDY == 0));    // Par2: AN5/AN4   //Que terminen los tres pares
    
    an1 = ADCBUF1;   // AN1 - Vbat
    an2 = ADCBUF2;   // AN2 - Vout AC
    an3 = ADCBUF3;   // AN3 - Cutoff
    an4 = ADCBUF4;   // AN4 - NTC puente H
    an5 = ADCBUF5;   // AN5 - NTC push-pull  // se leen los buffers dedicados
    
    ADSTATbits.P0RDY = 0;
    ADSTATbits.P1RDY = 0;
    ADSTATbits.P2RDY = 0;  // Limpiar flags 
    
    adc_buffer[0][adc_index] = an1;   // Vbat
    adc_buffer[1][adc_index] = an2;   // Vout AC
    adc_buffer[2][adc_index] = an3;   // Cutoff
    adc_buffer[3][adc_index] = an4;   // NTC puente H
    adc_buffer[4][adc_index] = an5;   // NTC push-pull  // Guardar en buffer circular para promediado
    
    adc_index++;
    if (adc_index >= ADC_SAMPLES)  // Controlador circular para buffer 
        adc_index = 0;

    sys_state.battery_voltage =      // Vbat: 13 V 
        ADC_Average(0) * (VREF / ADC_MAX) * VBAT_SCALE;

    sys_state.output_voltage =  // Vout AC sensada 
        ADC_Average(1) * (VREF / ADC_MAX) * (250.0f / VREF);
    
    sys_state.cutoff_voltage =  // Cutoff 0V a 2.1V 
        ADC_Average(2) * (VREF / ADC_MAX) * (2.1f / VREF);

    // NTCs: dos temperaturas separadas
    float t_bridge   = NTC_Convert_Temp_10K((uint16_t)ADC_Average(3)); // AN4
    float t_pushpull = NTC_Convert_Temperature_100K((uint16_t)ADC_Average(4)); // AN5
    
    sys_state.temp_bridge   = t_bridge;
    sys_state.temp_pushpull = t_pushpull;
    sys_state.temperature   = (t_bridge > t_pushpull) ? t_bridge : t_pushpull;
}
    an2 = ADCBUF2;   // AN2 - Vout AC
    an3 = ADCBUF3;   // AN3 - Cutoff
    an4 = ADCBUF4;   // AN4 - NTC puente H
    an5 = ADCBUF5;   // AN5 - NTC push-pull  // se leen los buffers dedicados
    
    ADSTATbits.P0RDY = 0;
    ADSTATbits.P1RDY = 0;
    ADSTATbits.P2RDY = 0;  // Limpiar flags 
    
    adc_buffer[0][adc_index] = an1;   // Vbat
    adc_buffer[1][adc_index] = an2;   // Vout AC
    adc_buffer[2][adc_index] = an3;   // Cutoff
    adc_buffer[3][adc_index] = an4;   // NTC puente H
    adc_buffer[4][adc_index] = an5;   // NTC push-pull  // Guardar en buffer circular para promediado
    
    adc_index++;
    if (adc_index >= ADC_SAMPLES)  // Controlador circular para buffer 
        adc_index = 0;

    sys_state.battery_voltage =      // Vbat: 13 V 
        ADC_Average(0) * (VREF / ADC_MAX) * VBAT_SCALE;

    sys_state.output_voltage =  // Vout AC sensada 
        ADC_Average(1) * (VREF / ADC_MAX) * (250.0f / VREF);
    
    sys_state.cutoff_voltage =  // Cutoff 0V a 2.1V 
        ADC_Average(2) * (VREF / ADC_MAX) * (2.1f / VREF);

    // NTCs: dos temperaturas separadas
    float t_bridge   = NTC_Convert_Temperature_100K((uint16_t)ADC_Average(3)); // AN4
    float t_pushpull = NTC_Convert_Temp_10K((uint16_t)ADC_Average(4)); // AN5
    
    sys_state.temp_bridge   = t_bridge;
    sys_state.temp_pushpull = t_pushpull;
    sys_state.temperature   = (t_bridge > t_pushpull) ? t_bridge : t_pushpull;
}

// CONFIGURACIÓN INTERRUPCIONES
void Interrupts_Init(void) {
    IPC3bits.ADIP = 5;     // ADC prioridad media-alta
    INTCON1bits.NSTDIS = 0; // Nesting habilitado
}

// CONFIGURACIÓN TIMERS
void Timer_Init(void) {
    // Timer2: Generar interrupción para actualizar SPWM a 60 Hz
    // Frecuencia de actualización = 60 Hz × 256 puntos = 15 360 Hz
    // PR2 = (FCY / (15 360 × Prescaler)) - 1
    // Con FCY = 48.75 MHz y Prescaler = 1:
    // PR2  (48 750 000 / 15 360) - 1 = 3173

    T2CONbits.TON  = 0;    // Apagar Timer2
    T2CONbits.TCS  = 0;    // Clock interno (FCY)
    T2CONbits.TCKPS = 0;   // Prescaler 1:1

    TMR2 = 0;
    PR2  = 3173;           //  60 Hz fundamental con 256 muestras

    IPC1bits.T2IP = 6;     // Prioridad alta
    IFS0bits.T2IF = 0;     // Limpiar flag de interrupción
    IEC0bits.T2IE = 1;     // Habilitar interrupción de T2

    T2CONbits.TON = 1;     // Iniciar Timer2
}

// CONFIGURACIÓN I2C PARA OLED
void I2C_Init(void) {
    // BRG = (FCY / FSCL - FCY / 10,000,000) - 1   I2C1 a 100 kHz
    // BRG = (48,750,000 / 100,000 - 48,750,000 / 10,000,000) - 1 = 481
    I2C1BRG = 481;
    I2C1CONbits.I2CEN = 1;  // Habilitar I2C
}

// INTERRUPCIÓN TIMER2 - ACTUALIZACIÓN SPWM
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;  // Limpiar flag
    
    if(sys_state.inverter_enabled && sys_state.soft_start_complete) {
        SPWM_Generate();
    }
}

// GENERACIÓN SPWM
void SPWM_Generate(void)
{
    uint16_t sine_value;
    float s;                  // valor senoidal normalizado (-1..1)
    float center, amp;
    int16_t dutyA, dutyB;

    sine_value = sine_table[sys_state.spwm_index];   // obtiene muestra de tabla seno 
    s = ((float)sine_value - 2048.0f) / 2047.0f;    // nrmaliza a rango -1..+1
    center = (float)(PHASE_BRIDGE + 1) * 0.5f;   // centro del duty en ticks (50% de PHASE)
    amp = center * MODULATION_INDEX;  // amplitud en ticks 
    dutyA = (int16_t)(center + amp * s);   // PWM1H/L       // SPWM bipolar para puente H
    dutyB = (int16_t)(center - amp * s);   // PWM2H/L

    if (dutyA < 0) dutyA = 0;      // limita a rango válido
    else if (dutyA > PHASE_BRIDGE) dutyA = PHASE_BRIDGE;

    if (dutyB < 0) dutyB = 0;
    else if (dutyB > PHASE_BRIDGE) dutyB = PHASE_BRIDGE;

    PDC1 = (uint16_t)dutyA;     // actualiza PDC1/PDC2 (Puente H A y B)
    PDC2 = (uint16_t)dutyB; 

    sys_state.spwm_index++;   // Incrementar índice de tabla
    if (sys_state.spwm_index >= SPWM_TABLE_SIZE)
        sys_state.spwm_index = 0;
}

// PROMEDIO ADC
float ADC_Average(uint8_t channel) {
    uint32_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < ADC_SAMPLES; i++) {
        sum += adc_buffer[channel][i];
    } 
    return (float)sum / (float)ADC_SAMPLES;
}

// CONVERSIÓN NTC A TEMPERATURA
float NTC_Convert_Temperature_100K(uint16_t adc_value) {
    float voltage, resistance, temp_kelvin;
    voltage = ((float)adc_value / ADC_MAX) * VREF;       // Convertir ADC a voltaje
    resistance = NTC_100K_PULLUP * voltage / (5.0f - voltage);     // Calcular resistencia NTC (divisor de voltaje)
   
      // Ecuación Steinhart-Hart  1/T = 1/T0 + (1/B) × ln(R/R0)
    temp_kelvin = 1.0f / (1.0f/298.15f + (1.0f/NTC_100K_BETA) * log(resistance/NTC_100K_R25));
    return temp_kelvin - 273.15f;  // Convertir a Celsius
}

//CONVERSION NTC A TEMPERATURA DE EL OTRO NTC 
float NTC_Convert_Temp_10K(uint16_t adc_value) {
    float voltage, resistance, temp_kelvin;
    voltage = ((float)adc_value / ADC_MAX) * VREF;
    resistance = NTC_10K_PULLUP * voltage / (5.0f - voltage);
    temp_kelvin = 1.0f / (1.0f/298.15f + (1.0f/NTC_10K_BETA) * logf(resistance/NTC_10K_R25));
    return temp_kelvin - 273.15f;
}

// VERIFICACIÓN DE PROTECCIONES
void Protection_Check(void) {
    sys_state.fault_flags = 0;

    if(sys_state.battery_current > MAX_CURRENT_A) { // Sobrecorriente
        sys_state.fault_flags |= FAULT_OVERCURRENT;
    }
    if(sys_state.battery_voltage < MIN_VOLTAGE_V) {  // Bajo voltaje
        sys_state.fault_flags |= FAULT_UNDERVOLTAGE;
    }
    if(sys_state.battery_voltage > MAX_VOLTAGE_V) {    // Sobre voltaje batería
        sys_state.fault_flags |= FAULT_OVERVOLTAGE; 
    }
    if(sys_state.temperature > MAX_TEMP_C) {    // Sobre temperatura
        sys_state.fault_flags |= FAULT_OVERTEMP;
    }
    if(sys_state.output_voltage > MAX_OUTPUT_VPEAK) {   // Sobre voltaje salida
        sys_state.fault_flags |= FAULT_OUTPUT_OV;
    }
    if(sys_state.fault_flags != 0) {   // Si hay fallas críticas, apagar inmediatamente
        Emergency_Shutdown();
    }
}

// PROCESO SOFT-START
void SoftStart_Process(void) {
    if (!sys_state.inverter_enabled) {
        return;
    }
    uint16_t total_steps = (SOFT_START_TIME_MS / 10);
    if (sys_state.soft_start_counter < total_steps) {    // Proceso de rampa
        SSR_PIN = 1;
        sys_state.soft_start_counter++;
        // Calcular duty cycle para push-pull con rampa 0% a 50%
        // STPER define el periodo del PWM push-pull
        uint16_t max_duty = (STPER_PUSH + 1) / 2;  // 50% de STPER_PUSH
        uint16_t current_duty = (uint16_t)((max_duty * sys_state.soft_start_counter) / total_steps); // Rampa lineal de 0 a max_duty
        PDC3 = current_duty;   // Aplicar duty cycle
        
    } else {
        if (!sys_state.soft_start_complete) {  // Soft-start completado
            sys_state.soft_start_complete = true;
            PDC3 = (STPER_PUSH + 1) / 2;  // Fijar duty final al 50%
            SSR_PIN = 1;
        }
    }
}

// CONTROL DE VENTILADOR
void Cooler_Control(void) {
    uint16_t duty;

    if (sys_state.temperature < 40.0f) {
        duty = 0;  // Apagado
    } 
    else if (sys_state.temperature < 60.0f) {
        duty = (uint16_t)((PHASE4 + 1) * 0.25f);  // 25%
    } 
    else if (sys_state.temperature < 75.0f) {
        duty = (uint16_t)((PHASE4 + 1) * 0.50f);  // 50%
    } 
    else {
        duty = (uint16_t)((PHASE4 + 1) * 0.90f);  // 90%
    }

    PDC4 = duty;  
}

// APAGADO DE EMERGENCIA
void Emergency_Shutdown(void) {
    PDC1 = 0;
    PDC2 = 0;   // Deshabilitar todos los PWMs inmediatamente
    PDC3 = 0;
    PDC4 = 0;
    SSR_PIN = 0;  // Apagar SSR
  
    sys_state.inverter_enabled = false;   // Actualizar estado
    sys_state.soft_start_complete = false;
    sys_state.soft_start_counter = 0;
}

// HABILITAR INVERSOR
void Enable_Inverter(void) {
    
    if(sys_state.fault_flags != 0) {  // Verificar condiciones de seguridad
        return;
    }
    if(sys_state.battery_voltage < MIN_VOLTAGE_V) { // verifica niveles de voltaje 
        return;
    }
    if(sys_state.battery_voltage > MAX_VOLTAGE_V) {
        return;
    }

    sys_state.inverter_enabled = true;
    sys_state.soft_start_complete = false;
    sys_state.soft_start_counter = 0;
    sys_state.spwm_index = 0;
    
    // Inicializa PWM push-pull
    PDC3 = 0;  // Empezar desde 0
    SSR_PIN = 1;
}

// INICIAR PUSH-PULL
void PushPull_Enable(void)
{
    PDC3 = 0; // arrancar desde 0
    sys_state.soft_start_counter  = 0;
    sys_state.soft_start_complete = false;
}

// DESHABILITAR INVERSOR
void Disable_Inverter(void) {
    Emergency_Shutdown();
}

// LEER Y ESTABILIZAR UN BOTÓN
bool Button_Read_Stable(ButtonState_t* btn, uint8_t pin_value) {
    bool edge_detected = false;
    
    btn->current_state = pin_value;
    if (btn->current_state != btn->last_state) { // estado cambia, reiniciar contador
        btn->debounce_counter = DEBOUNCE_TIME;
        btn->last_state = btn->current_state;
    }
    if (btn->debounce_counter > 0) {  // Contar hacia abajo si hay un cambio pendiente
        btn->debounce_counter--;
        if (btn->debounce_counter == 0) {  // contador llega a 0, aceptar el nuevo estado
            if (btn->stable_state != btn->current_state) {
                if (btn->current_state == 0) {  // Detectar presión (pull-up activo bajo)
                    edge_detected = true;
                }
                btn->stable_state = btn->current_state;  
            }
        }
    }
    return edge_detected;
} 

// PROCESAMIENTO DE BOTONES
void Button_Process(void) {
    static uint16_t repeat_counter = 0;
    uint8_t up_pin = BTN_UP;  // Leer estados actuales de los pines
    uint8_t down_pin = BTN_DOWN;
    uint8_t enter_pin = BTN_ENTER;

    bool up_pressed = Button_Read_Stable(&btn_up_state, up_pin);  // Detectar eventos 
    bool down_pressed = Button_Read_Stable(&btn_down_state, down_pin);
    bool enter_pressed = Button_Read_Stable(&btn_enter_state, enter_pin);

    if (menu_active) {  // MODO MENÚ 
        if (up_pressed) {
            if (menu_index > 0) {
                menu_index--;
                menu_show();
            }
            repeat_counter = REPEAT_TIME;
        }
        if (down_pressed) {
            if (menu_index < menu_items_count - 1) {
                menu_index++;
                menu_show();
            }
            repeat_counter = REPEAT_TIME;
        }
        if (enter_pressed) {
            menu_select();
            repeat_counter = REPEAT_TIME;
        }
        if (btn_up_state.stable_state == 0 || btn_down_state.stable_state == 0) { // Autorepetición si se mantiene presionado
            if (repeat_counter > 0) { 
                repeat_counter--;
            } else {   
                if (btn_up_state.stable_state == 0 && menu_index > 0) {    // Repetir acción
                    menu_index--;
                    menu_show();
                    repeat_counter = 5;  // Repetir más rápido
                }
                if (btn_down_state.stable_state == 0 && menu_index < menu_items_count - 1) {
                    menu_index++;
                    menu_show();
                    repeat_counter = 5;
                }
            }
        }
        
    } else {  //  MODO NORMAL
        if (enter_pressed) {   // ENTER abre el menú en modo normal
            menu_active = true;
            menu_index = 0;
            menu_show();
        }
    }
}
