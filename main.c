/*
 * File:   main.c
 * Author: Pablo
 * Ejemplo de uso de I2C Master
 * Created on 17 de febrero de 2020, 10:32 AM
 */
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "oscilador.h"
#include "adc.h"
#include "I2C_librery.h"
#include "LCD.h"
#include "SPI.h"
#include "tmr0.h"
#include <xc.h>
#include <stdio.h>
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 4000000

//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup1(void);
uint8_t z;
uint8_t dato;

uint8_t hours = 12;                             // Variable de 8 bits para las horas.
uint8_t minutes = 59;                           // Variable de 8 bits para los minutos.
uint8_t seconds = 45;                           // Variable de 8 bits para los segundos.
uint8_t POT1 = 0;
uint8_t POT2 = 0;
uint8_t canal_ADC = 0;
uint8_t VALOR_RECIBIDO = 0;
uint8_t VALOR_A_ENVIAR = 0;
uint8_t entrada = 0;
uint8_t entrada2 = 0;
uint8_t bandera = 0;
uint8_t PUSH = 0;
uint8_t PUSH2 = 0;
uint8_t bandera2 = 0;
uint8_t bandera3 = 0;
uint8_t POTENCIOMETRO2 = 25;
uint8_t cambiar = 0;
uint8_t temperatura = 0;
uint8_t adelante = 0;
uint8_t estado = 0;
uint8_t direccion = 0;
uint8_t mover = 0;
uint8_t horas = 1;

char s[];
unsigned short VOLTAJE_0 = 0;
uint8_t init_POT_0 = 0;
uint8_t dec_POT_0 = 0;
uint8_t VAL_POT_0 = 0;

uint8_t BCD_a_Decimal (uint8_t numero);
uint8_t Decimal_a_BCD (uint8_t numero);    // Función que convierte un número decimal a BCD.
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);
//*****************************************************************************
// Main
//*****************************************************************************

void __interrupt() isr(void){
    if(INTCONbits.RBIF){ //Interrupción del puerto B
        if(!PORTBbits.RB1){//cambiamos dirección del motor DC
            direccion = 1;
        }
        else if(!PORTBbits.RB2){//cambiamos dirección del motor DC
            direccion = 0;
        }
        INTCONbits.RBIF = 0;
    }
    if(PIR1bits.ADIF){//Leemos fotorresistencia
        POT1 = adc_read();
    }
    if(PIR1bits.RCIF){//Interrupción de recepción USART
        entrada = RCREG;
        if(entrada == 0){//Verificiamos que nos enviaron para enviar de regreso la variable que pidieron 
            VALOR_A_ENVIAR = seconds;
        }
        else if(entrada == 1){
            VALOR_A_ENVIAR = minutes;
        }
        else if(entrada == 2){
            VALOR_A_ENVIAR = hours;
        }
        else if(entrada == 3){
            VALOR_A_ENVIAR = POT1;
        }
        else if(entrada == 4){
            VALOR_A_ENVIAR = PUSH;
        }
        else if(entrada == 5){
            VALOR_A_ENVIAR = PUSH2;
        }
        else if(entrada == 6){
            VALOR_A_ENVIAR = temperatura;
        }
        TXREG = VALOR_A_ENVIAR;//Enviamos el valor segun la bandera
    }
    if(INTCONbits.T0IF){ //Interrupción TMR0?
        if(mover == 1){
            if(bandera2 == 1){ //Movemos el motor SERVO según la variable POTENCIOMETRO 2
                bandera2 = 0;
                if (bandera2 == 0){
                    PORTDbits.RD0 = 0;
                }
                TMR0 = POTENCIOMETRO2; //cargamos valor
            }
            else if(bandera2 == 0){
                bandera2 = 1;
                if (bandera2 == 1){
                    PORTDbits.RD0 = 1;
                }
                TMR0 = 255-POTENCIOMETRO2; //cargamos valor
            }
        }
        INTCONbits.T0IF = 0; //limpiamos bandera
    }
    if(PIR1bits.TMR1IF)//INTERRUPCION TMR1
        if (mover == 1){
            cambiar++;
            if (cambiar == 10){//Cada cierto tiempo movemos el STEPPER de dirección
                adelante = !adelante;
                cambiar = 0;
            }
            if (bandera3 == 0){ //Movemos SERVO para adelante y para atrás
                POTENCIOMETRO2++;
                if (POTENCIOMETRO2 >= 35){
                    bandera3 = 1;
                }
            }
            else if (bandera3 == 1){
                POTENCIOMETRO2--;
                if (POTENCIOMETRO2 <= 8){
                    bandera3 = 0;
                }
            }
        }
        PIR1bits.TMR1IF = 0;       // Poner a 0 la bandera de bit del TMR1IF
    }
    if(PIR1bits.TMR2IF){// INTERRUPCION TMR2?
        if (mover == 1){
            if (adelante == 1){//Secuencia STEPPER para adelante
                if (estado == 0){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    estado = 1;
                }
                else if (estado == 1){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    estado = 2;
                }
                else if (estado == 2){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    estado = 3;
                }
                else if (estado == 3){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    estado = 4;
                }
                else if (estado == 4){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 1;
                    estado = 5;
                }
                else if (estado == 5){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    estado = 6;
                }
                else if (estado == 6){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    estado = 7;
                }
                else if (estado == 7){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0 ;
                    estado = 0;
                }
            }
            else if (adelante == 0){ //Secuencia stepper hacia atrás
                if (estado == 0){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    estado = 7;
                }
                else if (estado == 1){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    estado = 0;
                }
                else if (estado == 2){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    estado = 1;
                }
                else if (estado == 3){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    estado = 2;
                }
                else if (estado == 4){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 1;
                    estado = 3;
                }
                else if (estado == 5){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    estado = 4;
                }
                else if (estado == 6){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    estado = 5;
                }
                else if (estado == 7){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0 ;
                    estado = 6;
                }
            }
        }
        PIR1bits.TMR2IF = 0;
    }
    return;
}

void main(void) {
    setup1();
    seconds = Decimal_a_BCD(seconds);
    minutes = Decimal_a_BCD(minutes);
    hours = Decimal_a_BCD(hours);

    I2C_Start();                // Llamamos a la función Start.
    I2C_Write(0xD0);            // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
    I2C_Write(0x00);            // Dirección de segundos.
    I2C_Write(seconds);            // Reiniciamos los segundos.
    I2C_Write(minutes);         // Cargamos el valor de minutos en la dirección de minutos.
    I2C_Write(hours);           // Cargamos el valor de horas en la dirección de horas.
    I2C_Stop();                 // Llamamos a la función Stop.
    __delay_ms(200);            // Retardo de 200 ms.
    while(1){
        I2C_Start();                        // Llamamos a la función Start.
        I2C_Write(0xD0);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
        I2C_Write(0);                       // Dirección de segundos.
        I2C_ReStart();                      // Llamamos a la función ReStart.
        I2C_Write(0xD1);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 +1 de lectura.
        seconds=I2C_Read();                 // Cargamos la variable "seconds" con el valor de SSPBUF.
        I2C_Ack();                          // ACK.
        minutes=I2C_Read();                 // Cargamos la variable "minutes" con el valor de SSPBUF.
        I2C_Ack();                          // ACK.
        hours=I2C_Read();                   // Cargamos la variable "hours" con el valor de SSPBUF.
        I2C_NO_Ack();                       // NO ACK.
        I2C_Stop();                         // Llamamos a la función Stop.
        __delay_ms(50);                     // Retardo de 50 ms. 

        adc_start(12);

        I2C_Start();                        // Llamamos a la función Start.
        I2C_Write(0x90);                    // Escribimos en SSPBUF la dirección del sensor de TEMPERATURA LM75 + 0 de escritura.
        I2C_Write(0);                       // Dirección de temperatura.
        I2C_ReStart();                      // Llamamos a la función ReStart.
        I2C_Write(0x91);                    // Escribimos en SSPBUF la dirección del sensor de TEMPERATURA LM75 +1 de lectura.
        temperatura=I2C_Read();                 // Cargamos la variable "temperatura" con el valor de SSPBUF.
        I2C_NO_Ack();                       // NO ACK.
        I2C_Stop();                         // Llamamos a la función Stop.
        __delay_ms(50); 
        
        PUSH = !PORTBbits.RB1;              //El sensor infrarrojo es un pull up, entonces se alterna
        PUSH2 = !PORTBbits.RB2;             //El sensor infrarrojo es un pull up, entonces se alterna
        if (mover == 0){                    //Apagamos el LED que representa el agua
            PORTDbits.RD7 = 0;
        }
        else if (mover == 1){               //Encendemos el LED que representa el agua
            PORTDbits.RD7 = 1;
        }
        
        if (mover == 1){
            PIE1bits.TMR1IE = 1;       // Interrupción habilitada por desbordamiento
            T1CONbits.TMR1ON = 1;            // Encender el temporizador Timer1
            INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
            T2CONbits.TMR2ON = 1;       // Encendemos TMR2
            PIE1bits.TMR2IE = 1;       // Interrupción habilitada por desbordamiento
            if (direccion == 1){
                PORTDbits.RD1 = 1;
                PORTDbits.RD2 = 0;
            }
            else if (direccion == 0){
                PORTDbits.RD1 = 0;
                PORTDbits.RD2 = 1;
            }
        }
        else if (mover == 0){
            PORTDbits.RD0 = 0;
            PIE1bits.TMR1IE = 0;       // Interrupción deshabilitada por desbordamiento
            T1CONbits.TMR1ON = 0;            // Apagamos el temporizador Timer1
            INTCONbits.T0IE = 0;        // Des habilitamos interrupcion TMR0
            T2CONbits.TMR2ON = 0;       // Apagamos TMR2
            PIE1bits.TMR2IE = 0;       // Interrupción des habilitada por desbordamiento
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 0;
        }
        
        if (temperatura >= 15){ //Si la temperatura es menor a 15 no hacemos nadaa
            if (POT1 >= 100){ //Si la luminosidad es menor a 100 no hacemos nada
                horas = BCD_a_Decimal(hours); //Establecemos ciertos horarios de riego
                if (horas == 7){
                    mover = 1;
                }
                else if (horas == 10){
                    mover = 1;
                }
                else if (horas == 13){
                    mover = 1;
                }
                else if (horas == 16){
                    mover = 1;
                }
                else{
                    mover = 0;
                }
            }
            else{
                mover = 0;
            }
        }
        else {
            mover = 0;
        }
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup1(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0b00001111;
    TRISC = 0b10000001;
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    init_osc_MHz(2);
    I2C_Master_Init(100000);        // Inicializar Comuncación I2C
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    
    //TMR0
    INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
    tmr0_init(64);  //configuración prescaler 256 TMR0 
    INTCONbits.T0IF = 0; //limpiamos bandera
    TMR0 = 0;
    
    //TMR1
    PIR1bits.TMR1IF = 0;       // Poner a 0 la bandera de bit del TMR1IF
    TMR1H = 0x22;          // Poner el valor inicial para el temporizador Timer1
    TMR1L = 0x00;
    TMR1CS = 0;            // Temporizador1 cuenta los pulsos del oscilador interno
    T1CKPS1 =1 ; // El valor del pre-escalador asignada es 1:8
    T1CKPS0 = 1;
    PIE1bits.TMR1IE = 1;       // Interrupción habilitada por desbordamiento
    T1CONbits.TMR1ON = 1;            // Encender el temporizador Timer1
    
    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    PR2 = 20;                  // periodo de 2ms
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TOUTPS = 0b1111; //postscaler 16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    PIE1bits.TMR2IE = 1;       // Interrupción habilitada por desbordamiento
    
    //PORTB
    INTCONbits.RBIE = 1;   
    IOCB = 0b00000110;         
    INTCONbits.RBIF = 0;     
    
    //adc
    adc_init(1,0,0);
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t BCD_a_Decimal (uint8_t numero)            // Función que convierte un número BCD a decimal.
{
  return ((numero >> 4) * 10 + (numero & 0x0F));  // Retornamos la variable "numero" desplazado 4 posiciones a la izquierda, multiplicado por 10 más "numero" &&  1111.
}
uint8_t Decimal_a_BCD (uint8_t numero)            // Función que convierte un número decimal a BCD. 
{
    return (((numero / 10) << 4) + (numero % 10));// Retornamos la decena de la variable "numero" desplazado 4 bits a la derecha y las unidades de la variable "numero" en el nibble menos significativo
}