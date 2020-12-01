#include <xc.h>
#include <stdio.h>

// See /opt/microchip/xc8/v<version>/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

//#pragma intrinsyc(_delay)
#define _XTAL_FREQ 20000000

const unsigned char registrosTMR1 = 65535 - 34285; // 0.05s
const unsigned char contadorTMR1 = 5;
const unsigned limiteMonitor = 4;
const unsigned limiteSensores = 20;
unsigned char x;
unsigned cont;
unsigned contMonitor;
unsigned contSensores;
unsigned short adc_valor;
unsigned char adc_lectura=0;

void init_general()
{  OSCCON = 0b00001000;
   init_uart();
   init_adc();
   init_TMR1();
   INTCONbits.GIE=1;
   cont = 0;
   contMonitor = 0;
}

void init_uart(void)
{
    TXSTAbits.BRGH =0;
    BAUDCTLbits.BRG16=0;
    SPBRGH = 0b00000000;
    SPBRG = 0b00100000;
    TXSTAbits.SYNC=0;
    TXSTAbits.TX9=0;
    RCSTAbits.RX9=0;
    PIE1bits.TXIE=0;
    PIE1bits.RCIE=0;
    RCSTAbits.SPEN=1;
    TXSTAbits.TXEN=0;
    TXSTAbits.TXEN=1;
}
void init_TMR1()
{   PIE1bits.TMR1IE= 1;
    T1CONbits.T1GINV = 1;
    T1CONbits.TMR1GE = 0;
    T1CONbits.T1CKPS = 0b11;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.T1SYNC = 0;
    T1CONbits.TMR1CS = 0;
    T1CONbits.TMR1ON = 1;
    TMR1 = registrosTMR1; //Interrupcion cada 0.05 segundos
}
void init_TMR2()
{   PR2 = 0xA6;
    
}
void init_adc(void)
{   ADCON0bits.ADCS1=1;
    ADCON0bits.ADCS0=0;
    ADCON0bits.ADON=1;
    ADCON0bits.CHS=0;
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=0;
    ADCON0bits.CHS3=0;

    ADCON1bits.ADFM=1;
    ADCON1bits.VCFG0=0;
    ADCON1bits.VCFG1=0;

    ADRESL=0B00000000;

    PIE1bits.ADIE=1;
    INTCONbits.PEIE=1;
}
void __interrupt() int_handler()
{   if (PIR1bits.TMR1IF){
        TMR1 = registrosTMR1;
        if(cont == contadorTMR1){ // 250 ms
            cont = 0;
            contMonitor++;
            contSensores++;
            // ADCON0bits.GO=1;
        }
        if(contMonitor == limiteMonitor){ // 1 segundo
            // actualizar tmp monitor
            contMonitor = 0;
        }
        if(contSensores == limiteSensores){ // 5 segundos
            // actualizar el valor de la consigna en el monitor
            contSensores = 0;
        }
        cont++;
        PIR1bits.TMR1IF= 0;
    }
    // if(PIR1bits.ADIF){
    //    PIR1bits.ADIF=0;
    //    adc_valor=ADRESH << 8;
    //    adc_valor|=ADRESL;
    //    PORTB = ADRESL;
    //    adc_lectura=1;
    //}
}
void putch(char c)
{
    while(!TXSTAbits.TRMT);
    TXREG=c;
}
void main(void)
{  

   while(1)
   {
       while(adc_lectura==0);
       adc_lectura=0;
       printf("%d\r\n",adc_valor);
   }

}


