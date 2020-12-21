#include <xc.h>
#include <stdio.h>

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

//#pragma intrinsyc(_delay)
#define _XTAL_FREQ 20000000

#define abs(x) (x < 0 ? -x : x)

const unsigned int registrosTMR1 = 65535 - 34285; // 0.05s
const int contadorTMR1 = 5;
const int limiteMonitor = 4;
const int limiteSensores = 20;
const int maxAnalogRead = 1023;
const int registroPR2 = 167;
int lastDial = 0;
int newDial = 0;
int cont;
int contMonitor;
int contSensores;
int contUpdate;
int consignaTemperatura;

// Dial
float sumaDial = 0;
int numLecturas = 0;
int mediaDial;

//Lectura sensores
float iluminancia = 0;
int tempExt = 0;
int tempInt = 0;
float humedad = 0;

void initUSART ()
{
    TXSTAbits.BRGH = 0;
    BAUDCTLbits.BRG16 = 0;
    SPBRGH = 0b00000000;
    SPBRG = 0b00100000;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TX9 = 0;
    RCSTAbits.RX9 = 0;
    PIE1bits.TXIE = 0;
    PIE1bits.RCIE = 0;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 0;
    TXSTAbits.TXEN = 1;
}

void initTMR1 ()
{   
    PIE1bits.TMR1IE= 1;
    T1CONbits.T1GINV = 1;
    T1CONbits.TMR1GE = 0;
    T1CONbits.T1CKPS = 0b11;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.T1SYNC = 0;
    T1CONbits.TMR1CS = 0;
    T1CONbits.TMR1ON = 1;
    TMR1 = registrosTMR1; //Interrupcion cada 0.05 segundos
}

void initTMR2 ()
{
    T2CONbits.TMR2ON = 1;
}

void initPWM ()
{
    CCP2CONbits.CCP2M = 0b1100;
    PR2 = registroPR2;
}

void initPortB()
{
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
}

void initPortC ()
{
    
/*
 * refrigerador - RC4
 * calefactor - RC3
 * PWM - RC1
 */ 
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
}

void initADC ()
{   
    /*
     *  sensor temperatura interior  1 - AN12 / RB0
     *  sensor temperatura exterior  2 - AN10 / RB1
     *  sensor humedad               3 - AN8  / RB2
     *  sensor intensidad luminosa   4 - AN9  / RB3
     *  potenciometro consigna       5 - AN11 / RB4
     */
    ADCON0bits.ADCS1 = 1;
    ADCON0bits.ADCS0 = 0;
    ADCON0bits.ADON = 1;
    ADCON1bits.ADFM = 1;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADRESL = 0;
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0;
    ANSELHbits.ANS8 = 1;
    ANSELHbits.ANS9 = 1;
    ANSELHbits.ANS10 = 1;
    ANSELHbits.ANS11 = 1;
    ANSELHbits.ANS12 = 1;
}

void initGeneral ()
{
    OSCCON = 0b00001000;
    initUSART();
    initTMR1();
    initTMR2();
    initPWM();
    initPortB();
    initPortC();
    initADC();
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
}

float normalizarDial ()
{
    return (newDial/maxAnalogRead) * 3.8;
}

float normalizarTemperatura (float v)
{
    return (v/maxAnalogRead) * 60;
}

void calculoTempMedia ()
{
    if (numLecturas != 0)
    {
        mediaDial = sumaDial / numLecturas;
        numLecturas = 0;
        sumaDial = 0;
    }
}

float normalizarIluminancia (int iluminancia)
{
    return ((iluminancia/maxAnalogRead)*5) / 3.8E-4;
}

int lecturaADC ()
{
    // esperar tiempo S&H - 3,8e-7
    // Tcy = 2,7e-7 => vale con dos nop
    asm("NOP");
    asm("NOP");
    ADCON0bits.GO = 1;
    while (PIR1bits.ADIF != 1);
    int value = ADRESH << 8;
    value |= ADRESL;
    PIR1bits.ADIF = 0;
    return value;
}

void sensorTemperaturaInterior ()
{
    ADCON0bits.CHS = 0b1100;
    int tempIntLectura = lecturaADC();
    tempInt = (tempIntLectura/maxAnalogRead) * 60;
}

void sensorTemperaturaExterior ()
{
    ADCON0bits.CHS = 0b1010;
    int tempExtLectura = lecturaADC();
    tempExt = (tempExtLectura/maxAnalogRead) * 165 - 40;
}

void sensorHumedad ()
{
    ADCON0bits.CHS = 0b1000;
    int humedadLectura = lecturaADC();
    humedad = (humedadLectura/maxAnalogRead) * 100;
}

void sensorIluminancia ()
{
    ADCON0bits.CHS = 0b1001;
    int iluminanciaLectura = lecturaADC();
    iluminancia = normalizarIluminancia(iluminanciaLectura); 
}

void leerSensores ()
{
    sensorHumedad();
    sensorIluminancia();
    sensorTemperaturaExterior();
    sensorTemperaturaInterior();
}

void apagarSistema ()
{
    
}

void updateMonitor (int apagado)
{
    if (apagado == 0) {
        leerSensores();
        printf("Consigna: %d\r\n", mediaDial);
        printf("Temperatura exterior: %d\r\n", tempExt);
        printf("Temperatura interior: %d\r\n", tempInt);
        printf("Luminosidad: %f\r\n", iluminancia);
        printf("Humedad ambiente: %f\r\n", humedad);
    } else {
        printf("Apagando el Horno \r\n");
        apagarSistema();
    }
}

void lecturaSignal ()
{
    lastDial = newDial;
    ADCON0bits.CHS = 0b1011;
    newDial = lecturaADC();
    float Vnormalizado = normalizarDial();
    if (abs(newDial - lastDial) < 0.5)
    {
        contUpdate++;
        if(contUpdate == limiteSensores)
        {
            consignaTemperatura = newDial;
            contUpdate = 0;
        }
    }
    else
    {
        contUpdate = 0;
        if (Vnormalizado > 3)
        {
            updateMonitor(1);
        }
        else
        {
            float temperatura = normalizarTemperatura(Vnormalizado);
            sumaDial += temperatura;
            numLecturas++;
        }
    }
}

void __interrupt() intHandler()
{
    if (PIR1bits.TMR1IF) {
        cont++;
        TMR1 = registrosTMR1;
        if(cont == contadorTMR1){ // 250 ms
            cont = 0;
            contMonitor++;
            contSensores++;
            lecturaSignal();
        }
        if (contMonitor == limiteMonitor){ // 1 segundo          
            contMonitor = 0;
            calculoTempMedia();
            updateMonitor(0);
        }
        if (contSensores == limiteSensores){ // 5 segundos
            updateMonitor(0);
            contSensores = 0;
        }
        PIR1bits.TMR1IF= 0;
    }
}

float calculoError ()
{
    if (consignaTemperatura == 0)
    {
        return (consignaTemperatura - tempInt) / 0.0001;
    }
    else
    {
        return (consignaTemperatura - tempInt) / consignaTemperatura;
    }
}

void calefactor ()
{
    RC4 = 0;
    RC3 = 1;
}

void refrigerador ()
{
    RC3 = 0;
    RC4 = 1;
}

void apagarPeltier ()
{
    RC4 = 0;
    RC3 = 0;
}

void ventilador ()
{
    int max = 30;
    int maxCicloTrabajo = 75;
    int minCicloTrabajo = 25;
    int diff = abs(newDial - lastDial);
    int dutyCycle;
    if (diff > 30)
    {
        dutyCycle = maxCicloTrabajo;
    }
    else
    {
        dutyCycle = (diff/max) * (maxCicloTrabajo - minCicloTrabajo) + minCicloTrabajo;
    }
    CCPR2L = (dutyCycle/100) * registroPR2;
}

void controlTemperatura ()
{
    float error = calculoError();
    if (error > 0.05)
    {
        calefactor();
    } 
    else if (error < 0.05) 
    {
        refrigerador();
    } 
    else 
    {
        apagarPeltier();
        CCPR2L = 0;
    }
    ventilador();
}

void putch(char c)
{
    while(!TXSTAbits.TRMT);
    TXREG = c;
}

void main ()
{
    initGeneral();
    while(1)
    {
        controlTemperatura();
    }
}