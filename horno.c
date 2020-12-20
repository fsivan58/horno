#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

//#pragma intrinsyc(_delay)
#define _XTAL_FREQ 20000000

#define abs(x) (x < 0 ? -x : x)

const int registrosTMR1 = 65535 - 34285; // 0.05s
const int contadorTMR1 = 5;
const int limiteMonitor = 4;
const int limiteSensores = 20;
const int maxAnalogRead = 1023;
int lastDial = 0;
int newDial = 0;
int cont;
int contMonitor;
int contSensores;
int contUpdate;
int consignaTemperatura;

// Dial
int valoresDial [10];
int indiceDial = 0;
int tamLecturas = 0;

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
    PR2 = 166;
}

void initPortB()
{
    // 9.1.1
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    ANSELHbits.ANS8 = 1;
    ANSELHbits.ANS9 = 1;
    ANSELHbits.ANS10 = 1;
    ANSELHbits.ANS11 = 1;
    ANSELHbits.ANS12 = 1;
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
     *  potenciómetro consigna       5 - AN11 / RB4
     */
    ADCON0bits.ADCS1 = 1;
    ADCON0bits.ADCS0 = 0;
    ADCON0bits.ADON = 1;
    ADCON1bits.ADFM = 1;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADRESL = 0;
    PIE1bits.ADIE = 1;  // Habilitacion de la interrupcion
    PIR1bits.ADIF = 0;  // No se ha iniciado o completado la conversion
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

int calculoTempMedia ()
{
	int i, suma = 0;
    float media;

	for (i = 0; i < tamLecturas; i++) {
        suma = suma + valoresDial[i];
    }
    media = suma / tamLecturas;
	tamLecturas = 0;
	indiceDial = 0;

	return (int) media;
}

float normalizarIluminancia (int iluminancia)
{
    return ((iluminancia/maxAnalogRead)*5) / 3.8E-4;
}

int lecturaADC ()
{
    // esperar tiempo S&H - 3,8e-7
    // Tcy = 2,7e-7 => vale con dos nop
    NOP
    NOP
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
        printf("Valor Dial: %d\r\n", calculoTempMedia());
        printf("Temperatura exterior: %d\r\n",tempExt);
        printf("Temperatura interior: %d\r\n",tempInt);
        printf("Luminosidad: %d\r\n",iluminancia);
        printf("Humedad ambiente: %d\r\n",humedad);
    } else {
        printf("Apagando el Horno \r\n");
        apagarSistema();
    }    
}

void lecturasignal ()
{
    lastDial = newDial;
    ADCON0bits.CHS = 0b1011;
    newDial = lecturaADC();
    float Vnormalizado = normalizarDial();
    if (abs(newDial - lastDial) < 0.5)
    {
        contUpdate++;
        if(contUpdate == limiteSensores)
        { // 5 segundos
			calculoTempMedia();
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
            valoresDial[indiceDial] = temperatura;
			if (indiceDial < 9)
			{
				indiceDial++;
				tamLecturas++;
			}
			else
            {
				indiceDial = 0;
            }
        }
    }
}

void __interrupt() intHandler()
{
    if (PIR1bits.TMR1IF) {
        TMR1 = registrosTMR1;
        if(cont == contadorTMR1){ // 250 ms
            cont = 0;
            contMonitor++;
            contSensores++;
            lecturasignal();
        }
        if(contMonitor == limiteMonitor){ // 1 segundo          
            contMonitor = 0;
            updateMonitor(0);
        }
        if(contSensores == limiteSensores){ // 5 segundos
            // actualizar el valor de la temperatura introducida en el monitor
            contSensores = 0;
        }
        cont++;
        PIR1bits.TMR1IF= 0;
    }
}

float calculoError ()
{
    return (consignaTemperatura - tempInt) / consignaTemperatura;
}

void calefactor ()
{
    // Preguntar para alimentar con 5V
    PORTCbits.RC4 = 0;
    PORTCbits.RC3 = 4095;
}

void refrigerador ()
{
    // Preguntar para alimentar con 5V
    PORTCbits.RC3 = 0;
    PORTCbits.RC4 = 4095;
}

void apagarPeltier ()
{
    PORTCbits.RC4 = 0;
    PORTCbits.RC3 = 0;
}

void controlTemperatura ()
{
    float error = calculoError();
    if (error > 0.05) {
        calefactor();
    } else if (error < 0.05) {
        refrigerador();
    } else {
        apagarPeltier();
    }
}

float ventilador (int diff)
{
    int max = 30;
    int maxCicloTrabajo = 75 - 25;
    int minCicloTrabajo = 25;
    int pwm;
    if (diff > 30)
        pwm = maxCicloTrabajo;
    else
        pwm = ((diff/max) * maxCicloTrabajo) + minCicloTrabajo;
    return pwm;
}

void putch(char c)
{
    while(!TXSTAbits.TRMT);
    TXREG = c;
}

void main(void)
{
    initGeneral();
    while (1)
    {
        controlTemperatura();
    }
}