#include <xc.h>
#include <stdio.h>

// See /opt/microchip/xc8/v<version>/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

//#pragma intrinsyc(_delay)
#define _XTAL_FREQ 20000000

const unsigned  registrosTMR1 = 65535 - 34285; // 0.05s
const unsigned  contadorTMR1 = 5;
const unsigned limiteMonitor = 4;
const unsigned limiteSensores = 20;
unsigned lastDial = 0;
unsigned newDial = 0;
unsigned cont;
unsigned contMonitor;
unsigned contSensores;
unsigned short adc_valor;
unsigned char adc_lectura=0; 

// Dial
unsigned valoresDial [10];
unsigned indiceDial =0;

//Lectura sensores
float iluminancia = 0;
unsigned tempExt = 0;
unsigned tempInt = 0;
float humedad = 0;


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
            lecturasignal();
            
            // ADCON0bits.GO=1;
        }
        if(contMonitor == limiteMonitor){ // 1 segundo          
            contMonitor = 0;
            updateMonitor(0);
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
void lecturasignal (){
    lastDial= newDial;
    newDial= PORTBbits.RB4;
   float Vnormalizado = normalizarDial();
   if(Vnormalizado > 3){
       updateMonitor(1);      
   }  
   else {
    unsigned temperatura = normalizarTemperatura(Vnormalizado);
    valoresDial[indiceDial]=temperatura;
    indiceDial++;
   }
}


float normalizarDial(){
   unsigned max = 4095;
   return (newDial/max)*3.8;
}


unsigned normalizarTemperatura(float v){
    float max = 3;
    return (v/max)*60;
}

void putch(char c)
{
    while(!TXSTAbits.TRMT);
    TXREG=c;
}

void updateMonitor(unsigned apagado){
    if(apagado == 0){
  sensores();      
  printf("Valor Dial: %d\r\n",adc_valor);//Valor Media Temperatura  
  printf("Temperatura exterior: %d\r\n",tempExt);
  printf("Temperatura interior: %d\r\n",tempInt);
  printf("Luminosidad: %d\r\n",iluminancia);
  printf("Humedad ambiente: %d\r\n",humedad);
    }
    else{
        printf("Apagando el Horno \r\n");
        apagarSistema();
    }
    
}
void sensores(){
    sensorHumedad();
    sensorIluminancia();
    sensorTemperaturaExterior();
    sensorTemperaturaInterior();
}

void sensorTemperaturaInterior (){
   
   unsigned tempIntLectura = PORTBbits.RB0;
   unsigned max = 4095;
   tempInt = (tempExtLectura/max)*60;
   
}

void sensorTemperaturaExterior (){
   
   unsigned tempExtLectura = PORTBbits.RB1;
   unsigned max = 4095;
   tempExt = (tempExtLectura/max)*165-40;
   
}


void sensorHumedad (){
   
   unsigned humedadLectura = PORTBbits.RB2;
   unsigned max = 4095;
   humedad = (humedadLectura/max)*100;
   
}

void sensorIluminancia (){
    
   unsigned iluminanciaLectura = PORTBbits.RB3;
   iluminancia = normalizarIluminancia(iluminanciaLectura); 
   
}

float normalizarIluminancia(unsigned ilu){
   unsigned max = 4095;
   return ((ilu/max)*5)/3.8E-4;
}


void apagarSistema(){
    
}

void initPortB(){
    
   TRISBbits.TRISB0 = 1;
   TRISBbits.TRISB1 = 1;
   TRISBbits.TRISB2 = 1;
   TRISBbits.TRISB3 = 1;
   TRISBbits.TRISB4 = 1;
}


void main(void)
{  
   //inicializar initportB
   while(1)
   {
       while(adc_lectura==0);
       adc_lectura=0;
       printf("%d\r\n",adc_valor);
   }

}


