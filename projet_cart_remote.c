/*
* File:   projet_session_01.c
* Author: nbourre
*
* Created on October 17, 2016, 7:51 PM
*/


#include "xc.h"
#include "I2C.h"

#define LOCAL_NUNCHUCK 0

_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & COE_OFF & ICS_PGx1 & FWDTEN_OFF & FWPSA_PR32 & WDTPS_PS1)
_CONFIG2(0x7987)


typedef enum {
    RUNNING_MODE, 
    CONFIG_MODE
} progState;

progState currentState = RUNNING_MODE;
        
struct Nunchuck {
    // Axes x, y, z
    int az; 
    int ay;
    int ax;
    
    // Joystick x, y
    unsigned char jy;
    unsigned char jx;
    
    // Button Z, C
    int bz;
    int bc;
};

// Variables globales;
volatile struct Nunchuck nunchuck;


int blinkEnabled = 0;
int blinkAcc = 0;

int btnCPressTime = 0;

int x_min = 128;
int x_max = 128;
int y_min = 128;
int y_max = 128;

double slopeX = 1;
double slopeY = 1;

long commTicks = 0;
int commInt = 3000;
int rxFlag = 0;

void InitRPs()
{
  // Unlock registers
  asm volatile ( 	"MOV #OSCCON,W1 \n"
  "MOV #0x46,W2	\n"
  "MOV #0x57,W3	\n"
  "MOV.b W2,[W1]	\n"
  "MOV.b W3,[W1]	\n"
  "BCLR OSCCON,#6");
  
  _RP3R = 3;		// Assign TX to RP3 (0x3)
  _U1RXR = 2;
  
  _RP14R = 18;      // Output Compare 1
  
  // Lock registers
  asm volatile ( 	"MOV #OSCCON,W1 \n"
  "MOV #0x46,w2	\n"
  "MOV #0x57,w3	\n"
  "MOV.b W2,[W1]	\n"
  "MOV.b W3,[W1]	\n"
  "BSET OSCCON,#6");
}

void initTimer1() {
  // D�marrage de la minuterie � 1ms
  _T1IF = 0;
  _T1IE = 1;
  _T1IP = 2;
  PR1 = 999;
  T1CON = 0x8010;
}

// Timer a 20ms
void initTimer2() {
    T2CON = 0;
    TMR2 = 0;
    PR2 = 19999;    // Ch. tick selon le prescaler
    T2CONbits.TCKPS = 0b01;  // Prescaler ralentir 8 MHz / 64

    _T2IP = 0X01;   // PRIORITE D'INTERRUPTION
    _T2IF = 0;      // Remise � z�ro
    _T2IE = 1;      // Activition de l'interruption
    T2CONbits.TON = 1;       // Activation du timer
}

void initTimers() {
    initTimer1();
    initTimer2();
}

// OCx --> Output Comparator
// Voir page 133 pour les config
void initOC1(void) {
    OC1CON = 0x0005; // Sur TM2 (bit3) voir page 133
    OC1R = 0;
    OC1RS = 1500;
}

void setOC1(int value) {
    OC1RS = value;
}


volatile unsigned long  nb_ms = 0;

// Toutes les ms
void _ISRFAST __attribute__((auto_psv)) _T1Interrupt(void)
{
  if(nb_ms > 0)   --nb_ms;
  commTicks++;
  
  
  _T1IF = 0;
}

// Tous 20 ms
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    
    if (nunchuck.bc) {
        btnCPressTime++;
    } else {
        btnCPressTime = 0;
    }
    
    if (currentState == CONFIG_MODE) {
        blinkAcc++;
    }
    
	IFS0bits.T2IF = 0;
}


void Delai(int ms)
{
  nb_ms = ms;
  while(nb_ms > 0);
}

void SendChar(unsigned char c)
{
  while(U1STAbits.UTXBF);
  U1TXREG = c;
}

// Format des donn�es
// jx = donnees[0];
// jy = donnees[1];
// ax = (donnees[2] << 2) + ((donnees[5] & 0x0C) >> 2);
// ay = (donnees[3] << 2) + ((donnees[5] & 0x30) >> 4);
// az = (donnees[4] << 2) + ((donnees[5] & 0xC0) >> 6);
// bz = (donnees[5] & 0x01) >> 0;
// bc = (donnees[5] & 0x02) >> 1;





struct Nunchuck convertNunchuckData(unsigned char data[6]) {
    struct Nunchuck result;   
    
//    jx = donnees[0];
//    jy = donnees[1];
//    ax = (donnees[2] << 2) + ((donnees[5] & 0x0C) >> 2);
//    ay = (donnees[3] << 2) + ((donnees[5] & 0x30) >> 4);    
//    az = (donnees[4] << 2) + ((donnees[5] & 0xC0) >> 6);
//    
//    bz = !(donnees[5] & 0x01);
//    bc = !((donnees[5] & 0x02) >> 1);
        
    result.jx = data[0];
    result.jy = data[1];
    result.ax = (data[2] << 2) + ((data[5] & 0x0C) >> 2);
    result.ay = (data[3] << 2) + ((data[5] & 0x30) >> 4);    
    result.az = (data[4] << 2) + ((data[5] & 0xC0) >> 6);
    
    result.bz = !(data[5] & 0x01);
    result.bc = !((data[5] & 0x02) >> 1);
    
    return result;
}

void calibrate() {
    slopeX = 1.0 * 19999 / (x_max - x_min);
    slopeY = 1.0 * 19999 / (y_max - y_min);
}

void modeRunning() {
    if (nunchuck.bc > 630)
    {
      _RB15 = 1;
    }
    else
    {
      _RB15 = 0;
    }
    
    // Transition vers etat de configuration
    if (btnCPressTime > 150) {
        btnCPressTime = 0;
        
        currentState = CONFIG_MODE;
    }
    
    setOC1(nunchuck.jx * slopeX);
}

void modeConfig() {
    // Transition vers etat d'execution
    if (btnCPressTime > 150) {
        btnCPressTime = 0;
        
        currentState = RUNNING_MODE;
    }

    if (blinkAcc > 25) {
        blinkAcc = 0;
        _RB15 = ~_RB15;
    }
    
    int dirtyData = 0;
    
    if (nunchuck.jx < x_min) {
        x_min = nunchuck.jx;
        dirtyData = 1;
    }
    
    if (nunchuck.jx > x_max) {
        x_max = nunchuck.jx;
        dirtyData = 1;
    }
    
    if (nunchuck.jy < y_min) {
        y_min = nunchuck.jy;
        dirtyData = 1;
    }
    
    if (nunchuck.jy > y_max) {
        y_max = nunchuck.jy;
        dirtyData = 1;
    }
    
    if (dirtyData) {
        calibrate();
    }
    
}


void manageSystem() {
    switch (currentState) {
        case RUNNING_MODE:
            modeRunning();
            break;
        case CONFIG_MODE:
            modeConfig();
            break;
        default:
            modeRunning();
            break;
    }
}


void initRxTx() {
#if LOCAL_NUNCHUCK
    // Configuration du port s�rie (UART)
    U1BRG  = 16;	// BRGH=1 16=115200
    U1STA  = 0x2000;	// Interruption � chaque caract�re re�u
    U1MODE = 0x8008;	// BRGH = 1

    U1STAbits.UTXEN = 1;
#else
    // Configuration du port s�rie (UART)
    U1BRG  = 51; // 
    //U1BRG = 0;

    U1STA  = 0x2000;	// Interruption � chaque caract�re re�u
    U1MODE = 0x8000;    // BRGH = 0

    U1STAbits.UTXEN = 1;

    _U1RXIP = 2; // Priorite
    _U1RXIF = 0;
    _U1RXIE = 1; 
#endif
}

int main(void)
{
  unsigned char donnees[6], i;
  //calibrate();
  
  
  InitRPs();
  
  AD1PCFG = 0xFFFF;
  TRISA = 0x0000;
  TRISB = 0x0004;
  
  initRxTx();
  

  
  initTimers();
  initOC1();
  _RB14 = 0;
  _RB15 = 0;
  
//  I2C_Initialisation();
//  
//  Delai(50);
//  
//  I2C_ConditionDemarrage();
//  I2C_Adresse(0x52, 0);
//  I2C_EnvoiOctet(0xF0);       // Sequence compatible aux deux modeles sans encodage
//  I2C_EnvoiOctet(0x55);
//  I2C_ConditionArret();
//  
//  Delai(10);
//  
//  I2C_ConditionDemarrage();
//  I2C_Adresse(0x52, 1);
//  I2C_LireOctets(donnees,6);
//  I2C_ConditionArret();
//  
  while (1)
  {
      
      if (commTicks > commInt) {
          commTicks = 0;
          _RB15 = ~_RB15;
          
          SendChar('w');
      }
      
      if (rxFlag) {
          rxFlag = 0;
          _RB14 = ~_RB14;
      }
    
//    Delai(1);
//    
//    I2C_ConditionDemarrage();
//    I2C_Adresse(0x52, 0);
//    I2C_EnvoiOctet(0x00);
//    I2C_ConditionArret();
//    
//    Delai(1);
//    
//    I2C_ConditionDemarrage();
//    I2C_Adresse(0x52, 1);
//    I2C_LireOctets(donnees,6);
//    I2C_ConditionArret();
//    
//    
//    // Communication avec GRAccel.exe
//    SendChar(0x5A);
//    
//    for(i = 0; i < 6; ++i) {
//      SendChar(donnees[i]);
//    }
//    
//    nunchuck = convertNunchuckData(donnees);
//    
//  
//    manageSystem();
    
  }
  return 0;
}


void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    rxFlag = 1;
    
//    rxChar = U1RXREG;   //Appel la fonction de lecture
//    
//    
//    
//    if (rxChar == 'w') {
//        forward = ~forward;
//        _RB7 = ~_RB7;
//    }
//    
//    // Mettre en mode config
//    if (rxChar == 'c') {
//        if (currentCartState == CONFIG_MODE ) {
//            currentCartState = RUNNING_MODE;
//        } else {
//            currentCartState = CONFIG_MODE;
//        }
//    }
//    
//    if (currentCartState == CONFIG_MODE ) {
//        if (rxChar == 's') {
//            speedCalibFlag = 1;
//            rTicks = 0;
//            lTicks = 0;
//            rwAcc = 0;
//            lwAcc = 0;
//        }
//    } else if (currentCartState == RUNNING_MODE) {
//        
//    }
    
    
    
    _U1RXIF = 0;
}