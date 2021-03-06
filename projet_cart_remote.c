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

void setProgState(progState);
void manageComm(void);
void manageSystem(void);
void manageBlinking(void);
void blinkingSetDelay (int, int);

// Variables globales;
unsigned int blinkRedAcc = 0;
int blinkRedDelay = 0;

unsigned int blinkGreenAcc = 0;
int blinkGreenDelay = 0;


// Nunchuck variables
volatile struct Nunchuck nunchuck;

int btnCPressTime = 0;
int btnCnZPressTime = 0;
int btnZPressTime = 0;

int x_min = 128;
int x_max = 128;
int y_min = 128;
int y_max = 128;
double slopeX = 1;
double slopeY = 1;

int xDeadZone = 16;
int yDeadZone = 16;
double rotateValue = 0;
double moveValue = 0;

// Output values
int rwValue = 128;
int lwValue = 128;

long commTicks = 0;
int commInt = 3000;
int rxFlag = 0;
long aliveAcc = 0;
int aliveHeartBeat = 5; // Envoie la communication � toutes les 5 ms

int config_delay = 2000;

void initRPs()
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
    aliveAcc++;
    
    blinkRedAcc++;
    blinkGreenAcc++;
  
    if (nunchuck.bz && nunchuck.bc) {
        btnCnZPressTime++;
        blinkingSetDelay(100, 100);
    } else {
        btnCnZPressTime = 0;
        
        if (nunchuck.bc) {
            btnCPressTime++;
        } else {
            btnCPressTime = 0;
            
        }

        if (nunchuck.bz) {
            btnZPressTime++;
            
        } else {
            btnZPressTime = 0;
            
        }
    }
    
    
  
  _T1IF = 0;
}

// Tous 20 ms
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    

    

    
	IFS0bits.T2IF = 0;
}


void delay(int ms)
{
  nb_ms = ms;
  while(nb_ms > 0);
}

void SendChar(unsigned char c)
{
  while(U1STAbits.UTXBF);
  U1TXREG = c;
}


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
    // Transition vers etat de configuration
    if (btnCnZPressTime > config_delay) {
        btnCnZPressTime = 0;
        
        blinkingSetDelay(200, 200);
        
        setProgState(CONFIG_MODE);
        return;
    } else if (btnCnZPressTime > 0) {
        blinkingSetDelay(100, 200);
        return;
    }
    
    blinkingSetDelay(1000, 1000);
    
    if (btnCPressTime > 500) {
        btnCPressTime = 0;
        
        
        rwValue = '!'; // Si les deux sont � 127, on entre en mode auto
        lwValue = 'x';
        _RB13 = _RB15 = 1;
        return;

    } else {
    
        // Src http://www.virtualroadside.com/WPILib/class_robot_drive.html#ac95118d7b535c4f3fb3d56ba5b041e40
        rotateValue = moveValue = 0;

        if (nunchuck.jx > 128 - xDeadZone && nunchuck.jx < 128 + xDeadZone) {
            rotateValue = 0;
        } else {
            rotateValue = (nunchuck.jx * 1.0) / 128 - 1;
        }

        if (nunchuck.jy > 128 - yDeadZone && nunchuck.jy < 128 + yDeadZone) {
            moveValue = 0;
        } else {
            moveValue = (nunchuck.jy * 1.0) / 128 - 1;
        }

        rwValue = 255 - ((moveValue + rotateValue) + 1) * 128 ;
        lwValue = 255 - ((moveValue - rotateValue) + 1) * 128;
        return;
    }
    

    
}


void modeConfig() {
    // Transition vers etat d'execution
    if (btnCnZPressTime > config_delay) {
        btnCnZPressTime = 0;
        
        blinkingSetDelay(1000, 1000);
        
        setProgState(RUNNING_MODE);
        return;
    } else if (btnCnZPressTime > 0) {
        blinkingSetDelay(200, 200);
        return;
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

void blinkingSetDelay (int red, int green) {
    blinkRedDelay = red;
    blinkGreenDelay = green;
}

void manageBlinking() {
    if (blinkRedAcc > blinkRedDelay) {
        blinkRedAcc = 0;
        _RB13 = ~_RB13;
    }
    
    if (blinkGreenAcc > blinkGreenDelay) {
        blinkGreenAcc = 0;
        _RB15 = ~_RB15;
    }
}

void manageComm() {
    
    if (aliveAcc < aliveHeartBeat) {
        return;
    }
    
    aliveAcc = 0;

    if (currentState == RUNNING_MODE) {
        SendChar(0x55);
        SendChar(rwValue);
        SendChar(lwValue);
    } else {
        SendChar(0x00);
    }
    
    
}


void manageSystem() {
    
    // Arr�t d'urgence
    if (btnZPressTime > 0) {
        btnZPressTime = 0;
        
        rwValue = lwValue = 128;
        return;
    }
    
    
    
    switch (currentState) {
        case RUNNING_MODE:
            modeRunning();
            break;
        case CONFIG_MODE:
            modeConfig();
            break;
        default:
            rwValue = lwValue = 128;
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

void setProgState(progState newState) {
    currentState = newState;
}

int main(void)
{
  unsigned char data[6];
  //calibrate();
  
  
  initRPs();
  
  AD1PCFG = 0xFFFF;
  TRISA = 0x0000;
  TRISB = 0x0004;
  
  initRxTx();
  
  initTimers();
  initOC1();

  PORTB = 0x0000;
  
  I2C_Initialisation();
  
  delay(50);
  
  I2C_ConditionDemarrage();
  I2C_Adresse(0x52, 0);
  I2C_EnvoiOctet(0xF0);       // Sequence compatible aux deux modeles sans encodage
  I2C_EnvoiOctet(0x55);
  I2C_ConditionArret();
  
  delay(10);
  
  I2C_ConditionDemarrage();
  I2C_Adresse(0x52, 1);
  I2C_LireOctets(data,6);
  I2C_ConditionArret();
    
  lwValue = rwValue = 128;
  blinkRedDelay = blinkGreenDelay = 1000;
  
  while (1)
  {      
      
    if (rxFlag) {
        // G�rer la communication entrante ici
        rxFlag = 0;

    }
    
    delay(1);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 0);
    I2C_EnvoiOctet(0x00);
    I2C_ConditionArret();
    
    delay(1);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 1);
    I2C_LireOctets(data,6);
    I2C_ConditionArret();
    
    nunchuck = convertNunchuckData(data);

    manageSystem();
    manageComm();
    manageBlinking();
    
  }
  return 0;
}





void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    rxFlag = 1;
    
    
    
    
    _U1RXIF = 0;
}