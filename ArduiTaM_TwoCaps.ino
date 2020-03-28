#include <Wire.h>
#include <SPI.h>

/**********************Initializing Tunning and Matching step motors ************************/
// Define stepper motor connections and steps per revolution:
#define Tunning_dirPin  2
#define Tunning_stepPin 3
#define Matching_dirPin  4
#define Matching_stepPin 5
#define stopm_pin 6
#define stopt_pin 7

int stepsPerRevolution=20;
int step_delay=1000; //microseconds 500 extremely fast//5000
//unsigned int turns=28; 
unsigned int cap_steps;//=4;
unsigned int size_step;//= turns*stepsPerRevolution/cap_steps;
//int TandM_Matrix[8][8];

/********************************************************************************************/

/************     Hardware connection Arduino Uno to Synthesizer-Board (3.3V logic)     *****
                                  +-----+
     +----[PWR]-------------------| USB |--+
     |                            +-----+  |                        output: 33Mhz - 4.4Ghz ~ 3 dbm
     |         GND/RST2  [ ][ ]            |             
     |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |                                       #
     |          5V/MISO2 [ ][ ]  A4/SDA[ ] |    |---|_3.3k_|-----\                 |
     |                             AREF[ ] |    !---|_3.3k_|---\ |            _____|______
     |                              GND[ ] |----!---|_3.3k_|-| | |           |            |
     | [ ]N/C                    SCK/13[ ] |------|_1.5k_|---!-|-|----Clock--|ADF4351     |
     | [ ]v.ref                 MISO/12[ ] |                   | |           |PLL Board   |
     | [ ]RST                   MOSI/11[ ]~|------|_1.5k_|-----!-|----Data---|3.3V logic  |
     | [ ]3V3    +---+               10[ ]~|------|_1.5k_|---LE--/           |            |            
     | [ ]5v     | A |                9[ ]~|   LCD               |----LE-----|            |
     | [ ]GND   -| R |-               8[ ] |   LCD                           |____________|
     | [ ]GND   -| D |-                    |                     
     | [ ]Vin   -| U |-               7[ ] |   LCD               
     |          -| I |-               6[ ]~|   LCD               
Keyb | [ ]A0    -| N |-               5[ ]~|   LCD               
RSSI | [ ]A1    -| O |-               4[ ] |   LCD               
     | [ ]A2     +---+           INT1/3[ ]~|
     | [ ]A3                     INT0/2[ ] |
     | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |
     | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |
     |            [ ] [ ] [ ]              |
     |  UNO       GND MOSI 5V  ____________/
      \_______________________/

*/
/**********************Initializing freq generator ******************************************/
int envelopePin = A0;    // select the input pin for the envelope signal amplified
int envelopeValue = 1024;  // variable to store the value coming from the envelope signal
int min_value = 1024;   // variable to store the max envelope value
unsigned int cm_min = 0;   // variable to store the tunning capacitor for the max envelope value
unsigned int ct_min =0; // variable to store the matching capacitor for the max envelope value


const int slaveSelectPin = 10;  // Enabling pin for: ADF4350  (SPI-SS)

long CFreq = 4493000;  //Larmor frequency divided by 10
long Freq;  //current frequency
long refin = 2500000; // Refrenquarz = 25Mhz
long ChanStep = 1000; //{ 625, 1000, 2500, 5000, 10000, 50000, 100000} Steps from 6.25 Khz to 1MHz
long FreqSpan = 1000000;
long FreqHigh;
long FreqLow;

unsigned long Reg[6]; //ADF4351 Reg's

byte tenHz, hundredHz, ones, tens, hundreds, thousands, tenthousands, hundredthousands, millions; 
/*********************************************************************************************************/



void setup() {
  Serial.begin(9600);
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, LOW);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin();
  delay(500);
  SetFreq(CFreq);
  Freq=CFreq;
  
  // Declare pins as output for StepMotors control:
  pinMode(Tunning_stepPin, OUTPUT);
  pinMode(Tunning_dirPin , OUTPUT);
  pinMode(Matching_stepPin, OUTPUT);
  pinMode(Matching_dirPin , OUTPUT);
  // if stop sensor installed here should run an initializing position
  main_routine();
}

/*  void loop() EMPTY : after tunning and matching Arudino does nothing */

void loop() {

}

/*  MAIN ROUTIN : tunning and matching occur only once */

void main_routine (){
  Serial.println("Type_something!");
  while(!Serial.available());
  SetFreq(Freq);
  min_value=1024;
  cm_min=0;
  ct_min=0;
  int m = 0;
  int t = 0;
  
  //****************FIRST CYCLE***************************//
  size_step=64;
  cap_steps=16;// depending on the stepper motor, this sets the number of turns
  /*********   Initial homing: go beyond the limit **********************/
  stepper_rotate(Matching_dirPin, Matching_stepPin, LOW, 10); 
  stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, 10);
  /* IF STOP SENSOR IS INSTALLED THEN CHANGE coment previous lines and uncoment 
   **************************  'MOVE WHILE'LINES *****************************/
  //while (!stopm_pin) {stepper_rotate(Matching_dirPin, Matching_stepPin, LOW, 1);}
  //while (!stopt_pin) {stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, 1);}

  
  for ( m = 0; m < cap_steps+1; m++) 
  {
    for ( t = 0; t < cap_steps+1; t++) 
    { 
        envelope(m,t);
        stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step);
    }

    stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, size_step*(cap_steps+1));
    stepper_rotate(Matching_dirPin, Matching_stepPin, HIGH, size_step);
  }
  stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step*(cap_steps+1));
  
  //Serial.print(cm_min);Serial.print(",");Serial.print(ct_min);Serial.print(",");Serial.println(min_value);


  //****************SECOND to FOURTH CYCLE***************************//
  int i=0;
  while (i<3){
      
      stepper_rotate(Matching_dirPin, Matching_stepPin, LOW, size_step*(cap_steps+1)); //RETURN TO POSITION 0 WITHIN THE ACTUAL RANGE OF MOVEMENT
      stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, size_step*(cap_steps+1));
      if (cm_min<1) {cm_min++;}
      if (cm_min>(cap_steps-1)) {cm_min--;}
      stepper_rotate(Matching_dirPin, Matching_stepPin, HIGH, size_step*(cm_min-1)); // GO TO POSITION 0 OF THE RANGE OF MOVEMENT
      if (ct_min<1) {ct_min++;}
      if (ct_min>(cap_steps-1)) {ct_min--;}
      stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step*(ct_min-1));
    
      min_value=1024;
      cm_min=0;
      ct_min=0;
      size_step=size_step/4;
      
      for ( m = 0; m < cap_steps+1; m++) 
      {
        for ( t = 0; t < cap_steps+1; t++) 
        { 
            envelope(m,t);
            stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step);
        }
    
        stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, size_step*(cap_steps+1));
        stepper_rotate(Matching_dirPin, Matching_stepPin, HIGH, size_step);
      }
      stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step*(cap_steps+1));
      
      //Serial.print(cm_min);Serial.print(",");Serial.print(ct_min);Serial.print(",");Serial.println(min_value);
      i++;
  }
  //****************FINAL POSITIONING ***************************//
    stepper_rotate(Matching_dirPin, Matching_stepPin, LOW, size_step*(cap_steps+1)); //RETURN TO POSITION 0 WITHIN THE ACTUAL RANGE OF MOVEMENT
    stepper_rotate(Tunning_dirPin, Tunning_stepPin, LOW, size_step*(cap_steps+1));
    stepper_rotate(Matching_dirPin, Matching_stepPin, HIGH, size_step*(cm_min)); // GO TO POSITION 0 OF THE RANGE OF MOVEMENT
    stepper_rotate(Tunning_dirPin, Tunning_stepPin, HIGH, size_step*(ct_min));
}
/************************************* READING ENVELOPE *******************************************/
void envelope(unsigned int m, unsigned int t){
    delay (50);
    envelopeValue = analogRead(envelopePin);
    if (envelopeValue<min_value)
    {
      min_value=envelopeValue;
      cm_min=m;
      ct_min=t;
    }
    Serial.println(envelopeValue);
}

/********************************* ADF FUNTIONS *****************************************/
void stepper_rotate(unsigned int dirPin, unsigned int stepPin, bool dir, long st){
  digitalWrite(dirPin, dir);
  long i = 0;
  while( i < st ){
      // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(step_delay);
    i++;
  }
}


void SetFreq(long Freq)
{
  ConvertFreq(Freq, Reg);
  WriteADF2(5);   //delayMicroseconds(2500);
  WriteADF2(4);   //delayMicroseconds(2500);
  WriteADF2(3);   //delayMicroseconds(2500);
  WriteADF2(2);   //delayMicroseconds(2500);
  WriteADF2(1);   //delayMicroseconds(2500);
  WriteADF2(0);   //delayMicroseconds(2500);
}


void WriteADF2(int idx)
{ // make 4 byte from integer for SPI-Transfer
  byte buf[4];
  for (int i = 0; i < 4; i++)
    buf[i] = (byte)(Reg[idx] >> (i * 8));
  WriteADF(buf[3], buf[2], buf[1], buf[0]);
}
int WriteADF(byte a1, byte a2, byte a3, byte a4) {
  // write over SPI to ADF4350
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(10);
  SPI.transfer(a1); 
  SPI.transfer(a2); 
  SPI.transfer(a3); 
  SPI.transfer(a4); 
  Toggle();
}


int Toggle() {
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(slaveSelectPin, LOW);
}


void ConvertFreq(long freq, unsigned long R[])
{
  // PLL-Reg-R0         =  32bit
  // Registerselect        3bit
  // int F_Frac = 4;       // 12bit
  // int N_Int = 92;       // 16bit
  // reserved           // 1bit

  // PLL-Reg-R1         =  32bit
  // Registerselect        3bit
  //int M_Mod = 5;        // 12bit
  int P_Phase = 1;     // 12bit bei 2x12bit hintereinander pow()-bug !!
  int Prescal = 0;     // 1bit geht nicht ???
  int PhaseAdj = 0;    // 1bit geht auch nicht ???
  // reserved           // 3bit

  // PLL-Reg-R2         =  32bit
  // Registerselect        3bit
  int U1_CountRes = 0; // 1bit
  int U2_Cp3state = 0; // 1bit
  int U3_PwrDown = 0;  // 1bit
  int U4_PDpola = 1;    // 1bit
  int U5_LPD = 0;       // 1bit
  int U6_LPF = 1;       // 1bit 1=Integer, 0=Frac not spported yet
  int CP_ChgPump = 7;     // 4bit
  int D1_DoublBuf = 0; // 1bit
  //  int R_Counter = 1;   // 10bit
  //  int RD1_Rdiv2 = 0;    // 1bit
  //  int RD2refdoubl = 0; // 1bit
  int M_Muxout = 0;     // 3bit
  int LoNoisSpur = 0;      // 2bit
  // reserved           // 1bit

  // PLL-Reg-R3         =  32bit
  // Registerselect        3bit
  int D_Clk_div = 150; // 12bit
  int C_Clk_mode = 0;   // 2bit
  //  reserved          // 1bit
  int F1_Csr = 0;       // 1bit
  //  reserved          // 2bit
  int F2_ChgChan = 0;   // 1bit
  int F3_ADB = 0;       // 1bit
  int F4_BandSel = 0;  // 1bit
  //  reserved          // 8bit

  // PLL-Reg-R4         =  32bit
  // Registerselect        3bit
  int D_out_PWR = 0 ;    // 2bit
  int D_RF_ena = 1;     // 1bit
  int D_auxOutPwr = 0;  // 2bit
  int D_auxOutEna = 0;  // 1bit
  int D_auxOutSel = 0;  // 1bit
  int D_MTLD = 0;       // 1bit
  int D_VcoPwrDown = 0; // 1bit 1=VCO off

  //  int B_BandSelClk = 200; // 8bit

  int D_RfDivSel = 3;    // 3bit 3=70cm 4=2m
  int D_FeedBck = 1;     // 1bit
  // reserved           // 8bit

  // PLL-Reg-R5         =  32bit
  // Registerselect     // 3bit
  // reserved           // 16bit
  // reserved     11    // 2bit
  // reserved           // 1bit
  int D_LdPinMod = 1;    // 2bit muss 1 sein
  // reserved           // 8bit

  // Referenz Freg Calc
  //  long refin = 250000; // Refrenquarz = 25000000hz
  int R_Counter = 1;   // 10bit
  int RD1_Rdiv2 = 0;    // 1bit
  int RD2refdoubl = 0; // 1bit
  int B_BandSelClk = 200; // 8bit
  //  int F4_BandSel = 0;  // 1bit

  // int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;

  long RFout = Freq;   // VCO-Frequenz
  // calc bandselect und RF-div
  int outdiv = 1;

  if (RFout >= 220000000) {
    outdiv = 1;
    D_RfDivSel = 0;
  }
  if (RFout < 220000000) {
    outdiv = 2;
    D_RfDivSel = 1;
  }
  if (RFout < 110000000) {
    outdiv = 4;
    D_RfDivSel = 2;
  }
  if (RFout < 55000000) {
    outdiv = 8;
    D_RfDivSel = 3;
  }
  if (RFout < 27500000) {
    outdiv = 16;
    D_RfDivSel = 4;
  }
  if (RFout < 13800000) {
    outdiv = 32;
    D_RfDivSel = 5;
  }
  if (RFout < 6900000) {
    outdiv = 64;
    D_RfDivSel = 6;
  }

  float PFDFreq = refin * ((1.0 + RD2refdoubl) / (R_Counter * (1.0 + RD1_Rdiv2))); //Referenzfrequenz
  float N = ((RFout) * outdiv) / PFDFreq;
  int N_Int = N;
  long M_Mod = PFDFreq * (100000 / ChanStep) / 100000;
  int F_Frac = round((N - N_Int) * M_Mod);

  R[0] = (unsigned long)(0 + F_Frac * pow(2, 3) + N_Int * pow(2, 15));
  R[1] = (unsigned long)(1 + M_Mod * pow(2, 3) + P_Phase * pow(2, 15) + Prescal * pow(2, 27) + PhaseAdj * pow(2, 28));
  //  R[1] = (R[1])+1; // Registerselect adjust ?? because unpossible 2x12bit in pow() funktion
  R[2] = (unsigned long)(2 + U1_CountRes * pow(2, 3) + U2_Cp3state * pow(2, 4) + U3_PwrDown * pow(2, 5) + U4_PDpola * pow(2, 6) + U5_LPD * pow(2, 7) + U6_LPF * pow(2, 8) + CP_ChgPump * pow(2, 9) + D1_DoublBuf * pow(2, 13) + R_Counter * pow(2, 14) + RD1_Rdiv2 * pow(2, 24) + RD2refdoubl * pow(2, 25) + M_Muxout * pow(2, 26) + LoNoisSpur * pow(2, 29));
  R[3] = (unsigned long)(3 + D_Clk_div * pow(2, 3) + C_Clk_mode * pow(2, 15) + 0 * pow(2, 17) + F1_Csr * pow(2, 18) + 0 * pow(2, 19) + F2_ChgChan * pow(2, 21) +  F3_ADB * pow(2, 22) + F4_BandSel * pow(2, 23) + 0 * pow(2, 24));
  R[4] = (unsigned long)(4 + D_out_PWR * pow(2, 3) + D_RF_ena * pow(2, 5) + D_auxOutPwr * pow(2, 6) + D_auxOutEna * pow(2, 8) + D_auxOutSel * pow(2, 9) + D_MTLD * pow(2, 10) + D_VcoPwrDown * pow(2, 11) + B_BandSelClk * pow(2, 12) + D_RfDivSel * pow(2, 20) + D_FeedBck * pow(2, 23));
  R[5] = (unsigned long)(5 + 0 * pow(2, 3) + 3 * pow(2, 19) + 0 * pow(2, 21) + D_LdPinMod * pow(2, 22));
}
