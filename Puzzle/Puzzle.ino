#include <Event.h>
#include <Timer.h>

#include "PuzzlePins.h"
Timer t;

//Easy VR
#include <EasyVR.h>
EasyVR easyvr(VOICESERIAL);
bool alreadyListening = false;
int8_t group,idx;
enum Wordset3 
{
  S3_ZERO = 0,
  S3_ONE = 1,
  S3_TWO = 2,
  S3_THREE = 3,
  S3_FOUR = 4,
  S3_FIVE = 5,
  S3_SIX = 6,
  S3_SEVEN = 7,
  S3_EIGHT = 8,
  S3_NINE = 9,
  S3_TEN = 10,
};
int8_t currentWord = -1;

//***************

#include <MFRC522.h>
#include <EEPROM.h>     // We are going to read and write PICC's UIDs from/to EEPROM
#include <SPI.h>
#include <Servo.h>
Servo Scale1Servo;
Servo Scale2Servo;



//RFID setup*************
String bytesToChar(byte* b, uint8_t b_size);
void readAllRFIDS();
void setupRFID();
const byte ssPins[] = {RFIDNOSE, RFID1,RFID2,RFID3,RFID4 };
bool isRFIDSetup[] = {false, false, false, false, false};
MFRC522 mfrc522[NUMREADERS];
String currentIDs[NUMREADERS];
String correctIDs[] = {RFIDNOSEANSWER,RFID1ANSWER,RFID2ANSWER,RFID3ANSWER,RFID4ANSWER};
//***********************

//puzzle setup**********************************
bool dialSolved = false;
bool scaleSolved = false;
bool eyesSolved = false;
bool noseSolved =  false;
bool voiceSolved = false;
bool gemsSolved =false;
bool prizeSolved = false;
//*******************

//Button Setup**********************************
volatile bool btnDialPressed;
bool dialLightOn = 0;
volatile bool btnRedPressed;
volatile bool btnYellowPressed;
volatile bool btnGreenPressed;
volatile bool btnBluePressed;
int btnPins[] = {BTNRED,BTNBLUE,BTNYELLOW, BTNGREEN};
int lightPins[] = {BTNREDLIGHT, BTNBLUELIGHT, BTNYELLOWLIGHT, BTNGREENLIGHT};
int btnPressed[NUMBTNS];
//*********************************************


///********SIMON SETUP***********
int sequence[MAX_LEVEL];
int your_sequence[MAX_LEVEL];
int level = 1;
int velocity = 1000;
//***************************



void setup() {
  delay(5000);
  Serial.begin(9600);
  Serial.println("Initializing: ");
  delay(2000);
  // put your setup code here, to run once:
  setupRFID();
  setupHallEffect();
  setupButtons();
  setupScale();
  setupSolenoids();
  setupVoiceModule();

  Serial.println("Finished Setup");
  digitalWrite(BTNDIALLIGHT, HIGH);
}

int8_t prevWord = -1;
#define OUTSIDEPUZZLES 1
#define WAITFORHEADRELEASE  2
#define INTERIORPUZZLE  3
#define WON 4
int state = OUTSIDEPUZZLES;

void loop() {
  runPuzzle();
  // test(); 
}


void testSolenoid(uint8_t pin){
    openLock(pin, 1000);
 
}

void testSimon(){
  while(1){
   if (level == 1){
        Serial.println("generating sequence");
        generate_sequence();//generate a sequence;
      }      
      show_sequence();    //show the sequence
      get_sequence();     //wait for your sequence
      if(level >= 2){      
        Serial.println("won simon");
        break;
        //openLock(PRIZELOCK);
      }
  }
}

void test(){

    digitalWrite(BTNDIALLIGHT, HIGH);
    digitalWrite(BTNREDLIGHT, HIGH);
    digitalWrite(BTNYELLOWLIGHT, HIGH);
    digitalWrite(BTNBLUELIGHT, HIGH);
    digitalWrite(BTNGREENLIGHT, HIGH);

    testSolenoid(DIALDRAWERLOCK);
    // digitalWrite(HEADLOCKS, LOW);
    testSolenoid(HEADLOCKS);
    testSolenoid(SCALEMOUTHLOCK);
    testSolenoid(NOSELOCK);
    testSolenoid(VOICEMOUTHLOCK);
    testSolenoid(EYEMOUTHLOCK);
    testSolenoid(PRIZELOCK);

    // testSimon();

    while(1){
      t.update();

      // for(int i = 0; i<NUMBTNS; i++){
      //   Serial.print(btnPressed[i]);
      // }
      // Serial.println(btnDialPressed);
      handleDial();
      handleScales();
      handleEyes();
      handleVoice();
      handleRFID();
    }


}

int whiteButtonTimerID;

void runPuzzle(){
  t.update();
  switch(state){
    case OUTSIDEPUZZLES:{
      //this case will check dial, scales, photoresistors, and nose rfid
      handleDial();
      handleScales();
      handleEyes();
      handleVoice();
      handleRFID();
      if(gemsSolved){ //outside puzzles solved switch to checking gems
        state = WAITFORHEADRELEASE;
        whiteButtonTimerID = t.oscillate(BTNDIALLIGHT, 500, 200);
        scalesOff();
      }
      break;
    }
    case WAITFORHEADRELEASE:{

      if(btnDialPressed){
        Serial.println("White button pressed starting simon");
        t.stop(whiteButtonTimerID);
        digitalWrite(BTNDIALLIGHT, HIGH);
        // openLock(HEADLOCKS, 500); //in future
        state = INTERIORPUZZLE;
        delay(500);
      }
      break;
    }
    case INTERIORPUZZLE:{
      if (level == 1){
        generate_sequence();//generate a sequence;
      }      
      show_sequence();    //show the sequence
      get_sequence();     //wait for your sequence
      if(level >= MAX_LEVEL){      
        openLock(PRIZELOCK, 500);
        state = WON;
      }
      break;
    }
    case WON:{
      break;
    }
  }
}
///********SIMON FUNCTIONS***********************

void generate_sequence()
{
  randomSeed(millis()); //in this way is really random!!!

  for (int i = 0; i < MAX_LEVEL; i++)
  {
    sequence[i] = random(0,4);
  }
}
void show_sequence()
{
  Serial.print("showing sequence: ");

  for(int pin : lightPins){
    digitalWrite(pin, LOW);   
  }
  delay(1000);

  for (int i = 0; i < level; i++)
  {
    Serial.println("Lighting ");
    Serial.print(sequence[i]);
    Serial.print("  ");
    digitalWrite(lightPins[sequence[i]], HIGH);
    delay(velocity);
    digitalWrite(lightPins[sequence[i]], LOW);
    delay(200);
  }
  Serial.println();
}

void get_sequence(){
int flag = 0; //this flag indicates if the sequence is correct
  for (int i = 0; i < level; i++){
    flag = 0;
    while(flag == 0){
      for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
        if(btnPressed[btnNum] == 1){
          Serial.print("Button pressed: ");
          Serial.println(btnNum);
          digitalWrite(lightPins[btnNum], HIGH);
          your_sequence[i] = btnNum;
          flag = 1;
          delay(200);
          if(your_sequence[i] != sequence[i]){
            wrong_sequence();
            return;
          }
          digitalWrite(lightPins[btnNum], LOW);
        }
      }
    }
  }
right_sequence();
}

void wrong_sequence()
{
  for (int i = 0; i < 3; i++)
  {
    for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
      digitalWrite(btnPins[btnNum], HIGH);
    }
    delay(250);
    for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
      digitalWrite(btnPins[btnNum], HIGH);
    }
    delay(250);
  }
  level = 1;
  velocity = 1000;
}

void right_sequence()
{
  for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
    digitalWrite(btnPins[btnNum], LOW);
  }
  delay(250);
  for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
      digitalWrite(btnPins[btnNum], HIGH);
  }
  delay(500);
  for(int btnNum = 0; btnNum<NUMBTNS; btnNum++){
      digitalWrite(btnPins[btnNum], LOW);
    }
  delay(500);

  if (level < MAX_LEVEL){
    level++;
  }
velocity -= 50; //increase difficulty
}


//*********************************************

//*************RFID FUNCTIONS******************************************************
void setupRFID(){
  Serial.println("Setting up RFID");

  delay(2000);
  SPI.begin();
  SPI.setMOSI(RFIDMOSI);
  SPI.setMISO(RFIDMISO);
  SPI.setSCK(RFIDSCLK);
  Serial.println("Looking for RFID Reader");

  for(uint8_t i = 0; i<NUMREADERS;i++){
    mfrc522[i].PCD_Init(ssPins[i], RFIDRESET);
    Serial.print("Reader #");
    Serial.print(i);
    Serial.print(" on pin ");
    Serial.println(ssPins[i]);
    mfrc522[i].PCD_SetAntennaGain(MFRC522::PCD_RxGain::RxGain_max);
    Serial.print("Antenna Stength: ");
    Serial.println(mfrc522[i].PCD_GetAntennaGain());
    if(mfrc522[i].PCD_GetAntennaGain() == MFRC522::PCD_RxGain::RxGain_max){
      isRFIDSetup[i] = true;
      Serial.println("Successfully setup");
    }
    Serial.print("Version: ");
    mfrc522[i].PCD_DumpVersionToSerial();
    mfrc522[i].PICC_HaltA();
    mfrc522[i].PCD_StopCrypto1();
    delay(100);
  }
  Serial.println("Done setting up rfid");

}

long int noseOpenTime = 0;
void handleRFID(){
   if(millis() - noseOpenTime > 5000){
    digitalWrite(DIALDRAWERLOCK, LOW);
  }
  if(!gemsSolved || !noseSolved){
    readAllRFIDS();
    bool nose = isRfidCorrect(RFIDNOSEIDX);
    if(nose && !noseSolved){
      noseSolved = true;
      Serial.println("Nose solved");
      digitalWrite(NOSELOCK, HIGH);
      noseOpenTime = millis();
      // openLock(NOSELOCK, 5000);
    }

    bool gems = isRfidCorrect(RFID1IDX) && isRfidCorrect(RFID2IDX) && isRfidCorrect(RFID4IDX);
    if(gems && !gemsSolved){
      Serial.println("Gems Solved");
      gemsSolved = true;
    }
  }
}

void readAllRFIDS(){
  bool valueChanged = false;
  for(int i = 0; i< NUMREADERS; i++){
    if(isRFIDSetup[i]){
    // int i = 4;
      // Serial.print("reading ");
      // Serial.println(i);
      mfrc522[i].PCD_Init();
      String readRFID = "";
      if(mfrc522[i].PICC_IsNewCardPresent() && mfrc522[i].PICC_ReadCardSerial()){
        //extract ID
        readRFID = bytesToString(mfrc522[i].uid.uidByte, mfrc522[i].uid.size);
      }
      if(readRFID!= currentIDs[i]){
        currentIDs[i] = readRFID;
        valueChanged = true;
      }
      mfrc522[i].PICC_HaltA();
      mfrc522[i].PCD_StopCrypto1();
    }
  }
  if(valueChanged){
    for(int i = 0; i < NUMREADERS; i++){
      Serial.print("Read RFID #");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(currentIDs[i]);
    }
    Serial.println();
    Serial.println();
  }
}

bool isRfidCorrect(uint8_t idx){
  return currentIDs[idx] == correctIDs[idx];
}


String bytesToString(byte* b, uint8_t b_size){
  char dest[(b_size+1)*2];
  memset(dest,0,sizeof(dest));
  for(int i = 0; i< b_size; i++){
    // convert byte to its ascii representation
    sprintf(&dest[i * 2], "%02X", b[i]);
  } 
  return String(dest);
}//**********************************************************************



//*******SOlenoid functions************************************************************
void setupSolenoids(){
  Serial.println("Setting up solenoids");

  pinMode(SCALEMOUTHLOCK, OUTPUT);
  pinMode(VOICEMOUTHLOCK, OUTPUT);
  pinMode(EYEMOUTHLOCK, OUTPUT);
  pinMode(NOSELOCK, OUTPUT);
  pinMode(HEADLOCKS, OUTPUT);
  pinMode(DIALDRAWERLOCK, OUTPUT);
  pinMode(PRIZELOCK, OUTPUT);
}
void openLock(uint8_t pin, int time){
  Serial.print("Opening pin: ");
  Serial.println(pin);
  digitalWrite(pin, HIGH);
  t.pulse(pin, time, LOW);
  // digitalWrite(pin, HIGH);
  // delay(200);
  // digitalWrite(pin, LOW);
}
void closeLock(uint8_t pin){
  digitalWrite(pin, LOW);
}
//******************************************************************


//**********DIAL FUNCTIONS*****************************************
void setupHallEffect(){
  Serial.println("Setting up Hall effect");
  pinMode(OUTERHALL, INPUT_PULLUP);
  pinMode(MIDHALL, INPUT_PULLUP);
  pinMode(INNERHALL, INPUT_PULLUP);
}

long int dialOpenTime;
void handleDial(){
  if(millis() - dialOpenTime > 5000){
    digitalWrite(DIALDRAWERLOCK, LOW);
  }
  if(!dialSolved){
    bool solved = isDialSolved();
    if(solved && !dialSolved){
      dialSolved = true;
      Serial.println("Dial Solved");
      digitalWrite(DIALDRAWERLOCK, HIGH);
      dialOpenTime = millis();
      //openLock(DIALDRAWERLOCK, 5000);
    }
  }
}
bool isDialSolved(){
  int thresh = 75; //any analog read values less than this will return true: 
  //sensors read 1 when magnet is present
  int val1 = analogRead(OUTERHALL);
  int val2 = analogRead(MIDHALL);
  int val3 = analogRead(INNERHALL);

  // Serial.print("OUTERHALL: ");
  // Serial.print(val1);
  // Serial.print("  MIDHALL: ");
  // Serial.print(val2);
  // Serial.print("  INNERHALL: ");
  // Serial.println(val3);


  if(val2<thresh){ // && val2 < thresh && val3 < thresh){ //disabled all except outer hall since its the only one that works
    return true;
  }
  return false;
}
//************************************************


//**************EYE FUNCTIONS********************
void setupEyes(){
  Serial.println("Setting up eyes");

  pinMode(EYE1, INPUT);
  // pinMode(EYE2, INPUT);
}
bool areEyesSolved(){
  int threshold = 200;
  int val1 = analogRead(EYE1);
  // int val2 = analogRead(EYE2);
  // Serial.println("eye");
  // Serial.println(val1);
  if(val1 < threshold){
    return true;
  }
  return false;
}

void handleEyes(){
  if(!eyesSolved){
    bool solved = areEyesSolved();
    if(solved && !eyesSolved){
      eyesSolved = true;
      Serial.println("Eyes solved");
      openLock(EYEMOUTHLOCK, 500);
    }
  }
}

//***************************************************


//*******VOICE FUNCTIONS********************************************
void setupVoiceModule(){
  Serial.println("Setting up voice");

  bridge:
  // bridge mode?
  int mode = easyvr.bridgeRequested(Serial);
  switch (mode)
  {
  case EasyVR::BRIDGE_NONE:
    // setup EasyVR serial VOICESERIAL
    VOICESERIAL.begin(9600);
    // run normally
    Serial.println(F("Bridge not requested, run normally"));
    Serial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_NORMAL:
    // setup EasyVR serial VOICESERIAL (low speed)
    VOICESERIAL.begin(9600);
    // soft-connect the two serial VOICESERIALs (PC and EasyVR)
    easyvr.bridgeLoop(Serial);
    // resume normally if aborted
    Serial.println(F("Bridge connection aborted"));
    Serial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_BOOT:
    // setup EasyVR serial VOICESERIAL (high speed)

    VOICESERIAL.begin(115200);
    Serial.end();
    Serial.begin(115200);
    Serial.println("High speed connection fo voice");
    // soft-connect the two serial VOICESERIALs (PC and EasyVR)
    easyvr.bridgeLoop(Serial);
    // resume normally if aborted
    Serial.println(F("Bridge connection aborted"));
    Serial.println(F("---"));
    break;
  }
  group = EasyVR::NUMBER_SET; //start group (customize)


  // initialize EasyVR  
  while (!easyvr.detect())
  {
    Serial.println(F("EasyVR not detected!"));
    for (int i = 0; i < 10; ++i)
    {
      if (Serial.read() == EasyVR::BRIDGE_ESCAPE_CHAR)
        goto bridge;
      delay(100);
    }
  }

  Serial.print(F("EasyVR detected, version "));
  Serial.print(easyvr.getID());
  easyvr.setPinOutput(EasyVR::IO1, LOW); //Low is not listening
  Serial.println("EasyVR detected!");
  easyvr.setDelay(0); // speed-up replies

  easyvr.setTimeout(31);
  easyvr.setMicDistance(EasyVR::HEADSET);
  easyvr.setLanguage(EasyVR::ENGLISH);
  easyvr.setKnob(EasyVR::LOOSE);//confidence level LOOSER Lowest threshold, most results reported
                    //LOOSE Lower threshold, more results reported
                    //TYPICAL Typical threshold (default)
                    //STRICT Higher threshold, fewer results reported
                    //STRICTER Highest threshold, fewest results reported
  alreadyListening = false;
}

void checkVoiceModule(){
  // allows Commander to request bridge on Zero (may interfere with user protocol)
  if (Serial.read() == '?')
  {
    easyvr.stop();
    setup();
    return;
  }
  if(!alreadyListening){
    easyvr.setPinOutput(EasyVR::IO1,HIGH);
    Serial.print("Say a command in Group ");
    Serial.println(group);
    easyvr.recognizeWord(group);
    alreadyListening = true;
  }
  if(easyvr.hasFinished()){
    easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off
    alreadyListening=false;
    idx = easyvr.getWord();
      if (idx >= 0)
      {
      // print debug message
        currentWord = idx;
        uint8_t flags = 0, num = 0;
        char name[32];
        Serial.print("Word: ");
        Serial.print(idx);
        if (easyvr.dumpGrammar(-group, flags, num))
        {
          for (uint8_t pos = 0; pos < num; ++pos)
          {
            if (!easyvr.getNextWordLabel(name))
              break;
            if (pos != idx)
              continue;
            Serial.print(F(" = "));
            Serial.println(name);
            break;
          }
        }        
      // perform some action
      }    
  }
  // else{
  //   currentWord = -1;
  // }
}

bool isVoiceSolved(){
  if(currentWord == VOICECORRECTIDX){
    return true;
  }
  return false;
}


void handleVoice(){
  if(!voiceSolved){
    checkVoiceModule();
    bool solved = isVoiceSolved();
    if(solved){
      voiceSolved = true;
      openLock(VOICEMOUTHLOCK, 2000);
    }
  }
}
//*****************************************************************************************

void setupButtons(){

  for(int i = 0; i< NUMBTNS; i++){
    btnPressed[i] = 0;
  }

  //I should make these a class
  Serial.println("setting up buttons");
  pinMode(BTNDIAL, INPUT_PULLUP);
  pinMode(BTNDIALLIGHT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BTNDIAL),[]{btnDialPressed = !digitalRead(BTNDIAL);}, CHANGE);

  pinMode(BTNRED, INPUT_PULLUP);
  pinMode(BTNREDLIGHT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BTNRED),[]{btnPressed[0] = !digitalRead(BTNRED);}, CHANGE);

  pinMode(BTNBLUE, INPUT_PULLUP);
  pinMode(BTNBLUELIGHT,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BTNBLUE),[]{btnPressed[1] = !digitalRead(BTNBLUE);}, CHANGE);


  pinMode(BTNYELLOW, INPUT_PULLUP);
  pinMode(BTNYELLOWLIGHT,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BTNYELLOW),[]{btnPressed[2] = !digitalRead(BTNYELLOW);}, CHANGE);

  pinMode(BTNGREEN, INPUT_PULLUP);
  pinMode(BTNGREENLIGHT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BTNGREEN),[]{btnPressed[3] = !digitalRead(BTNGREEN);}, CHANGE);
}


//***********SCALE FUNCTIONS********************************************************
void setupScale(){
  Serial.println("Setting up scales");
  pinMode(SCALE1POT, INPUT);
  pinMode(SCALE2POT, INPUT);
  Scale1Servo.attach(SCALE1MOTOR);
  Scale2Servo.attach(SCALE2MOTOR);
}
void runScaleServos(){
  int val1 = analogRead(SCALE1POT);
  int val2 = analogRead(SCALE2POT);

  int motor1Val = map(val1, 0, 1024, 119,170);
  int motor2Val = map(val2, 0, 1024, 119,170);
  if(motor1Val < 120){motor1Val = 90;}
  if(motor2Val < 120 ){motor2Val = 90;}

  // Serial.print("Scale 1 value: ");
  // Serial.println(val1);
  // Serial.print("Scale 2 value: ");
  // Serial.println(val2);
  // Serial.println(motor1Val);

  Scale1Servo.write(motor1Val);
  Scale2Servo.write(motor2Val);
}
void scalesOff(){
  Scale1Servo.write(90);
  Scale2Servo.write(90);
}
bool isScaleSolved(){
  int val1 = analogRead(SCALE1POT);
  int val2 = analogRead(SCALE2POT);
  int thresh = 350; //how close the scale needs to be to the corect value
  if(abs(val1 - SCALE1VALUE) < thresh){// && abs(val2 - SCALE2VALUE) < thresh){ //only checking scale one temporarily
    return true;
  }
  return false;
}

void handleScales(){
  if(!scaleSolved){
    runScaleServos();
    bool solved = isScaleSolved();
    if(solved && !scaleSolved){
      scaleSolved = true;
      Serial.println("Scale Solved");
      openLock(SCALEMOUTHLOCK, 500);
    }
  }
}
//********************************************************************
