#ifndef PUZZLE_PINS_H
#define PUZZLE_PINS_H
//SIDE 1 is the side with the scale hands green
//SIDE 2 is for the voice recognition module in ear blue
//SIDE 3 is photoresistors in the eyes yellow
//SIDE 4 is the swapping body part side red btn



//RFID Setup********************************************************************

#define RFIDSCLK 14
#define RFIDMISO 8
#define RFIDMOSI 7
#define RFIDRESET 38

#define RFID1 33
#define RFID1IDX 1
#define RFID1ANSWER "FDCDD183"

#define RFID2 34
#define RFID2IDX 2
#define RFID2ANSWER "36B77AF7"

#define RFID3 35
#define RFID3IDX 3
#define RFID3ANSWER "6917C820"

#define RFID4 36
#define RFID4IDX 4
#define RFID4ANSWER "9934BE20"

#define RFIDNOSE 37
#define RFIDNOSEIDX 0
#define RFIDNOSEANSWER "66987CF7"
//make sure to update puzzle.ino correctids if add more rfids
#define NUMREADERS 5 ///Change this to reflect number of RFID sensors connected
//END RFID SETUP****************************************************************

//solenoid setup****************************************************************
#define SCALEMOUTHLOCK 32 
#define VOICEMOUTHLOCK 31
#define EYEMOUTHLOCK 30
#define NOSELOCK 29
#define HEADLOCKS 28 //this is actually 4 solenoids actuated by realy
#define DIALDRAWERLOCK 27
#define PRIZELOCK 26
//end solenoid setup************************************************************

//Dial Hall effect setup *******************************************************
//https://www.adafruit.com/product/158?gclid=CjwKCAiAjNjgBRAgEiwAGLlf2qn13g8NA4FpkffIYcW2ZVwQ-Q3Ns69BRBwBOoz19MAtaabW1XeOIhoCp6gQAvD_BwE
#define OUTERHALL A21 //needs 10k resistor to power on 3rd leg
#define MIDHALL A22 
#define INNERHALL A20 //also digital pin 39
//******************************************************************************

//Photoresistors ***************************************************************
#define EYE1 19 //needs 10k resistor to ground
// #define EYE2 18 //needs 10k resistor to ground
//******************************************************************************

//Voice Module *****************************************************************
#define VOICETX 10 //tx on teensy
#define VOICERX 9 //rx on teensy
#define VOICESERIAL Serial2
#define VOICECORRECTIDX 7
//http://www.veear.eu/files/archive/EasyVR%203%20User%20Manual%201.0.9.pdf
//******************************************************************************

//Scale*************************************************************************
#define SCALE1MOTOR 23 //pwm
#define SCALE2MOTOR 22 //pwm
#define SCALE1POT 21
#define SCALE2POT 20
#define SCALE1VALUE 1000
#define SCALE2VALUE 500
//*********************************************************************************

//Button Setup*********************************************************************
#define BTNDIAL 24 //white //should be wired one end to gnd and the other to the correct pin
#define BTNDIALLIGHT 25

#define BTNRED 17
#define BTNREDLIGHT 16

#define BTNBLUE 15
#define BTNBLUELIGHT 13

#define BTNGREEN 2
#define BTNGREENLIGHT 3

#define BTNYELLOW 4
#define BTNYELLOWLIGHT 5
#define NUMBTNS 4

//**********************************************************************************

#define FOGRELAY 11
#define LEDRELAY 12


//SIMON
#define MAX_LEVEL 5


#endif