// This code will provide ESC telemetry from T-Motor's Alpha series of ESC's, over serial.
// Should be uploaded to Teensy 4.1, or to a similar microcontroller.
// You should also use the motorESCAlpha.py script to interpret this data inside a companion computer 
// inside the drone, and monitor its status, in order to detect a potential motor failure.
// Please download the PRD_TMAESC library. This can be easily done inside the Arduino IDE, before
// uploading your code to the microcrontroller
// Author: Thiago Lages (github.com/thiagolages)

// libs
#include <PRDC_TMAESC.h>
#include <HardwareSerial.h>
#include <stdio.h>

// variables definition
#define DEBUG_BAUD  1000000
#define LED_PIN     13

const int numMotors = 6;
void clearInputBuffer();
void sendDataSerial();
int sendDataMotorIdx = 0;

float timeArm_ms[numMotors];
float throttleReceivePercent[numMotors];
float throttleOutPercent[numMotors];
float RPM[numMotors];
float voltage[numMotors];
float current[numMotors];
float currentPhase[numMotors];
float tempMOSFET[numMotors];
float tempCap[numMotors];
char data[numMotors][256] = {"noData1", "noData2", "noData3", "noData4", "noData5", "noData6"}; // will hold data from each motor
char c;                                                                                   // will hold temp data, will be used to empty the input buffer

bool isMotorUpdated[numMotors] = {false,false,false,false,false,false}; // indicates if we have received an update from a motor or not. Will be used to send data or not

PRDC_TMAESC esc[numMotors];

// setup function
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  for (int i = 0; i < numMotors; i++){
    esc[i].set_poles(12);
  }
  
  esc[0].init(Serial1);
  esc[1].init(Serial2);
  esc[2].init(Serial3);
  esc[3].init(Serial4);
  esc[4].init(Serial5);
  esc[5].init(Serial6);
  Serial.begin(DEBUG_BAUD);
}

//  loop function
void loop() {

// waits for data update
  for (int motorIdx = 0; motorIdx < numMotors; motorIdx++){
    if (esc[motorIdx].update()) {
      getDataFromMotor(motorIdx);
      isMotorUpdated[motorIdx] = true;
    }
    updateMotorData(motorIdx);
  }

  // only send data if we receive something in the serial port
  if(Serial.available() > 0){
    clearInputBuffer();
    sendDataSerial();
  }
}

// data reading function
void getDataFromMotor(int motorIdx) {
  
  timeArm_ms[motorIdx] = esc[motorIdx].data.time;
  
  if (esc[motorIdx].data.fault) {
    //Serial.print("Fault(s) detected! Status code: ");
    Serial.println(esc[motorIdx].data.status, BIN);
  }
  else {
   // Serial.println("No faults detected.");
  }

  throttleReceivePercent[motorIdx] = esc[motorIdx].data.throttle_rx;
  throttleOutPercent[motorIdx]     = esc[motorIdx].data.throttle_out;
  RPM[motorIdx]                    = esc[motorIdx].data.rpm; 
  voltage[motorIdx]                = esc[motorIdx].data.voltage_bus;
  current[motorIdx]                = esc[motorIdx].data.current_bus;
  currentPhase[motorIdx]           = esc[motorIdx].data.current_phase;
  tempMOSFET[motorIdx]             = esc[motorIdx].data.temperature_mos;
  tempCap[motorIdx]                = esc[motorIdx].data.temperature_cap;
   
}

void updateMotorData(int motorIdx){
   // copy all this data to data[motorIdx]  
  sprintf (data[motorIdx], "%d,%d,%.0f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f" , motorIdx, (isMotorUpdated[motorIdx] ? 1 : 0), timeArm_ms[motorIdx], throttleReceivePercent[motorIdx], throttleOutPercent[motorIdx], RPM[motorIdx], voltage[motorIdx], current[motorIdx], currentPhase[motorIdx], tempMOSFET[motorIdx], tempCap[motorIdx]); 
}

void clearInputBuffer(){
  while(Serial.available() > 0){
    c = (char)Serial.read();  
  }
}

void sendDataSerial(){
  // print data from data[motorIdx], which has been filled inside getDataFromMotor()
  Serial.println(data[sendDataMotorIdx]);
  isMotorUpdated[sendDataMotorIdx] = false;
  
  if (sendDataMotorIdx >= numMotors-1){
    sendDataMotorIdx = 0;
  }else{
    sendDataMotorIdx++;
  }
}