#include <SPI.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include "pitches.h"

#define NUMBER_OF_CHANNELS 4
#define LANE_SNS_FINISH_1 1
#define LANE_SNS_START_1  7
uint16_t SensorThreshold = 200;

uint8_t laneAssignmentFinish[] = {1, 2, 3, 4};
uint8_t laneAssignmentStart[] = {10, 9, 8, 7};
uint8_t laneAssignmentAlpha4[] = {0x74, 0x73, 0x72, 0x71};

#define LENGTH_OF_ARRAY(x) ((sizeof(x)/sizeof(x[0])))

// pin mapping of quad 595s driving 7 segment Lane-Place
uint8_t latch595Pin = 53; //Pin connected to ST_CP of 74HC595
uint8_t clock595Pin = 52; //Pin connected to SH_CP of 74HC595
uint8_t data595Pin = 51; //Pin connected to DS of 74HC595
uint8_t clear595Pin = 49; //Pin connected to DS of 74HC595
uint8_t _num595[] = {0, 0, 0, 0};

// pin mapping of Buzzer
uint8_t buzzerPin[] = {11, 12};

// pin mapping of starting gates solenoid
uint8_t startSolenoidPin = 2;

// pin mapping of starting trigger switch
uint8_t startTriggerPin = 3;

// pin mapping of starting gate sensor
uint8_t startGatePin = A6;

const uint8_t  numeral595mapping[] PROGMEM = {
  B11000000,  // 0
  B11111001,  // 1
  B10100100,  // 2
  B10110000,  // 3
  B10011001,  // 4
  B10010010,  // 5
  B10000010,  // 6
  B11111000,  // 7
  B10000000,  // 8
  B10011000,  // 9
  B01111111,  // DP
  B10111111   // -
}; 

Adafruit_AlphaNum4 alpha4[5] = Adafruit_AlphaNum4();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Setup");
  
  //Initialize pins to output so you can control the quad 595 shift register
  pinMode(latch595Pin, OUTPUT);
  //pinMode(clock595Pin, OUTPUT);
  //pinMode(data595Pin, OUTPUT);
  pinMode(clear595Pin, OUTPUT);
  digitalWrite(clear595Pin, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Initialize Adafruit_LEDBackpack
  for (uint8_t channel = 0; channel < LENGTH_OF_ARRAY(laneAssignmentAlpha4); channel++) {
    alpha4[channel].begin(laneAssignmentAlpha4[channel]);  // pass in the address
  }

  // Pre-Operating Self Test Sequences

  // Initialize Starting Trigger input.
  pinMode(startTriggerPin, INPUT);   //note INPUT_PULLUP does not always work
  digitalWrite(startTriggerPin, HIGH); // internal pull up enabled. 

  // Initialize Starting Gate Sensor.
  pinMode(startGatePin, INPUT);   //note INPUT_PULLUP does not always work
  digitalWrite(startGatePin, HIGH); // internal pull up enabled. 

  // Initialize Solenoid driver.
  digitalWrite(startSolenoidPin, LOW); // normal state is locked/LOW
  pinMode(startSolenoidPin, OUTPUT);
  
  // Test Solenoid
  digitalWrite(startSolenoidPin, HIGH); // normal state is locked/LOW
  delay(500);
  digitalWrite(startSolenoidPin, LOW);

  // Initialize Buzzer bootup tone sequence
  pinMode(buzzerPin[0],OUTPUT);
  pinMode(buzzerPin[1],OUTPUT);
  tone(buzzerPin[1], 262, 1000); //NOTE_C4 for 1/4 second

  // Test cycle display each number for both 595 and LEDPackpacks
  for (uint8_t count = 0; count < LENGTH_OF_ARRAY(numeral595mapping); count++) {
	
    for (uint8_t channel = 0; channel < LENGTH_OF_ARRAY(laneAssignmentAlpha4); channel++) {
			_num595[channel] = count;
      alpha4[channel].writeDigitAscii(0, 0x30 + count, HIGH);
      alpha4[channel].writeDigitAscii(1, 0x30 + count);
      alpha4[channel].writeDigitAscii(2, 0x30 + count);
      alpha4[channel].writeDigitAscii(3, 0x30 + count);
      alpha4[channel].writeDisplay();
			display595();
    }
    delay(100);
    if (!digitalRead(startTriggerPin))
    {
      digitalWrite(startSolenoidPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else {
      digitalWrite(startSolenoidPin, LOW);    // turn the LED off by making the voltage LOW
    }    
  }
  digitalWrite(startSolenoidPin, LOW);

//  int count = 9999;
//  uint8_t bcd[4];
//  // convert to BCD send to MAX7219
//  for (int i=0; i<=count; i++)   {
//    long2bcd(i, bcd, 4);
//    Serial.print(bcd[3], HEX); Serial.print(" ");
//    Serial.print(bcd[2], HEX); Serial.print(" ");
//    Serial.print(bcd[1], HEX); Serial.print(" ");
//    Serial.print(bcd[0], HEX); Serial.println(" ");
//
//    for (uint8_t channel = 1; channel < 6; channel++) {
//      alpha4[channel].writeDigitAscii(0, 0x30 + bcd[3], HIGH);
//      alpha4[channel].writeDigitAscii(1, 0x30 + bcd[2]);
//      alpha4[channel].writeDigitAscii(2, 0x30 + bcd[1]);
//      alpha4[channel].writeDigitAscii(3, 0x30 + bcd[0]);
//      alpha4[channel].writeDisplay();
//    }
//    delay(1);
//  } // end for  

  Serial.println("Starting Main Loop");

}

//#define NUMBER_OF_CHANNELS 4
//#define LANE_SNS_FINISH_1 1
//#define LANE_SNS_START_1  7

void loop() {
  bool laneStatus[5] = {false, false, false, false, false};

  // wait for starting Gate to be closed
  while(!digitalRead(startGatePin)) { 
    
    for (uint8_t sensorPin = 0; sensorPin < LENGTH_OF_ARRAY(laneAssignmentStart); sensorPin++) {
      uint16_t finishValue = analogRead(laneAssignmentFinish[sensorPin]);
      uint16_t startValue = analogRead(laneAssignmentStart[sensorPin]);
      if (startValue < SensorThreshold) {
      	laneStatus[sensorPin] = true; 
      	_num595[sensorPin] = true;
	      alpha4[sensorPin].writeDigitAscii(0, ' ');
	      alpha4[sensorPin].writeDigitAscii(1, 'C');
	      alpha4[sensorPin].writeDigitAscii(2, 'a');
	      alpha4[sensorPin].writeDigitAscii(3, 'r');
      }     
      else {
      	laneStatus[sensorPin] = false;
      	_num595[sensorPin] = false;
	      alpha4[sensorPin].writeDigitAscii(0, 'N');
	      alpha4[sensorPin].writeDigitAscii(1, 'o');
	      alpha4[sensorPin].writeDigitAscii(2, 'n');
	      alpha4[sensorPin].writeDigitAscii(3, 'e');
      }
      alpha4[sensorPin].writeDisplay();
  		display595();
    }
    delay(125);
  }

    if (!digitalRead(startGatePin))
    {
      digitalWrite(startSolenoidPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else {
      digitalWrite(startSolenoidPin, LOW);    // turn the LED off by making the voltage LOW
    }    


}

// return an array of BCD digits from a long 
uint8_t* long2bcd(long number, uint8_t* bcd, uint8_t numofDigitsofBCD) {
  for (uint8_t digitPosition = 0; digitPosition < numofDigitsofBCD; digitPosition++) {
    bcd[digitPosition] = number % 10;
    number = number / 10; 
  }
  return bcd;
}

void write595mapping (uint8_t num0, uint8_t num1, uint8_t num2, uint8_t num3) {
  digitalWrite(latch595Pin, LOW);
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[num0])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[num1])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[num2])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[num3])));
  digitalWrite(latch595Pin, HIGH); 
}

void display595 () {
  digitalWrite(latch595Pin, LOW);
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[0]])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[1]])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[2]])));
    SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[3]])));
  digitalWrite(latch595Pin, HIGH); 
}