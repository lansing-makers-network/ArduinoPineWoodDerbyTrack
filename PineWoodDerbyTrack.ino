#include <SPI.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include "pitches.h"

#define NUMBER_OF_CHANNELS 4
#define LANE_SNS_FINISH_1 1
#define LANE_SNS_START_1  7
#define SensorThreshold 200
#define MILLISRACETIMEOUT (1000 * 10) // 10 seconds
#define SOLENOID_OPEN_PERIOD 250 //ms

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
  B01111111,  // DP (10)
  B10111111,  // -  (11)
  B11111111   // OFF(12)
}; 

Adafruit_AlphaNum4 alpha4[5] = Adafruit_AlphaNum4();
bool carPresent[5] = {false, false, false, false, false};

enum track_state_m {
  just_booted,
  print_waiting_for_closed_gate,
  waiting_for_closed_gate,
  waiting_for_cars_at_gate,
	race_triggered,
  waiting_for_gate_drop,
  wait_to_turn_off_solenoid,
  race_started,
  waiting_for_cars_to_finish,
	raced_finished,
  waiting_for_last,
  race_timed_out
  };

uint32_t microsRaceStart;
uint32_t millisRaceStart;
uint32_t millisCloseSolenoid;
uint32_t millisRaceExpire;
uint8_t placeOrder[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint8_t lanesTimeMs[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint8_t lanesTimeUs[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint8_t currentPlace;

track_state_m track_state;

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
  track_state = just_booted;
}

void loop() {


  if (track_state == just_booted) {
  	track_state = print_waiting_for_closed_gate;
  }
  else if (track_state == print_waiting_for_closed_gate) {
	  if(digitalRead(startGatePin)) { 
	    alpha4[0].clear();
	    alpha4[1].writeDigitAscii(0, 'O');
	    alpha4[1].writeDigitAscii(1, 'P');
	    alpha4[1].writeDigitAscii(2, 'E');
	    alpha4[1].writeDigitAscii(3, 'N');
	
	    alpha4[2].writeDigitAscii(0, 'G');
	    alpha4[2].writeDigitAscii(1, 'A');
	    alpha4[2].writeDigitAscii(2, 'T');
	    alpha4[2].writeDigitAscii(3, 'E');
	    alpha4[3].clear();
	    
	    alpha4[0].writeDisplay();
	    alpha4[1].writeDisplay();
	    alpha4[2].writeDisplay();
	    alpha4[3].writeDisplay();
	
    	_num595[0] = 12;
    	_num595[1] = 12;
    	_num595[2] = 12;
    	_num595[3] = 12;
  		display595();
	  }

		// clear out any prior places
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
			placeOrder[lane] = 0;
			lanesTimeMs[lane] = 0;
			lanesTimeUs[lane] = 0;			
		}
		currentPlace = 0;

	  track_state = waiting_for_closed_gate;
  }
  else if (track_state == waiting_for_closed_gate) {
	  if(digitalRead(startGatePin)) { 
	    delay(100);
	  }
	  else {
		  track_state = waiting_for_cars_at_gate;
		}
  }
  else if (track_state == waiting_for_cars_at_gate) {
	  if (digitalRead(startGatePin)) { // if gate is open go back a step
	  	track_state = print_waiting_for_closed_gate;
	  }
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentStart); lane++) {
      if ((uint16_t) analogRead(laneAssignmentStart[lane]) < SensorThreshold) {
	      alpha4[lane].writeDigitAscii(0, ' ');
	      alpha4[lane].writeDigitAscii(1, 'C');
	      alpha4[lane].writeDigitAscii(2, 'a');
	      alpha4[lane].writeDigitAscii(3, 'r');
        alpha4[lane].writeDisplay();
	    	carPresent[lane] = true; 
      }     
      else {
	      alpha4[lane].writeDigitAscii(0, 'N');
	      alpha4[lane].writeDigitAscii(1, 'o');
	      alpha4[lane].writeDigitAscii(2, 'n');
	      alpha4[lane].writeDigitAscii(3, 'e');
	      alpha4[lane].writeDisplay();
      	carPresent[lane] = false;
      }

      if ((uint16_t) analogRead(laneAssignmentFinish[lane]) < SensorThreshold) {
      	_num595[lane] = 1; // "-"
	  		display595();
      }     
      else {
      	_num595[lane] = 12; // " "
	  		display595();
      }
    }
    if (!digitalRead(startTriggerPin))
    {
		  track_state = race_triggered;
    }
    delay(100);
  }
  else if (track_state == race_triggered) {
		microsRaceStart = micros();
		millisRaceStart = millis();
		millisCloseSolenoid = (microsRaceStart + SOLENOID_OPEN_PERIOD);
		millisRaceExpire = (microsRaceStart + MILLISRACETIMEOUT);
		currentPlace = 0;
		
    digitalWrite(startSolenoidPin, HIGH); // open solenoid

  	// update display for only lanes in use.
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentStart); lane++) {
      if (carPresent[lane] == true) {
	      alpha4[lane].writeDigitAscii(0, '?', HIGH);
	      alpha4[lane].writeDigitAscii(1, '?');
	      alpha4[lane].writeDigitAscii(2, '?');
	      alpha4[lane].writeDigitAscii(3, '?');
      }     
      else {
				alpha4[lane].clear();
      }
      alpha4[lane].writeDisplay();

    	_num595[lane] = 12;
  		display595();
    }

	  track_state = waiting_for_gate_drop;
	}
  else if (track_state == waiting_for_gate_drop) {
		if(digitalRead(startGatePin)) { 
	  	track_state = wait_to_turn_off_solenoid;
	  }
  }
  else if (track_state == wait_to_turn_off_solenoid) {
		if ((int32_t)(millis() - millisCloseSolenoid) > 0) {
	    digitalWrite(startSolenoidPin, LOW);
	
		  track_state = race_started;
		  track_state = waiting_for_cars_to_finish;
		}
	}
  else if (track_state == waiting_for_cars_to_finish) {
		uint8_t lanesWaiting = 0;
		uint16_t finish_sensor_value[LENGTH_OF_ARRAY(laneAssignmentFinish)];

  	if  ((int32_t)(millis() - millisRaceExpire) > 0) {
		  track_state = raced_finished;
  	}

		// take a snap shot of in use lanes, in tight time frame.
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
      if ((carPresent[lane] == true) && (lanesTimeUs[lane] == 0)) {
	      finish_sensor_value[lane] = analogRead(laneAssignmentFinish[lane]);
	      lanesWaiting++;
	    }
    }
		
		if (lanesWaiting == 0) {
			track_state = raced_finished;
		}
		else {
			// check for presense of car on in use lanes.
	    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
	      if ((carPresent[lane] == true) && (lanesTimeUs[lane] == 0) && (finish_sensor_value[lane] < SensorThreshold)) {
					placeOrder[currentPlace++] = lane;
					lanesTimeMs[lane] = millis() - millisRaceStart;
					lanesTimeUs[lane] = micros() - microsRaceStart;
	
					// display place above lane
	      	_num595[lane] = currentPlace;
		  		display595();
	
					// display time above lane
	  		  uint8_t bcd[4];
		      long2bcd(lanesTimeMs[lane], bcd, 4);
			    alpha4[lane].writeDigitAscii(0, 0x30 + bcd[3], HIGH);
			    alpha4[lane].writeDigitAscii(1, 0x30 + bcd[2]);
			    alpha4[lane].writeDigitAscii(2, 0x30 + bcd[1]);
			    alpha4[lane].writeDigitAscii(3, 0x30 + bcd[0]);
			    alpha4[lane].writeDisplay();
				}
			}
		}

  }
  else if (track_state == raced_finished) {
	  tone(buzzerPin[1], 262, 1000); //NOTE_C4
		delay(1000);
	  tone(buzzerPin[1], 262, 1000); //NOTE_C4
  	track_state = waiting_for_closed_gate;
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