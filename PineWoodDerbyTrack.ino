#include <SPI.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include <Adafruit_NeoPixel.h>

uint32_t millisRaceTimeOut  = (1000 * 10); // 10 seconds
uint32_t solenoidOpenPeriod =  250; //ms
uint32_t gateOpenTimeOut    = 3000; //ms

uint32_t runningTimeUpdateRate = 10; //ms (0 is off)

//       per Lane                       1,    2,    3,    4
uint8_t  laneAssignmentFinish[]   = {   1,    2,    3,    4}; // Input pins
uint8_t  laneAssignmentStart[]    = {  10,    9,    8,    7}; // Input pins
uint8_t  laneAssignmentAlpha4[]   = {0x74, 0x73, 0x72, 0x71}; // i2c addresses
uint8_t  laneAssignmentNum595[]   = {   0,    1,    2,    3}; // Large Digit Display order
uint16_t SensorFinishThreshold[]  = { 200,  200,  200,  200}; // A2D threshold 
uint16_t SensorStartThreshold[]   = { 200,  200,  200,  200}; // A2D threshold 
uint8_t  laneAssignmentNeoPixel[] = {   0,    1,    2,    3}; // pixels order
uint8_t  PlaceColorsNeoPixel[][3] = {
                                      {  0,   0, 255}, // Blue
                                      {  0, 255,   0}, // Red
                                      {255, 255, 255}, // White
                                      {255, 255,   0}, // Yellow
                                    };
uint8_t  PlaceColorsTerminal[]    = {0x36, 0x31, 0x37, 0x33}; // ASCII ForeGround Colors


#define LENGTH_OF_ARRAY(x) ((sizeof(x)/sizeof(x[0])))

// pin mapping of quad 595s driving 7 segment Lane-Place
uint8_t latch595Pin = 53; //Pin connected to ST_CP of 74HC595
uint8_t clock595Pin = 52; //Pin connected to SH_CP of 74HC595
uint8_t data595Pin  = 51; //Pin connected to DS of 74HC595
uint8_t clear595Pin = 49; //Pin connected to DS of 74HC595
uint8_t _num595[]   = {0, 0, 0, 0}; // initialized blank values

// pin mapping of Buzzer
uint8_t buzzerPin[] = {11, 12};

// pin mapping of starting gates solenoid
uint8_t startSolenoidPin = 2;

// pin mapping of starting trigger switch
uint8_t startTriggerPin = 3;

// pin mapping of starting gate sensor
uint8_t startGatePin = A6;

// pin mapping of NeoPixel per lane at starting gate
uint8_t neopixelPin = 6;

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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LENGTH_OF_ARRAY(laneAssignmentNeoPixel), neopixelPin, NEO_GRB + NEO_KHZ800);

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
  waiting_for_cars_to_finish,
  raced_finished
};

char track_states[][40] = {
  "just_booted",
  "print_waiting_for_closed_gate",
  "waiting_for_closed_gate",
  "waiting_for_cars_at_gate",
  "race_triggered",
  "waiting_for_gate_drop",
  "wait_to_turn_off_solenoid",
  "waiting_for_cars_to_finish",
  "raced_finished"
};

uint32_t microsRaceStart;
uint32_t millisRaceStart;
uint32_t millisRaceTriggered;
uint32_t millisCloseSolenoid;
uint32_t millisGateTimeOut;
uint32_t millisRaceExpire;
uint32_t millisCurrentRaceDuration;
uint32_t nextPrintCurrentTime;
uint8_t  placeOrder[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint32_t lanesTimeMs[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint32_t lanesTimeUs[LENGTH_OF_ARRAY(laneAssignmentFinish)];
uint8_t  currentPlace;
bool showRunningTime;

track_state_m track_state;
track_state_m prv_track_state;

HardwareSerial *SerialDebug;
HardwareSerial *SerialBT;

void setup() {

  SerialDebug = &Serial;
  SerialBT    = &Serial3;

  SerialDebug->begin(115200);
  SerialDebug->println("Starting Setup");

  SerialBT->begin(115200);
  SerialBT->println("Pinewood Derby Track.");
                                              
  //Initialize pins to output so you can control the quad 595 shift register
  pinMode(latch595Pin, OUTPUT);
  //pinMode(clock595Pin, OUTPUT);
  //pinMode(data595Pin, OUTPUT);
  pinMode(clear595Pin, OUTPUT);
  digitalWrite(clear595Pin, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Initialize Adafruit_LEDBackpack
  for (uint8_t channel = 0; channel < LENGTH_OF_ARRAY(laneAssignmentAlpha4); channel++) {
    alpha4[channel].begin(laneAssignmentAlpha4[channel]);  // pass in the address
  }

  // speed up I2C clock rate
  TWBR = 2;

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
  //  digitalWrite(startSolenoidPin, HIGH); // normal state is locked/LOW
  delay(500);
  digitalWrite(startSolenoidPin, LOW);

  // Initialize Buzzer bootup tone sequence
  pinMode(buzzerPin[0],OUTPUT);
  pinMode(buzzerPin[1],OUTPUT);
  tone(buzzerPin[1], 262, 1000); //NOTE_C4 for 1/4 second

  // Test cycle display each number for both 595 and LEDPackpacks
  for (uint8_t count = 0; count < LENGTH_OF_ARRAY(numeral595mapping); count++) {

    for (uint8_t channel = 0; channel < LENGTH_OF_ARRAY(laneAssignmentAlpha4); channel++) {
      _num595[laneAssignmentNum595[channel]] = count;
      alpha4[channel].writeDigitAscii(0, 0x30 + count, HIGH); // with Period
      alpha4[channel].writeDigitAscii(1, 0x30 + count);
      alpha4[channel].writeDigitAscii(2, 0x30 + count);
      alpha4[channel].writeDigitAscii(3, 0x30 + count);
      alpha4[channel].writeDisplay();
      display595();
    }
    delay(150);
    if (!digitalRead(startTriggerPin)) {
      digitalWrite(startSolenoidPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else {
      digitalWrite(startSolenoidPin, LOW);    // turn the LED off by making the voltage LOW
    }
  }
  digitalWrite(startSolenoidPin, LOW);
  colorWipe(strip.Color(255,   0,   0), 50); // Green
  colorWipe(strip.Color(  0, 255,   0), 50); // Red
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue
  colorWipe(strip.Color(255, 255, 255), 50); // White
  
  showRunningTime = true;

  SerialDebug->println("Starting Main Loop");
  track_state = just_booted;
}

void loop() {

  // Begin of track_state machine
  if (track_state == just_booted) {
    track_state = print_waiting_for_closed_gate;
  }
  else if (track_state == print_waiting_for_closed_gate) {
    if(digitalRead(startGatePin)) {
      alpha4[3].clear();

      alpha4[2].writeDigitAscii(0, 'G');
      alpha4[2].writeDigitAscii(1, 'A');
      alpha4[2].writeDigitAscii(2, 'T');
      alpha4[2].writeDigitAscii(3, 'E');

      alpha4[1].writeDigitAscii(0, 'O');
      alpha4[1].writeDigitAscii(1, 'P');
      alpha4[1].writeDigitAscii(2, 'E');
      alpha4[1].writeDigitAscii(3, 'N');

      alpha4[0].clear();

      alpha4[0].writeDisplay();
      alpha4[1].writeDisplay();
      alpha4[2].writeDisplay();
      alpha4[3].writeDisplay();

      _num595[laneAssignmentNum595[0]] = 12;
      _num595[laneAssignmentNum595[1]] = 12;
      _num595[laneAssignmentNum595[2]] = 12;
      _num595[laneAssignmentNum595[3]] = 12;
      display595();

      SerialBT->println("Gate is Open");
    }

    track_state = waiting_for_closed_gate;
  }
  else if (track_state == waiting_for_closed_gate) {
    if(digitalRead(startGatePin)) {
      delay(100);
    }
    else {
      // clear out any prior places
      for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
        placeOrder[lane] = 0;
        lanesTimeMs[lane] = 0;
        lanesTimeUs[lane] = 0;
      }
//      SerialBT->write(27); SerialBT->print("[2J"); //escape sequence clears all characters above and below the cursor location
//      SerialBT->write(27); SerialBT->print("[;H"); //sets cursor position to 0,0
      SerialBT->print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
      SerialBT->println("Track Ready.");
      currentPlace = 0;
      track_state = waiting_for_cars_at_gate;
    }
  }
  else if (track_state == waiting_for_cars_at_gate) {
    if (digitalRead(startGatePin)) { // if gate is open go back a step
      track_state = print_waiting_for_closed_gate;
    }
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentStart); lane++) {
      if ((uint16_t) analogRead(laneAssignmentStart[lane]) < SensorStartThreshold[lane]) {
        alpha4[lane].writeDigitAscii(0, ' ');
        alpha4[lane].writeDigitAscii(1, 'C');
        alpha4[lane].writeDigitAscii(2, 'a');
        alpha4[lane].writeDigitAscii(3, 'r');
        alpha4[lane].writeDisplay();

        strip.setPixelColor(laneAssignmentNeoPixel[lane], strip.Color(255, 0, 0)); // Green
        strip.show();

        carPresent[lane] = true;
      }
      else {
        alpha4[lane].writeDigitAscii(0, ' ');
        alpha4[lane].writeDigitAscii(1, ' ');
        alpha4[lane].writeDigitAscii(2, ' ');
        alpha4[lane].writeDigitAscii(3, ' ');
        alpha4[lane].writeDisplay();

        strip.setPixelColor(laneAssignmentNeoPixel[lane], strip.Color(  0, 255,   0)); // Red
        strip.show();

        carPresent[lane] = false;
      }

      if ((uint16_t) analogRead(laneAssignmentFinish[lane]) < SensorFinishThreshold[lane]) {
        _num595[laneAssignmentNum595[lane]] = 11; // "-"
        display595();
      }
      else {
        _num595[laneAssignmentNum595[lane]] = 12; // " "
        display595();
      }
    }
    if (!digitalRead(startTriggerPin)) {
      track_state = race_triggered;
    }
    delay(100);
  }
  else if (track_state == race_triggered) {
    millisRaceTriggered = millis();
    millisCloseSolenoid = (millisRaceTriggered + solenoidOpenPeriod);
    millisGateTimeOut = (millisRaceTriggered + gateOpenTimeOut);
    millisRaceExpire = (millisRaceTriggered + millisRaceTimeOut);
    currentPlace = 0;
    SerialDebug->print("millisRaceTriggered ="); SerialDebug->print(millisRaceTriggered,DEC); SerialDebug->println();
    SerialDebug->print("millisCloseSolenoid ="); SerialDebug->print(millisCloseSolenoid,DEC); SerialDebug->println();
    SerialDebug->print("millisGateTimeOut ="); SerialDebug->print(millisGateTimeOut,DEC); SerialDebug->println();
    SerialDebug->print("millisRaceExpire ="); SerialDebug->print(millisRaceExpire,DEC); SerialDebug->println();
    SerialBT->println("Race Started.");
    
    digitalWrite(startSolenoidPin, HIGH); // open solenoid

    // update display for only lanes in use.
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentStart); lane++) {
      if (carPresent[lane] == true) {
        alpha4[lane].writeDigitAscii(0, '?', HIGH); // with Period
        alpha4[lane].writeDigitAscii(1, '?');
        alpha4[lane].writeDigitAscii(2, '?');
        alpha4[lane].writeDigitAscii(3, '?');
        strip.setPixelColor(laneAssignmentNeoPixel[lane], strip.Color(153, 255,   0)); // ORANGE
        strip.show();
      }
      else {
        alpha4[lane].clear();
        strip.setPixelColor(laneAssignmentNeoPixel[lane], strip.Color(  0,   0,   0)); // OFF
        strip.show();
      }
      alpha4[lane].writeDisplay();

      _num595[laneAssignmentNum595[lane]] = 12;
      display595();
    }

    track_state = waiting_for_gate_drop;
  }
  else if (track_state == waiting_for_gate_drop) {
    if(digitalRead(startGatePin)) {
      microsRaceStart = micros();
      millisRaceStart = millis();
      SerialDebug->print("microsRaceStart ="); SerialDebug->print(microsRaceStart,DEC); SerialDebug->println();
      SerialDebug->print("millisRaceStart ="); SerialDebug->print(millisRaceStart,DEC); SerialDebug->println();

      track_state = wait_to_turn_off_solenoid;
    }
    else if ((int32_t)(millis() - millisGateTimeOut) > 0) {
      digitalWrite(startSolenoidPin, LOW);

      SerialDebug->println("Gate Jammed!");
      alpha4[3].writeDigitAscii(0, 'P');
      alpha4[3].writeDigitAscii(1, 'R');
      alpha4[3].writeDigitAscii(2, 'O');
      alpha4[3].writeDigitAscii(3, 'B');

      alpha4[2].writeDigitAscii(0, 'L');
      alpha4[2].writeDigitAscii(1, 'E');
      alpha4[2].writeDigitAscii(2, 'M');
      alpha4[2].writeDigitAscii(3, ' ');

      alpha4[1].writeDigitAscii(0, 'G');
      alpha4[1].writeDigitAscii(1, 'A');
      alpha4[1].writeDigitAscii(2, 'T');
      alpha4[1].writeDigitAscii(3, 'E');

      alpha4[0].writeDigitAscii(0, 'J');
      alpha4[0].writeDigitAscii(1, 'A');
      alpha4[0].writeDigitAscii(2, 'M');
      alpha4[0].writeDigitAscii(3, 'M');

      alpha4[0].writeDisplay();
      alpha4[1].writeDisplay();
      alpha4[2].writeDisplay();
      alpha4[3].writeDisplay();

      _num595[laneAssignmentNum595[0]] = 12;
      _num595[laneAssignmentNum595[1]] = 12;
      _num595[laneAssignmentNum595[2]] = 12;
      _num595[laneAssignmentNum595[3]] = 12;
      display595();

      SerialBT->println("Gate Jammed!");

      track_state = wait_to_turn_off_solenoid;
    }
  }
  else if (track_state == wait_to_turn_off_solenoid) {
    if ((int32_t)(millis() - millisCloseSolenoid) > 0) {
      digitalWrite(startSolenoidPin, LOW);

      nextPrintCurrentTime = (millis() - millisRaceStart) + runningTimeUpdateRate;

      track_state = waiting_for_cars_to_finish;
    }
  }
  else if (track_state == waiting_for_cars_to_finish) {
    uint8_t lanesWaiting = 0;
    uint16_t finish_sensor_value[LENGTH_OF_ARRAY(laneAssignmentFinish)];
    bool updateTime = false;

    if  ((int32_t)(millis() - millisRaceExpire) > 0) {
      track_state = raced_finished;
      SerialBT->println("Race Timed Out.");
    }

    millisCurrentRaceDuration = millis() - millisRaceStart;
    if ((millisCurrentRaceDuration > nextPrintCurrentTime) && (runningTimeUpdateRate > 0)) {
      nextPrintCurrentTime = millisCurrentRaceDuration + runningTimeUpdateRate;
      updateTime = true;
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
      uint8_t currentTimebcd[4];
      long2bcd(millis() - millisRaceStart, currentTimebcd, 4);

      // check for presense of car on in use lanes.
      for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
        if ((carPresent[lane] == true) && (lanesTimeUs[lane] == 0)) {
          if (finish_sensor_value[lane] < SensorFinishThreshold[lane]) {
            placeOrder[currentPlace++] = lane;
            lanesTimeMs[lane] = millis() - millisRaceStart;
            lanesTimeUs[lane] = micros() - microsRaceStart;

            // display place above lane
            _num595[laneAssignmentNum595[lane]] = currentPlace;
            display595();

            strip.setPixelColor(laneAssignmentNeoPixel[lane], 
              strip.Color(
                PlaceColorsNeoPixel[currentPlace - 1][0],
                PlaceColorsNeoPixel[currentPlace - 1][1],
                PlaceColorsNeoPixel[currentPlace - 1][2]
              )
            );
            strip.show();

            // display time above lane
            uint8_t bcd[4];
            long2bcd(lanesTimeMs[lane], bcd, 4);
            alpha4[lane].writeDigitAscii(0, 0x30 + bcd[3], HIGH); // with Period
            alpha4[lane].writeDigitAscii(1, 0x30 + bcd[2]);
            alpha4[lane].writeDigitAscii(2, 0x30 + bcd[1]);
            alpha4[lane].writeDigitAscii(3, 0x30 + bcd[0]);
            alpha4[lane].writeDisplay();
            SerialDebug->print("currentPlace="); SerialDebug->print(currentPlace,DEC);
            SerialDebug->print(", lanesTimeMs["); SerialDebug->print(lane,DEC);SerialDebug->print("]="); SerialDebug->print(lanesTimeMs[lane],DEC);
            SerialDebug->print(", lanesTimeUs["); SerialDebug->print(lane,DEC);SerialDebug->print("]="); SerialDebug->println(lanesTimeUs[lane],DEC);

            SerialBT->print(F("\033[01;3")); SerialBT->write(PlaceColorsTerminal[currentPlace - 1]);SerialBT->print(F("m"));
            SerialBT->print("Place="); SerialBT->print(currentPlace,DEC);
            SerialBT->print(", lanes="); SerialBT->print(lane + 1,DEC);
            SerialBT->print(", "); SerialBT->print(lanesTimeMs[lane],DEC);SerialBT->println("ms");
            SerialBT->print(F("\033[00m"));
          }
          else if ((updateTime) && (runningTimeUpdateRate > 0) && showRunningTime) {
            alpha4[lane].writeDigitAscii(0, 0x30 + currentTimebcd[3], HIGH); // with Period
            alpha4[lane].writeDigitAscii(1, 0x30 + currentTimebcd[2]);
            alpha4[lane].writeDigitAscii(2, 0x30 + currentTimebcd[1]);
            alpha4[lane].writeDigitAscii(3, 0x30 + currentTimebcd[0]);
            alpha4[lane].writeDisplay();
          }
        }
      }
    }
  }
  else if (track_state == raced_finished) {
    for (uint8_t lane = 0; lane < LENGTH_OF_ARRAY(laneAssignmentFinish); lane++) {
      if ((carPresent[lane] == true) && (lanesTimeUs[lane] == 0)) {
        _num595[laneAssignmentNum595[lane]] = 12;
        display595();
        alpha4[lane].writeDigitAscii(0, '-');
        alpha4[lane].writeDigitAscii(1, '-');
        alpha4[lane].writeDigitAscii(2, '-');
        alpha4[lane].writeDigitAscii(3, '-');
        alpha4[lane].writeDisplay();
      }
    }

    SerialBT->println("Race Completed.");

    tone(buzzerPin[1], 262, 1000); //NOTE_C4
    delay(1000);
    tone(buzzerPin[1], 262, 1000); //NOTE_C4
    track_state = waiting_for_closed_gate;
  }
  // End of track_state machine

  if (track_state != prv_track_state) {
    SerialDebug->print("New track state = ");
    SerialDebug->println(track_states[prv_track_state]);
    prv_track_state = track_state;
  }
  
  if(SerialBT->available()) {
    parse_menu(SerialBT); // get command from serial input
  }
  if(SerialDebug->available()) {
    parse_menu(SerialDebug); // get command from serial input
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
  SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[laneAssignmentNum595[0]]])));
  SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[laneAssignmentNum595[1]]])));
  SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[laneAssignmentNum595[2]]])));
  SPI.transfer(pgm_read_byte_near ( &(numeral595mapping[_num595[laneAssignmentNum595[3]]])));
  digitalWrite(latch595Pin, HIGH);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void parse_menu(HardwareSerial *serial) {
  byte key_command = serial->read();
  serial->print(F("Received command: "));
  serial->write(key_command);
  serial->println(F(" "));
  
  if(key_command == 'h') {
    serial->println(F("-----------------------------"));
    serial->println(F("Available commands..."));
    serial->println(F("-----------------------------"));
    serial->println(F("[spacebar] Start Race"));
    serial->println(F("[t] Toggle Running Time more accurate"));
    serial->println(F("[d] Enable Debug on BT"));
    serial->println(F("[D] Disable Debug on BT"));
    serial->println(F("-----------------------------"));
    serial->println(F("Enter above key:"));
    serial->println(F("-----------------------------"));
  }
  else if(key_command == ' ') {
    serial->println(F("BT Starting Race!"));
    track_state = race_triggered;
  }
  else if(key_command == 't') {
    showRunningTime = !showRunningTime;
    serial->print(F("Show Running Time = "));
    if (showRunningTime) {
      serial->println(F("ON"));
    }
    else {
      serial->println(F("OFF"));
    }
  }
  else if(key_command == 'd') {
    showRunningTime = !showRunningTime;
    serial->print(F("BT Debug ON"));
    SerialDebug = &Serial3;
  }
  else if(key_command == 'D') {
    showRunningTime = !showRunningTime;
    serial->print(F("BT Debug OFF"));
    SerialDebug = &Serial;
  }

  serial->println(track_states[prv_track_state]);
}