#include "MeMCore.h"
#define RGBWait 200
#define LDRWait 10
#define TOLERANCE 0.8 // cm

MeLightSensor lightSensor(PORT_6);
MeRGBLed led(PORT_7);
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
MeBuzzer  buzzer;
MeLineFollower lineFinder(2);


MeUltrasonicSensor ultraSensor(PORT_3);

int red = 0;
int green = 0;
int blue = 0;

float colourArray[] = {0, 0, 0};
float whiteArray[] = {800.00, 800.00, 800.00};
float blackArray[] = {200.00, 200.00, 200.00};
float greyDiff[] = {600.00, 600.00, 600.00};

double distance = 0.00;
double distanceThreshold = 4.0; // for ultrasonic sensor
uint8_t slow_speed = 200; // when adjusting with proximity sensor
uint8_t normal_speed = 250; // when moving and turning

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

double Vout_to_dist(double Vout)
{
  //NEED TO RECALIBRATE RELATION BET Vout & DISTANCE
  //DEPENDING ON ACTUAL SENSOR DATA
  return (Vout * 5 / 1024) + 0.63;
  //return (Vout*5/1024 * 0.5) + 0.9; //distance in cm (get relation for characterization graph)
}

double left_IR_V = analogRead(A1); // left IR Vout
double right_IR_V = analogRead(A0); // right IR Vout


double left_IR_dist = Vout_to_dist(left_IR_V);
double right_IR_dist = Vout_to_dist(right_IR_V);

void calibrate() {
  // for initiale colour calibration
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready

  for (int i = 0; i <= 2; i++) {
    colour(i);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(5);         //scan 5 times and return the average,
    colour(3);
    delay(RGBWait);
  }

  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready

  for (int i = 0; i <= 2; i++) {
    colour(i);
    delay(RGBWait);
    blackArray[i] = getAvgReading(5);
    colour(3);
    delay(RGBWait);
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }

  Serial.println("White: ");
  for (int i = 0; i <= 2; i++) {
    Serial.println(whiteArray[i]);
  }
  Serial.println("Black: ");
  for (int i = 0; i <= 2; i++) {
    Serial.println(blackArray[i]);
  }
}

int getAvgReading(int times) {
  int reading;
  int total = 0;

  for (int i = 0; i < times; i++) {
    reading = lightSensor.read();
    total = reading + total;
    delay(LDRWait);
  }

  return total / times;
}

char whatColour() {
  for (int c = 0; c <= 2; c++) {
    colour(c);
    delay(RGBWait);
    colourArray[c] = getAvgReading(25);
    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;
    colour(3);
    delay(RGBWait);

  }
  if (colourArray[2] < 50.00 && colourArray[1] < 50.00 && colourArray[0] < 50.00) {
    //all RGB readings are lesser than 50.0. Colour is Black
    return 'E';
  }
  else if (colourArray[1] > colourArray[0] && colourArray[1] > colourArray[2] ) {
    // Green > Red && Green > Blue. Colour is Green
    return 'G';
  }


  else if (colourArray[0] > colourArray[2] && colourArray[1] > colourArray[2] && colourArray[1] >= 120.00) {
    // Red > Blue && Green > Blue && Blue >= 120.00. Colour is Yellow
    return 'Y';
  }


  else if (colourArray[0] > colourArray[1] && colourArray[0] > colourArray[2] ) {
    //Red > Green && Red > Blue. Colour is Red
    return 'R';
  }


  else if (colourArray[2] > colourArray[0] && colourArray[1] >= colourArray[0] ) {
    // Blue > Red && Green >= Red  . Colour is Blue
    return 'B';
  }

  else if (colourArray[2] > colourArray[0] && colourArray[0] > colourArray[1] && colourArray[1] < 175.00 ) {
    // Blue > Red && Red > Green && Green < 175.00 . Colour is Purple
    return 'P';
  }
}

void colour(int x) {
  led.setColorAt(0, 0, 0, 0);
  led.show();
  if (x == 0) {
    uint8_t red  = 255;
    uint8_t green = 0;
    uint8_t blue = 0;
    led.setColorAt(0, red, green, blue);
    led.show();
  }
  else if (x == 1) {
    uint8_t red  = 0;
    uint8_t green = 255;
    uint8_t blue = 0;
    led.setColorAt(0, red, green, blue);
    led.show();
  }
  else if (x == 2) {
    uint8_t red  = 0;
    uint8_t green = 0;
    uint8_t blue = 255;
    led.setColorAt(0, red, green, blue);
    led.show();
  }
  else if (x == 3) {
    uint8_t red  = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    led.setColorAt(0, red, green, blue);
    led.show();
  }
}

void move_forward() {
  // for bot to move forward
  motor1.run(-normal_speed);
  motor2.run(normal_speed);

}

void adjust()
{
  //bot was moving initially
  move_forward();

  double left_IR_V = analogRead(A1); // left IR Vout
  double right_IR_V = analogRead(A0); // right IR Vout


  double left_IR_dist = Vout_to_dist(left_IR_V);
  double right_IR_dist = Vout_to_dist(right_IR_V);

  //readjust
  if (left_IR_dist < 4 - TOLERANCE) //bot is too close to the left
  {
    motor1.stop();
    motor2.stop();

    //bot would have moved while stopping, need to recompute distance
    left_IR_V = analogRead(A1); // left IR Vout
    right_IR_V = analogRead(A0); // right IR Vout

    left_IR_dist = Vout_to_dist(left_IR_V);
    right_IR_dist = Vout_to_dist(right_IR_V);

    while (left_IR_dist < 4 - TOLERANCE)
    {
      //turn towards right slowly
      motor1.run(-slow_speed);
      motor2.run(-slow_speed);
      //recompute distance for loop exit condition
      left_IR_V = analogRead(A1); // left IR Vout
      right_IR_V = analogRead(A0); // right IR Vout

      left_IR_dist = Vout_to_dist(left_IR_V);
      right_IR_dist = Vout_to_dist(right_IR_V);
    }
    motor1.stop();
    motor2.stop();
    motor1.run(-normal_speed);
    motor2.run(normal_speed);
  }

  else if ((right_IR_dist < 4 - TOLERANCE)  )//bot is too close to the left
  {

    motor1.stop();
    motor2.stop();

    //bot would have moved while stopping, need to recompute distance
    left_IR_V = analogRead(A1); // left IR Vout
    right_IR_V = analogRead(A0); // right IR Vout

    left_IR_dist = Vout_to_dist(left_IR_V);
    right_IR_dist = Vout_to_dist(right_IR_V);

    while (right_IR_dist < 4 - TOLERANCE)
    {
      //turn toward left slowly
      motor1.run(slow_speed);
      motor2.run(slow_speed);

      //recompute distance for loop exit condition
      left_IR_V = analogRead(A1); // left IR Vout
      right_IR_V = analogRead(A0); // right IR Vout

      left_IR_dist = Vout_to_dist(left_IR_V);
      right_IR_dist = Vout_to_dist(right_IR_V);
    }
    motor1.stop();
    motor2.stop();
    motor1.run(-normal_speed);
    motor2.run(normal_speed);
  }
  
  //bot can continue moving
}

void TurnLeft() {
  // wheels of Bot move different direction to move left
  motor1.run(normal_speed);
  motor2.run(normal_speed);
  delay(270);
  // Bot stops moving
  motor1.stop();
  motor2.stop();
  // move forward while adjusting using proximity sensor
  adjust();

}

void TurnRight() {
  // wheels of Bot move different direction to move right
  motor1.run(-normal_speed);
  motor2.run(-normal_speed);
  delay(280);
  // Bot stops moving
  motor1.stop();
  motor2.stop();
  // move forward while adjusting using proximity sensor
  adjust();

}

// u turn clockwise
void reverse1() {
  motor1.run(-normal_speed);
  motor2.run(-normal_speed);
  delay(540);
  motor1.stop();
  motor2.stop();
  adjust();

}

// u turn anti-clockwise
void reverse2() {
  motor1.run(normal_speed);
  motor2.run(normal_speed);
  delay(450);
  motor1.stop();
  motor2.stop();
  adjust();

}

void TurnLeftLeft() {
  // Turn left, move forward 1 grid, turn left again
  TurnLeft();
  delay(550);
  TurnLeft();

}

void TurnRightRight() {
  // Turn right, move forward 1 grid, turn right again
  TurnRight();
  delay(600);
  TurnRight();
}


void TurnEnd() {
  // Stop moving and play victory tune. ENd of maze

  motor1.stop();
  motor2.stop();
  tune();
  delay(10000);


}

void turn() {

  // Check for colour
  char x = whatColour();
  Serial.println(x);

  if ( x == 'R') {
    // colour detected is Red
    TurnLeft();
  }

  else if ( x == 'G') {
    // colour detected is Green
    TurnRight();
  }

  else if ( x == 'Y') {
    // colour detected is Yellow

    left_IR_V = analogRead(A1); // left IR Vout
    right_IR_V = analogRead(A0); // right IR Vout

    left_IR_dist = Vout_to_dist(left_IR_V);
    right_IR_dist = Vout_to_dist(right_IR_V);

    if (right_IR_dist < left_IR_dist) {
      // bot closer to right side of path
      reverse2();
    }

    else {
      // bot closer to left side of path
      reverse1();
    }
  }

  else if ( x == 'P') {
    // colour detected is Purple
    TurnLeftLeft();

  }

  else if ( x == 'B') {
    // colour detected is Blue
    TurnRightRight();
  }

  else if ( x == 'E') {
    // colour detected is Black
    TurnEnd();

  }

}

int is_black_line() {
  // Check for black strip
  int sensorState = lineFinder.readSensors();
  if (sensorState == S1_IN_S2_IN || sensorState == S1_IN_S2_OUT || sensorState == S1_OUT_S2_IN) {
    return 1;
  }
  else {
    return 0;
  }
}

void Stop_or_Move() {
  distance = ultraSensor.distanceCm();
  if (distance <= distanceThreshold) {
    // Wall infront. Stop moving and do colour challenge
    motor1.stop();
    motor2.stop();
    turn();

  }

  if (is_black_line() == 1) {
    // Black strip detected. Stop moving and do colour challenge
    motor1.stop();
    motor2.stop();
    turn();
  }

  else {
    // Move forward while adjusting using proximity sensor
    adjust();
  }

}

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void tune() {
  buzzer.tone(800, 1000);
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    buzzer.tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    buzzer.noTone(8);
  }

}

void setup() {
  Serial.begin(9600);
  move_forward();
}

void loop() {
  // constantly check for distance of an obstacle in front of mBot
  distance = ultraSensor.distanceCm();
  Stop_or_Move();
}
