//#####################################
// SET THESE CONSTANTS TO MATCH
// THE VALUES FOR YOUR ROBOT
//#####################################

// the stop point for the left servo

const byte LEFT_STOP = 96;

// the stop point for the right servo
const byte RIGHT_STOP = 95;

// the values to add to / subtract from the stop point to make the robot drive straight
const byte LEFT_OFFSET = 5;
const byte RIGHT_OFFSET = 5;


//the threshold between light, dark and grey for each of the LDRs
//calibrated at beginning of setup func
unsigned int LEFT_LDR_WHITE;
unsigned int MIDDLE_LDR_WHITE;
unsigned int RIGHT_LDR_WHITE;

unsigned int LEFT_LDR_BLACK;
unsigned int MIDDLE_LDR_BLACK;
unsigned int RIGHT_LDR_BLACK;

unsigned int LEFT_LDR_MEAN;
unsigned int MIDDLE_LDR_MEAN;
unsigned int RIGHT_LDR_MEAN;


// OTHER GLOBAL VARIABLES

/* to use as a bar counter */

int barDetect = 0;

/* to set times each time a strip is detected and when leaving a strip */

unsigned long currentTime[6] = {0, 0, 0, 0, 0, 0};

/* to set the width of the bar and gap between */

unsigned long barWidth = 0;
unsigned long barGap[2];

/* how much to wait in ms before continue line following */

int x = 2000;

/* value to be used as a buffer for LDRs */

int BUFFER = 100;

//INCLUDES
#include <EEPROM.h>
#include <Servo.h>
Servo leftServo;
Servo rightServo;

//PIN SETUPS

#define PB1 4
#define GREEN 7
#define YELLOW 12
#define RED 13
#define rightLDR A0
#define middleLDR A1
#define leftLDR A2
#define IR_Transmitter 3



//DEBUGGING

//#define CONFIRM
//#define pushButtonTest
//#define testForNoMotion
//#define testLEDs
//#define testSetSpeeds
//#define testDistanceFunc
//#define testTurn90
//#define testCheckReflection
//#define obstacleAvoidanceNoLine
//#define testLDRs
//#define testLDRs2


//FUNCTION TO SET SPEEDS

void setSpeeds (float servoL, float servoR) {
  leftServo.write(LEFT_STOP + servoL);
  rightServo.write(RIGHT_STOP - servoR);
}

//DEBOUNCING BUTTONS

void waitKey(int pin) {
  while (digitalRead(pin) == HIGH) {} /* wait for pin to be HIGH (pressed) */
  delay(20);/* delay 20 milliseconds */
  while (digitalRead(pin) == LOW) {}/* wait for pin to be LOW (not pressed) */
  delay(20);/* delay 20 milliseconds */
}

//FUNCTION FOR DRIVING A DISTANCE STRAIGHT

void distance(float x) {
  if (x < 0) {
    setSpeeds(-20, -20); /* reverse */
  }
  else {
    setSpeeds(20, 20); /* fowards */
  }
  delay(abs(x) * 104.5); /*how long it will take to move the distance*/
  setSpeeds(0, 0);
}

//TURNING FUNCTIONS
/* int y is an angle in degrees */

void turnR( int y) {
  setSpeeds(20, -20);
  delay(y * (2000 / 90)); /* how long  to wait before turning stops*/
  setSpeeds(0, 0);
}

void turnL( int y) {
  setSpeeds(-20, 20);
  delay(y * (2000 / 90));
  setSpeeds(0, 0);
}

//funtion to set state of all 3 LEDs at once
void setLEDs(int greenState, int yellowState, int redState) {

  if (greenState == 0) {
    digitalWrite(GREEN, LOW);
  }
  else {
    digitalWrite(GREEN, HIGH);
  }

  if (yellowState == 0) {
    digitalWrite(YELLOW, LOW);
  }
  else {
    digitalWrite(YELLOW, HIGH);
  }

  if (redState == 0) {
    digitalWrite(RED, LOW);
  }
  else {
    digitalWrite(RED, HIGH);
  }
}

//FUNCTION TO TEST LDRs
void testLDRs() {
  Serial.print("The left LDR is: ");
  Serial.println(analogRead(leftLDR));
  Serial.print("The middle LDR is: ");
  Serial.println(analogRead(middleLDR));
  Serial.print("The right LDR is: ");
  Serial.println(analogRead(rightLDR));
  delay(2500);
}

//FUNCTION TO CALIBRATE LDRs

void calibrateLDRs() {

 /* To set the values of the white area */
  waitKey(PB1);
  LEFT_LDR_WHITE = analogRead(leftLDR);
  MIDDLE_LDR_WHITE = analogRead(middleLDR);
  RIGHT_LDR_WHITE = analogRead(rightLDR);

/* To set the values of the black area */
  waitKey(PB1);
  LEFT_LDR_BLACK = analogRead(leftLDR);
  MIDDLE_LDR_BLACK = analogRead(middleLDR);
  RIGHT_LDR_BLACK = analogRead(rightLDR);

/* To set the values of the grey area */
  LEFT_LDR_MEAN = ( LEFT_LDR_WHITE + LEFT_LDR_BLACK ) / 2;
  MIDDLE_LDR_MEAN = ( MIDDLE_LDR_WHITE + MIDDLE_LDR_BLACK ) / 2;
  RIGHT_LDR_MEAN =  ( RIGHT_LDR_WHITE + RIGHT_LDR_BLACK ) / 2;

}

//FUNC FOR TESTING LDR CAN DISTINGUISH LIGHT AND DARK
void lightDark() {

  /* LEFT LDR CHECK */
  if (analogRead(leftLDR) <= (LEFT_LDR_BLACK + BUFFER)) {
    Serial.println("left ldr is BLACK");
  }

  else if ((analogRead(leftLDR) <= (LEFT_LDR_MEAN + BUFFER)) && (analogRead(leftLDR) >= (LEFT_LDR_BLACK - BUFFER))) {
    Serial.println("left ldr is GREY");
  }

   else if (analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) {
    Serial.println("left ldr is WHITE");
  }

  /* MIDDLE LDR CHECK */
   if (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) {
    Serial.println("middle ldr is BLACK");
  }

  else if ((analogRead(middleLDR) <= (MIDDLE_LDR_MEAN + BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_BLACK - BUFFER))) {
    Serial.println("middle ldr is GREY");
  }

   else if (analogRead(middleLDR) >= (MIDDLE_LDR_WHITE - BUFFER)) {
    Serial.println("middle ldr is GREY");
  }

  /* RIGHT LDR CHECK */
   if (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER)) {
    Serial.println("right ldr is BLACK");
  }

  else if ((analogRead(rightLDR) <= (RIGHT_LDR_MEAN + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_BLACK - BUFFER))) {
    Serial.println("right ldr is GREY");
  }

   else if (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER)) {
    Serial.println("right ldr is GREY");
  }

  delay(3500);
}


//FUNCTION FOR CHECKING REFLECTION

bool checkReflection() {
  tone(3, 38000);
  delay(750);
  if (digitalRead(2) == LOW) /* reading IR return */ {
    Serial.println("Obstacle detected");
    return true;
  }
  else {
    Serial.println("No obstacle detected");
    return false;
  }
  noTone(3);
}

//AVOIDING OBSTACLE FUNCTION

void obstacleAvoidance() {

  int dist = 10; //buffer for area not detected when robot cannot pass
  if (checkReflection() == true) {
    while (checkReflection() == true) { /* loop until it passes front of object */
      Serial.print("obstacle detected");
      setSpeeds(0, 0);
      delay(500);
      turnR(90);
      distance(10);
      dist += 10;
      turnL(90);
    }
    turnR(90);
    distance(10); /* buffer in action here to make sure object cleared */
    turnL(90);
    distance(10);
    turnL(90);


    while (checkReflection() == true) { /* loop until side of object is passed */
      setSpeeds(0, 0);
      turnR(90);
      distance(10);
      turnL(90);
    }

    turnR(90);
    distance(10); /* buffer in action here to make sure object is cleared */
    turnL(90);
    distance(dist); /* same distance round the back as the front to get back to line following */
    turnR(90);

  }

}








void setup() {
  Serial.begin(9600);
  leftServo.attach(6);
  rightServo.attach(5);
  setSpeeds(0, 0);
  pinMode(PB1, INPUT);
  pinMode(rightLDR, INPUT);
  pinMode(middleLDR, INPUT);
  pinMode(leftLDR, INPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(IR_Transmitter, OUTPUT);
  calibrateLDRs();
  waitKey(PB1);

  storeBValue(0, LEFT_STOP, "left servo stopping point");
  storeBValue(1, LEFT_OFFSET, "left servo difference from LEFT_STOP to drive straight (when the right value is also set on the right servo");
  storeBValue(2, RIGHT_STOP, "right servo stopping point");
  storeBValue(3, RIGHT_OFFSET, "right servo difference from RIGHT_STOP to drive straight (when the left value is also set on the left servo");

  storeUIValue(4, LEFT_LDR_WHITE, "left LDR value on white");
  storeUIValue(6, LEFT_LDR_BLACK, "left LDR value on black");


  storeUIValue(8, MIDDLE_LDR_WHITE, "centre LDR value on white");
  storeUIValue(10, MIDDLE_LDR_BLACK, "centre LDR value on black");


  storeUIValue(12, RIGHT_LDR_WHITE, "centre LDR value on white");
  storeUIValue(14, RIGHT_LDR_BLACK, "centre LDR value on black");



#ifdef CONFIRM
  // now print out the values as a test
  Serial.println(readBValue(0));
  Serial.println(readBValue(1));
  Serial.println(readBValue(2));
  Serial.println(readBValue(3));

  Serial.println(readUIValue(4));
  Serial.println(readUIValue(6));
  Serial.println(readUIValue(8));
  Serial.println(readUIValue(10));
  Serial.println(readUIValue(12));
  Serial.println(readUIValue(14));
#endif

}

void loop() {

  //DEBUGGING TO TEST PUSHBUTTON

#ifdef pushButtonTest

  if (PB1 == HIGH) {
    serial.println("Push button 1 is being pressed ");
  }
  else {
    serial.println("No input");
  }

#endif

  //DEBUGGING TO FIND THE SERVO SPEED THEY STOP MOVING

#ifdef testForNoMotion

  servoSpeed = 75;

  while (servoSpeed < 105) {

    servoSpeed ++;
    rightServo.write(servoSpeed);
    leftServo.write(servoSpeed);
    Serial.print("The servo speed is ");
    Serial.println(servoSpeed);
    delay(1500);
  }

#endif

  //DEBUGGING TO TEST SET SPEEDS FUNCTION

#ifdef testSetSpeeds

  setSpeeds(10, 10);
  serial.println("Servo speeds are at 10");
  delay(2000);
  setSpeeds(0, 0);
  serial.println("Servo speeds are at 0");
  delay(1000);
  setSpeeds(20, 20);
  serial.println("Servo speeds are at 20");
  delay(2000);
  setSpeeds(0, 0);
  serial.println("Servo speeds are at 0");
  delay(1000);
  setSpeeds(40, 40);
  serial.println("Servo speeds are at 40");
  delay(2000);

#endif

  //DEBUGGING TO TEST DISTANCE FUNCTION

#ifdef testDistanceFunc

  distance(20);
  delay(500);
  distance(-20);

#endif



  //DEBUGGING TEST TO SEE IF ROBOT TURNS 90 DEGREES IN BOTH DIRECTIONS

#ifdef testTurn90

  waitKey(PB1);
  turnR(90);
  delay(500);
  turnL(90);

#endif

  //DEBUGGING TEST TO SEE IF IR_TRANSMITTER WORKS

#ifdef testCheckReflection

  checkReflection();

#endif

  // DEBUGGING TO TEST OBSTACLE AVOIDANCE WITHOUT LINE FOLLOWING

#ifdef obstacleAvoidanceNoLine

  setSpeeds(40, 40);
  obstacleAvoidance();

#endif

  //DEBUGGING TO TEST IF LEDs WORK

#ifdef testLEDs

  setLeds(0, 0, 1);
  delay(500);
  setLeds(0, 1, 0);
  delay(500);
  setLeds(1, 0, 0);
  delay(500);
  setLeds(0, 0, 1);
  delay(500);
  setLeds(0, 1, 1);
  delay(500);
  setLeds(1, 1, 1);
  delay(500);
  setLeds(0, 0, 0);
  delay(500);

#endif

  //DEBUGGING TO CHECK LDRs WORK

#ifdef testLDRs

  testLDRs();

#endif

  //DEBUGGING TO TEST LDRs CAN DISTINGUISH SHADES

#ifdef testLDRs2

  lightDark();

#endif


  //DRIVING STRAIGHT BLACK STRIP WHITE BACKGROUND

  /* LEFT LDR WHITE, MIDDLE LDR BLACK, RIGHT LDR WHITE */

  if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(40, 40);
    if ((currentTime[0] > 0) && (barDetect < 1)) {
      currentTime[1] = millis();
      barWidth = currentTime[1] - currentTime[0]; /* working out length of bar */
      barDetect = 1;
    }


    else if ((currentTime[2] > 0) && (barDetect < 2)) {
      barDetect = 2;
      /* TURN LEFT BARCODE*/
      if (barGap[0] > (barWidth * 2)) {
        setSpeeds(10, 20);
        setLEDs(0, 0, 1); /* turn on red LED while turning left */
        delay(x);
        barGap[0] = 0;
        setLEDs(0, 0, 0); /* turn off red LED */
      }


      else if ((barGap[0] < (barWidth * 2)) && (barGap[0] > 0)) {

        barDetect = 2;

        /* how much to wait before deciding if there is another strip or not */
        unsigned long fixedTime = millis() + (barWidth * 3);

        while (millis() < fixedTime) {
          /* continue counting bars at this time */

          /* TURN AROUND BARCODE */
          if (barDetect == 3) {
            delay(barWidth);
            currentTime[5] = millis();
            setSpeeds(0, 0);
            setLEDs(0, 1, 0); /* turn on yellow LED while turning around */
            turnR(180);
            setSpeeds(20, 20);
            delay(currentTime[5] - currentTime[0]); /* pass the whole barcode on return journey */
            delay(barWidth); /* extra barWidth for safety */
            setLEDs(0, 0, 0); /* turn off yellow LED */
          }

        }
        /* stop counting bars */

        /* TURN RIGHT BARCODE*/
        setSpeeds(20, 10);
        setLEDs(1, 0, 0); /* turn on green LED while turning left */
        delay(x);
        barGap[0] = 0;
        setLEDs(0, 0, 0); /* turn off green LED */
      }
    }
  }


  /* BAR DETECTED */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_BLACK + BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER))) {
    setSpeeds(40, 40);
    if ((barDetect == 0) && (currentTime[0] < 1)) {

      currentTime[0] = millis();
    }
    else if (barDetect == 1) {
      currentTime[2] = millis();
      barGap[0] = currentTime[2] - currentTime[1]; /* to workout length of gap in time at a certain speed */
      delay(barWidth);
      currentTime[3] = millis();

    }
    else if (barDetect == 2) {
      currentTime[4] = millis();
      barGap[1] = currentTime[4] - currentTime[3]; /* to workout length of 2nd gap */
      delay(barWidth);
      barDetect = 3;
    }
  }

  //DRIVING STRAIGHT BLACK STRIP GREY BACKGROUND

  else  if ((analogRead(leftLDR) >= (LEFT_LDR_MEAN - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_MEAN - BUFFER))) {
    setSpeeds(40, 40);
  }

  //DRIVING STRAIGHT GREY STRIP WHITE BACKGROUND

  else  if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_MEAN + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(40, 40);
  }


  //RECENTERING FOR BLACK STRIP WHITE BACKGROUND

  /* LEFT LDR WHITE, MIDDLE LDR WHITE, RIGHT LDR BLACK */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_WHITE - BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER))) {
    setSpeeds(40, 0);
  }

  /* LEFT LDR WHITE, MIDDLE LDR BLACK, RIGHT LDR BLACK */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER))) {
    setSpeeds(40, 0);
  }

  /* LEFT LDR BLACK , MIDDLE LDR WHITE , RIGHT LDR WHITE */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_BLACK + BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_WHITE - BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(0, 40);
  }

  /* LEFT LDR BLACK , MIDDLE LDR BLACK , RIGHT LDR WHITE */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_BLACK + BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(0, 40);

  }

  //RECENTERING FOR BLACK STRIP GREY BACKGROUND

  /* LEFT LDR GREY, MIDDLE LDR GREY, RIGHT LDR BLACK */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_MEAN - BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_MEAN - BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER))) {
    setSpeeds(40, 0);
  }

  /* LEFT LDR GREY, MIDDLE LDR BLACK, RIGHT LDR GREY */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_MEAN - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_BLACK + BUFFER))) {
    setSpeeds(40, 0);
  }


  /* LEFT LDR BLACK, MIDDLE LDR GREY, RIGHT GREY */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_BLACK + BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_MEAN - BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_MEAN - BUFFER))) {
    setSpeeds(0, 40);
  }

  /* LEFT LDR GREY, MIDDLE LDR BLACK, RIGHT LDR BLACK */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_MEAN - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_BLACK + BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_MEAN + BUFFER))) {
    setSpeeds(0, 40);

  }

  //RECENTERING FOR GREY STRIP WHITE BACKGROUND

  /* LEFT LDR WHITE, MIDDLE LDR WHITE, RIGHT LDR GREY */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_WHITE - BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_MEAN + BUFFER))) {
    setSpeeds(40, 0);
  }

  /* LEFT LDR WHITE , MIDDLE LDR GREY, RIGHT LDR GREY */
  else if ((analogRead(leftLDR) >= (LEFT_LDR_WHITE - BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_MEAN + BUFFER)) && (analogRead(rightLDR) <= (RIGHT_LDR_MEAN + BUFFER))) {
    setSpeeds(40, 0);
  }

  /* LEFT LDR GREY , MIDDLE LDR WHITE , RIGHT LDR WHITE */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_MEAN + BUFFER)) && (analogRead(middleLDR) >= (MIDDLE_LDR_WHITE - BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(0, 40);
  }

  /* LEFT LDR GREY, MIDDLE LDR GREY, RIGHT LDR WHITE */
  else if ((analogRead(leftLDR) <= (LEFT_LDR_MEAN + BUFFER)) && (analogRead(middleLDR) <= (MIDDLE_LDR_MEAN + BUFFER)) && (analogRead(rightLDR) >= (RIGHT_LDR_WHITE - BUFFER))) {
    setSpeeds(0, 40);

  }



 obstacleAvoidance();

}
#
// storeValue sets a single byte in the EEPROM.  Do not edit this function
void storeBValue(int eepromAddress, byte value, char * description) {
  EEPROM.update(eepromAddress, value);
  Serial.print("Storing value ");
  Serial.print(value);
  Serial.print(" to address ");
  Serial.print(eepromAddress);
  Serial.print(" to save the ");
  Serial.println(description);
}
// storeValue sets a single byte in the EEPROM.  Do not edit this function
void storeUIValue(int eepromAddress, unsigned int value, char * description) {
  EEPROM.put(eepromAddress, value);
  Serial.print("Storing value ");
  Serial.print(value);
  Serial.print(" to address ");
  Serial.print(eepromAddress);
  Serial.print(" to save the ");
  Serial.println(description);
}


byte readBValue(int eepromAddress) {
  return EEPROM[eepromAddress];
}

unsigned int readUIValue(int eepromAddress) {
  unsigned int uiVal;
  EEPROM.get(eepromAddress, uiVal);

  return uiVal;
}
