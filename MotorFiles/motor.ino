/*
SET MICROSTEP DRIVER AS:

(S1,S2,S3)=(OFF,OFF,OFF) 1/32 6400 microstep for 1 rotation
(S4,S5,S6)=(OFF,ON,OFF) current: 3.0 A  ;  peak current: 3.2 A


IN THE MOTOR FROM LEFT TO RIGHT:
  B-       B+     A-      A+
BLACK  -  GREEN  RED  -  BLUE

USED FUNCTIONS:

//---------------------SETDIR---------------------------------
void revmotor() {
  //Serial.println(My_Int);
  //Serial.println(My_TimeStamp);
  //Serial.println(My_CharArray);
  //Serial.print(My_Float, 5);  // prints to five places right of the decimal
  
  //Serial.println(setdir);
  setdir = !setdir;
}

//variables
bool setdir = HIGH; // Set Direction

//----------------------PRINCIPAL FUNCTION FOR SPINNING------------------------------
  digitalWrite(driverDIR, setdir); //dir LOW
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayTime);


//---------------------FUNCTIONS TO SET DIRECTION, SPEED AND N OF SPINS-----------------
  // Spin the stepper motor 1 revolution slowly:
  for (int i=0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);

  // Spin the stepper motor 1 revolution quickly:
  for (int i=0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);

  // Set the spinning direction clockwise:
  digitalWrite(dirPin, HIGH);

  // Spin the stepper motor 5 revolutions fast:
  for (int i=0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);

  // Spin the stepper motor 5 revolutions fast:
  for (int i=0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000);

*/

//declare variables
int reversePin = 11;  // Push button for reverse
const int dirPin = 5; //dir-
const int stepPin = 7; //pul-
const int delayTime = 100;
bool stop = false;
#define stepsPerRevolution 6400 //we are using (S1,S2,S3)=(OFF,OFF,OFF)

void startMotor(int numberRevolution, int speed, bool dir) {
  bool boolDir;

  if(dir){
    boolDir = HIGH;
  } else {
    boolDir = LOW;
  }
  //speed: LOW 20 BUT 15 WORKS AS MIN
  //10 IS 5000

  digitalWrite(dirPin, boolDir);

  // Spin the stepper motor 5 revolutions fast:
  for (int i=0; i < numberRevolution * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
  }
}

void setup() {
  Serial.begin(9600); //com speed at 9600 baudios
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  //pinMode (reversePin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(reversePin), revmotor, FALLING);
  //Serial.begin(96000);

  startMotor(5, 40, true); //speed from 20 LOW to 5000 HIGH

}
   

// the loop function runs over and over again forever
void loop() {
  
  /*while(!stop){
    startMotor(1, 40, false); //speed from 20 LOW to 5000 HIGH
  }*/
}

