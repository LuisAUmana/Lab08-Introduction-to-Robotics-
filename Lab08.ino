/* 
Drive down a hallway and identify where objects are along the wall
Alex Crotts and Luis Umana - 4/18/2022
*/

#include <Servo.h>
#include <SimpleRSLK.h>

Servo servo;

const int trigPin = 32; //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad 

int MotorSpeed = 10;
float WheelDiameter = 6.985;     // In centimeters
float PulsePerRev = 360;   // Number of encoder pulses per 1 wheel rotation
float WheelBase = 13.335;       // In centimeters

double PulsePerDegree = WheelBase/WheelDiameter;  // Number of pulses per 1 degree

void setup() {
  // Initialization
  pinMode(trigPin, OUTPUT);   // Set trigPin as an output
  pinMode(echoPin, INPUT);    // Set echoPin as an input
  servo.attach(38);
  servo.write(0);
  setupRSLK();
  resetLeftEncoderCnt();      // Reset encoder counts
  resetRightEncoderCnt();
  Serial.begin(9600);
  Serial.println("Beginning Scan");
  delay(1000);      // Delay to allow the serial monitor to settle
}

void Drive_Straight(int y) {    // This function can be called for any distance
  // Function for driving straight for X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);  // Set both motors to forward
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);       // Set both motors to the same speed
  int L_Pulse_Count = 0;      // Zero the left encoder pulse count
  int R_Pulse_Count = 0;      // Zero the right encoder pulse count

  while((L_Pulse_Count < y) || (R_Pulse_Count < y)) {
    // Run until the number of pulses reaches the value stated in the void loop
    L_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    // Drive the robot forward until it runs into something
    if(digitalRead(BP_SW_PIN_0) == 0)
      break;

    if(digitalRead(BP_SW_PIN_1) == 0)
      break;

    if(digitalRead(BP_SW_PIN_2) == 0)
      break;

    if(digitalRead(BP_SW_PIN_3) == 0)
      break;

    if(digitalRead(BP_SW_PIN_4) == 0)
      break;

    if(digitalRead(BP_SW_PIN_5) == 0)
      break;

    if((L_Pulse_Count < R_Pulse_Count)){
      // If left motor is slower than right, speed up left motor and slow down right
      setMotorSpeed(LEFT_MOTOR, ++MotorSpeed);     // Speed up the left motor by 1
      setMotorSpeed(RIGHT_MOTOR, --MotorSpeed);    // Slow down the right motor by 1
    }

    if((R_Pulse_Count < L_Pulse_Count)){
      // If right motor is slower than left, speed up right motor and slow down left
      setMotorSpeed(RIGHT_MOTOR, ++MotorSpeed);     // Speed up the right motor by 1
      setMotorSpeed(LEFT_MOTOR, --MotorSpeed);      // Slow down the left motor by 1
    }
    
    if(L_Pulse_Count >= y){
      // If the number of pulses reaches the specified value, turn off the motors
      disableMotor(LEFT_MOTOR);     // Turn off the left motor
      disableMotor(RIGHT_MOTOR);    // Turn off the right motor
      }
  }
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
}

void Rotate(int z, int L_Motor_Dir, int R_Motor_Dir) {    
  // The function can be called for any rotation direction and degree
  // Function for rotating the RSLK robot in place
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  // Set the left motor to drive in the specified direction
  setMotorDirection(LEFT_MOTOR, L_Motor_Dir);      
  // Set the right motor to drive in the specified direction
  setMotorDirection(RIGHT_MOTOR, R_Motor_Dir);      
  setMotorSpeed(BOTH_MOTORS, MotorSpeed/1.2);    // Set the motors to the same speed
  int L_CCW_Pulse_Count = 0;      // Zero the encoder count
  int R_CCW_Pulse_Count = 0;      // Zero the encoder count

    while(R_CCW_Pulse_Count < z) {
    // Run until the number of pulses read reaches the specified value
    L_CCW_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_CCW_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if(R_CCW_Pulse_Count >= z) {
      // If the number of pulses reaches the specified value, turn off the motors
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }
  }
}

long Read_Distance() {
  // This function reads the distance from the ultrasonic sensor
  byte Readings[7];   // Declare an array of readings
  int x = 0;          // Array indexed at zero
  long pulseLength;   // Length of the ultrasonic pulse
  long centimeters;   // Calculated distance
  long total = 0;     // Initially zero the total for averaging the array
  long average;       // Calculated average of the array

  // Sending the pulse to the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Calculating the distance from the pulse length
  pulseLength = pulseIn(echoPin, HIGH);
  centimeters = pulseLength / 58;

  // Set up the loop to store values in an array
  for(Readings[x]; x < 7; x++) {
    // Read from the sensor:
    Readings[x] = centimeters;
    // Add the reading to the total:
    total = total + Readings[x];
  
    // If we're at the end of the array...
    if (x >= 7) {
      // ...wrap around to the beginning:
      x = 0;
      }
  }
  
  // Calculate the average of the array:
  average = total / 7;
  // send it to the computer as ASCII digits
  Serial.print("Average Distance: ");
  Serial.println(average);
  delay(100);        // delay in between reads for stability
  return(average);
    }

void loop() {
  long RightArray[16];  //Store placement of boxes on the right side
  long LeftArray[16];   //Store placement of boxes on the left side
  long EncoderArray[16];  //Store encoder values at each stop
  long RightDistance; //Right side distances
  long LeftDistance;  //Left side distances
  int TotalPulses = 300*PulsePerDegree/(WheelDiameter*PI);  //Pulses for driving 3m
  int L_EncoderCnt;   //Left encoder value
  int R_EncoderCnt;   //Right encoder value

  for (int i = 0 ; i < 16; i++){
    EncoderArray[i] = 20*i*PulsePerRev/(WheelDiameter*PI);  //Store encoder value
    servo.write(180);   //Turn servo to the left
    LeftDistance = Read_Distance();   //Read distance
    delay(500);
    
    if (LeftDistance < 110) {
      LeftArray[i] = 1;   //If there is a box, store a 1
    }
    else {
      LeftArray[i] = 0;   //If there is no box, store a 0
    }
    
    servo.write(0);   //Turn servo to the right
    RightDistance = Read_Distance();  //Read distance
    delay(500);
    
    if (RightDistance < 110) {
      RightArray[i] = 1;    //If there is a box, store a 1
    }
    else {
      RightArray[i] = 0;    //If there is no box, store a 0
    }

    delay(500);
    //Drive forward 20 cm and begin the scan again
    Drive_Straight(20*PulsePerRev/(WheelDiameter*PI));
  }

  delay(500);
  servo.write(90);
  //After scanning the hallway, turn 185 degrees for compensation
  Rotate(185*PulsePerDegree, MOTOR_DIR_FORWARD, MOTOR_DIR_BACKWARD);
  delay(500);

  for (int j = 16; j > 0; j--) {
    //Check the left array for values of 1
    if (LeftArray[j] == 1) {   //If a value of 1 is found, drive forward
      //Drive until the pulses in the encoder array are reached
      Drive_Straight(EncoderArray[16-j]);   
      L_EncoderCnt = getEncoderLeftCnt();   //Record encoder count
      servo.write(180);   //Turn servo to face the object
      disableMotor(BOTH_MOTORS);  //Stop moving
      delay(3000);    //Wait 3 seconds
      servo.write(90);    //Turn servo back to middle
    }

    //Check the right array for values of 1
    if (RightArray[j] == 1) {  //If a value of 1 is found, drive forward
      //Drive until the pulses in the encoder array are reached
      Drive_Straight(EncoderArray[16-j]);   
      L_EncoderCnt = getEncoderLeftCnt();   //Record encoder count
      servo.write(0);   //Turn servo to face the object
      disableMotor(BOTH_MOTORS);  //Stop moving
      delay(3000);    //Wait 3 seconds
      servo.write(90);    //Turn servo back to middle
    }

    if(LeftArray[j] == 0 && RightArray[j] == 0){
      //If no box is detected, don't move
      servo.write(90);  //Keep servo in the middle
      L_EncoderCnt = getEncoderLeftCnt();   //Record encoder count
      disableMotor(BOTH_MOTORS);    //Don't move
    }
    
    if(L_EncoderCnt = TotalPulses){
      //If the encoder count reaches 3 meters, stop moving
      disableMotor(BOTH_MOTORS);
    }
  }
    //Print the left side array
    Serial.println("Left Array:");
    for (int k = 0; k < 16; k++) {
      Serial.print("[ ");
      Serial.print(LeftArray[k]);
      Serial.println(" ]");
    }
    
    //Print the right side array
    Serial.println();
    Serial.println("Right Array:");
    for (int m = 0; m < 16; m++) {
      Serial.print("[ ");
      Serial.print(RightArray[m]);
      Serial.println(" ]");
    }
  
      Serial.println();
      delay(10000);   //Wait 10 seconds before repeating
  }
