#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h>

#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

/* Color Module */
#define TCS230_s0 7 //TCS 230 led
#define TCS230_s1 8 //TCS230 led
#define TCS230_s2 12 // TCS230 led
#define TCS230_s3 13 // TCS230 led 
#define TCS230_out 4 //input from TCS230
volatile int TCS230_frequency = 0; //TCS230 frequency
volatile int TCS230_red = 0, TCS230_green = 0, TCS230_blue = 0;

/* Software reset */
void(* resetFunc) (void) = 0; 

void setup_TCS230(){
  pinMode(TCS230_s0, OUTPUT);
  pinMode(TCS230_s1, OUTPUT);
  pinMode(TCS230_s2, OUTPUT);
  pinMode(TCS230_s3, OUTPUT);
  pinMode(TCS230_out,INPUT);
  //setting frequency scaling to 20%
  digitalWrite(TCS230_s0,HIGH);
  digitalWrite(TCS230_s1,LOW);  
}

void TCS230_run(){
  //set red filtered photodiodes to be read
  digitalWrite(TCS230_s2,LOW);
  digitalWrite(TCS230_s3,LOW);
  //read output frequency
  TCS230_red = pulseIn(TCS230_out,LOW);
  //map red 
//  TCS230_red = map(TC/S2/30_red, 100, 500, 255, 0);/
  delay(100);  //check if time delay between each colour check is necessary
  
  //set green photodiodes to be read
  digitalWrite(TCS230_s2,HIGH);
  digitalWrite(TCS230_s3,HIGH);
  //read output frequency
  TCS230_green = pulseIn(TCS230_out,LOW);  
  //map green
//  TCS230_green = map(T/CS230_green,60,400,255,0);
  delay(100);
  
  //set blue filtered photodiodes to be read
  digitalWrite(TCS230_s2,LOW);
  digitalWrite(TCS230_s3,HIGH);
  //read output frequency
  TCS230_blue = pulseIn(TCS230_out,LOW);
  //map blue
//  TCS230_blue = ma/p(TCS230_blue, 85, 490, 255, 0);
  //print frequency values
  //Serial.print("red: ");
  //Serial.print(TCS230_red);
  //Serial.print(" green: ");
  //Serial.print(TCS230_green);
  //Serial.print(" blue: ");
  //Serial.println(TCS230_blue);
  //if all values are close to each other, colour is green
//  if (abs(TCS230_red - 255) < 30 && abs(TCS230_green - 255) < 30 && abs(TCS230_blue - 255) < 30){
//    //Serial.println("FINAL green");
//    sendMessage("GREEN BOI...\n"); 
//  } else if (abs(TCS230_red - 255) < 30 && abs(TCS230_green - 255) > 70 && abs(TCS230_blue - 255) > 70){
//    //Serial.println("FINAL red"); 
//    sendMessage("RED boiiiii...\n");
//  }
  if (TCS230_red < 40 && TCS230_green < 40 && TCS230_blue < 40){
    sendMessage("white boi?");
  } else if (TCS230_red > 70 && TCS230_green > 70 && TCS230_blue > 70){
    sendMessage("black boi?");
  } else {
    int min_col_val = min(TCS230_red,TCS230_green);
    if (TCS230_red == TCS230_green){
      sendMessage("red and green boi");
    }
    if (min_col_val == TCS230_red) 
      sendMessage("Red boi");
    else if (min_col_val == TCS230_green){
      sendMessage("Green boi"); 
    }
  }
  return 0;
}

typedef enum {
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4 
} TDirection;

volatile TDirection dir = STOP;

void WDT_off(void) {
  /* Global interrupt should be turned OFF here if not already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require turning off global interrupt */
}

void setupPowerSaving()
{
  // Turn off the Watchdog Timer
  WDT_off();
  // Modify PRR to shut down TWI
  PRR |= 0b10000000;
  // Modify PRR to shut down SPI
  PRR |= 0b00000100;
  // Modify ADCSRA to disable ADC,
  //ADCSRA &= 0b01111111;
  // then modify PRR to shut down ADC
  //PRR |= 0b00000001;
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  SMCR |= 0b00000000;
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= 0b00100000; //pinMode to set to output
  PORTB &= 0b11011111; //analogwrite to set to LOW
}

//Cancer code
void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  // This function puts ATmega328P’s MCU into sleep
  sleep_cpu();
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= (~PRR_TIMER0_MASK & ~PRR_TIMER1_MASK & ~PRR_TIMER2_MASK);
}


/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV      200

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          21

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin || AIN2
#define LR                  5   // Left reverse pin || AIN1
#define RF                  10  // Right forward pin|| BIN2
#define RR                  11  // Right reverse pin|| BIN1

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and Right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns; 
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns; 

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs; //CURRENTLY NOT IN USE

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// PI, for calculating turn of circumference
#define PI                  3.141592654

// Alex's length and breadth in cm
#define ALEX_LENGTH         8
#define ALEX_BREADTH        3

// Alex's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change
float AlexDiagonal = 0.0;

// Alex's turning circumference, calculated once
float AlexCirc = 0.0;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftForwardTicks, rightForwardTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 

  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
}
/* PID (not implemented)*/
volatile float ratio = 0.95;
volatile float error = 0, integral = 0;
const float Kp = -0.004, Ki = -0.0002;
volatile int leftWheelDiff = 0;
volatile int rightWheelDiff = 0; 


// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  leftWheelDiff++;
  if (dir==FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir==BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir==LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir==RIGHT) {
    leftForwardTicksTurns++;
  }
  if (leftWheelDiff == 25) {
    if (dir == FORWARD || dir == BACKWARD) {
      error = rightWheelDiff - leftWheelDiff;
      integral += error;
      ratio += (error*Kp > 0.01 ? error*Kp*0.01 : 0.01) + integral*Ki;
    }
    leftWheelDiff = rightWheelDiff = 0;
  }
  //Serial.print("LEFT: ");
  //Serial.println(leftForwardTicks);
}

void rightISR()
{
  rightWheelDiff++;
  //rightForwardTicks++;
  if (dir==FORWARD) {
    rightForwardTicks++;
  }
  else if (dir==BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir==LEFT) {
    rightForwardTicksTurns++;
  }
  else if (dir==RIGHT) {
    rightReverseTicksTurns++;
  }
  if (rightWheelDiff == 25) {
    if (dir == FORWARD || dir == BACKWARD) {
      error = rightWheelDiff - leftWheelDiff;
      integral += error;
      ratio += (error*Kp < -0.01 ? error*Kp*-0.01 : -0.01) + integral*Ki;
    }
    leftWheelDiff = rightWheelDiff = 0;
  }
}

/* iR bool */
/* true - normal stopping
 * false - engage auto mode; will cause bot to manually adjust left and right 
 *         and also move forward indefinitely ONCE (can be stopped manually or calling masterIR again)
 */
volatile bool masterIR = true;
volatile bool isHitLeft = false;
volatile bool isHitRight = false;
volatile bool isHitFront = false;
volatile bool isSent = false;

void setupADC() {
  PRR &= 0b11111110;
  ADCSRA |= 0b10001111;
  ADMUX |= 0b01000000;
}
void startADC() {
  ADCSRA |= 0b01000000;
}

ISR(ADC_vect) {
  unsigned int loval = ADCL;
  unsigned int hival = ADCH;
  unsigned int adcvalue = (hival << 8) + loval;
  //Serial.println("test");
  switch (ADMUX) {
    //right INFRA
    case 0b01000000:
      if (adcvalue < 50) isHitRight = true;
      else isHitRight = false;
      ADMUX = 0b01000001;
      break;
    //left INFRA
    case 0b01000001:
      if (adcvalue < 50) isHitLeft = true;
      else isHitLeft = false;
      ADMUX = 0b01000010;
      break;
    //front INFRA?
    case 0b01000010:
      if (adcvalue < 50) isHitFront = true;
      else isHitFront = false;
      ADMUX = 0b01000011;
      break;
    case 0b01000011:
      ADMUX = 0b01000100;
      break;
    case 0b01000100:
      ADMUX = 0b01000101;
      break;
    case 0b01000101:
      ADMUX = 0b01000000;
      break;
  }
  if (isHitFront) {
    if (dir == FORWARD) stop();
    //when stopped, automatically gets color reading
    if (!isSent) {
      TCS230_run();
      delay(100);
      sendMessage("YOU GOT HIT.... aRa aRa \n");
      isSent = true;
    }
  }
  if (isHitLeft) {
    if (dir == LEFT) stop();
    if (!masterIR) right(10,90);
    if (!isSent) {
      sendMessage("my LEFT hurts senpai.. uWu\n");
      isSent = true;
    }
  } 
  if (isHitRight) {
    if (dir == RIGHT) stop();
    if (!masterIR) left(10, 90);
    if (!isSent) {
      sendMessage("my RIGHT hurts.. YAMERO!!\n");
      isSent = true; 
    }
  }
  ADCSRA |= 0b01000000;
}



// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(57600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  
  int val = pwmVal(speed);
  int powerL = val;
  int powerR = val * 0.908;
  if (dist == 0)
    deltaDist = 999999;
  else 
    deltaDist = dist;
    
  newDist = reverseDist + deltaDist;

  analogWrite(LR, val);
  analogWrite(RR, val*0.95);
  analogWrite(LF,0);
  analogWrite(RF, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);
  int powerL = val;
  int powerR = val * 0.908;
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  if (dist == 0)
    deltaDist = 999999;
  else 
    deltaDist = dist;
  newDist = forwardDist + deltaDist;

  analogWrite(LF, val);
  analogWrite(RF, val*0.95);
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {

  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}


// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void right(float ang, float speed) //DONT USE RIGHT!!!
{
  dir = RIGHT;
  
  int val = pwmVal(speed);
  
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);
  
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  float Lratio = 1;
  if (ang == 90) {
    Lratio *= 1.25;
  }
  else if (ang == 180) {
    Lratio *= 1.28;
  }
  else if (ang == 360) {
    //Lratio *= 1.34;
  }
  else {
    Lratio *= 1;
  }

  analogWrite(RR, val);
  analogWrite(LF, (val*Lratio));
  analogWrite(RF, 0);
  analogWrite(LR, 0);
  /* 90 - scale by 1.21 for both
   * 180 - scale by 1.25 for both?
   * 360 - scale by
   */
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);
  
  targetTicks = leftReverseTicksTurns + deltaTicks;

  float Rratio = 1;
  if (ang == 90) {
    Rratio *= 0.85855;
  }
  else if (ang == 180) {
    Rratio *= 0.92555;
  }
  else if (ang == 360) {
    Rratio *= 0.98;
  }
  else {
    Rratio *= 1;
  }

  analogWrite(LR, val);
  analogWrite(RF, val*Rratio);
  analogWrite(RR, 0);
  analogWrite(LF, 0);
  /* 90 - scale by 0.86?
   * 180 - scale by 0.93
   * 360 - scale by
   */
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0; 
  leftReverseTicksTurns = 0; 
  rightReverseTicksTurns = 0; 

  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  isSent = false;
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        sendStatus();
        break;
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        sendOK();
        break;
    case COMMAND_GET_COLOR:
        sendOK();
        TCS230_run();
        break;
    case COMMAND_W:
        sendOK();
        forward((float) 10, (float) 75);
        break;
    case COMMAND_A:
        sendOK();
        left((float) 25, (float) 90);
      break;
    case COMMAND_S:
        sendOK();
        reverse((float) 10, (float) 75);
      break;
    case COMMAND_D:
        sendOK();
        right((float) 25, (float) 90);
      break;
    case COMMAND_AUTO:
    resetFunc();
    /*
        if (masterIR) {
          sendMessage("AUTO PILOT INITIATE\n");
          forward(0, 70);
          masterIR = 1 - masterIR;
        }
        else {
          sendMessage("AUTO PILOT disengaged...\n");
          stop();
          masterIR = 1 - masterIR;
        }
    */
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
 
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  /*Power Saving*/
  setupPowerSaving();
  
  /* Color Sensor */
  setup_TCS230();
  /* iR */
  setupADC();
  startADC();
  
  sei();
  //DDRC &= ~(0b00000001);
  //dir = BACKWARD;
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 // forward(0, 100);
//  delay(1000);
//  stop();
//  delay(1000);
// Uncomment the code below for Week 9 Studio 2

  // Code to stop motor once the forwardDist by encoders exceed newDist input by USER
  if (deltaDist > 0){
    if (dir == FORWARD){
      if (forwardDist >= newDist){
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } 
      if (dir == BACKWARD){
        if (reverseDist >= newDist){
          deltaDist = 0;
          newDist = 0;
          stop();
        }
      }
      if (dir == STOP){
          deltaDist = 0;
          newDist = 0;
          stop();
      }
    }
  
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
  
 // put your main code here, to run repeatedly:
 
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  } 
  else if (result == PACKET_INCOMPLETE && dir == STOP) {
    putArduinoToIdle();
  }
  
  //TCS230_run();
  //delay(1000);

  //reverse(500, 70);
}
