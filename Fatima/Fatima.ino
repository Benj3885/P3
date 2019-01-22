#include "math.h"
#include "EMGxBee.h"
#include "CrustCrawler.h"
#include "Trajectory.h"

double timer = 0;

///////////////// For controlSystem /////////////////
bool firstMoveRound = true; // Boolean for controlling first call of controlSystem function
const unsigned int samplerate = 100; //ms
const double sampleperiod = samplerate * 0.001; // Conversion from ms to s
float vel[4] = {0.025, 0.025, 0.133, 0.05}; //Velocity in m/s; 
float velMultiplier = 1;
volatile int8_t dir = 1; // For storing the direction of the robot movement or navigating the interface system
byte axis = 0; // For controlling the DOF of the robot
volatile byte segment = 0; // For what part of trajectories we're executing

const double L[3] = {0.06, 0.218, 0.285}; // The lengths of the robot links
const double L_2[2] = {pow(L[1], 2), pow(L[2], 2)}; // Used in inverse kinematiks (the robot links lengths squared)
const double alpha = -0.0713; 
const double DATA2RAD = 0.00153435538636864;// Conversion from the motors' units for angle to radians
const double RAD2DATA = 651.7395; // Conversion from radians to the motors' units for angle
int32_t outData[4] = {2048, 2600, 1536, 2048}; // The data to sent to the robot
const double outDataLimits[2][4] = {{-M_PI, -0.38052, -1.8458, -0.03529}, {M_PI, M_PI, 1.8443, 1.9210}};
const double posRefLim[2][4] = {{0.00, -0.2325, -2048 * DATA2RAD, (1615 - 2048) * DATA2RAD}, {0.46, 0.38, 2048 * DATA2RAD, 3300 * DATA2RAD}};

CrustCrawler bot;

///////////////// EMG /////////////////
const byte filter_size = 10;
const unsigned int input_thres[3] = {490, 50, 50};
unsigned int filter[3][filter_size] = {0};
int32_t filt_value[3] = {0};
unsigned int mean[3] = {0};
double activeTime[2] = {0};
bool activeEMG[2] = {0};
const unsigned int activeTimeThresh = 50; //ms
const unsigned int activeTimeFrequency = 1000;
double activeTimeCooldown = 0;
volatile byte EMG_state = 0;
EMGxBee xbee;

///////////////VARIABLES FOR THE INTERFACE///////////////////
//All of the shift register definition is here
const int SER_Pin = 23; //pin 14 on the 75HC595
int const RCLK_Pin = 22; //pin 12 on the 75HC595
int const SRCLK_Pin = 21; //pin 11 on the 75HC595

//How many of the shift registers – change this
#define number_of_74hc595s 2
#define numOfRegisterPins number_of_74hc595s * 8
boolean registers[numOfRegisterPins];

byte ledIndex; // to be changed later
byte ledpin[16] = {15, 14, 13, 0, 1, 12, 9 , 2, 11, 10, 7 , 8 , 6 , 5, 4, 3}; //all the pins of the leds
byte pinsMode[3] = {12, 9, 2}; // Pins for the 3 main modes
byte pinsMove[4] = {7 , 8 , 6 , 5}; // Pins for the fml LEDS
byte pinEat[2] = {11, 10}; // Pins for the 2 eating and drinking leds
byte pinConf[2] = {4, 3}; //Pins for the 2 conf leds
byte polarMove[2] = {0, 1}; //Pins for positive and negative

volatile byte blinkpin = 0; // Pin that we are going to light on
double blinkled = 0; //Allows us to set the blinking using millis()
byte *modearray; // Is being set to point to the array that the LEDs are currently moving in
byte asize; // Is set to the size of the modearray to control the indexing of modearray

bool StartBlink = false; //Indicates whether to start blinking or not
bool enterfun = true; //boolean to know if we have entered a function (stop blinking)
volatile byte moveMode = 0; //checking if we are moving in freemode
volatile bool BreakFree = false; // Is used as a flag between the main code and the interrupts to control automatic movement


IntervalTimer mainloop;
IntervalTimer xBeeTimer;
IntervalTimer blinkint;

void modeNav(bool gripper = false);

void maincode() {
  sendPosition(); // Sends the values found in controlSystem
  if (moveMode != 0) { // Check if it's time to move
    switch (moveMode) { // Decides how to find the next position for the servos
      case 1:
        if (EMG_state == 2)
          controlSystem(axis); // Gets the next joint values from the Free mode control
        break;
      case 2:
        eatMove(); // Gets next joint values for he eating trajectory
        break;
      case 3:
        drinkMove(); // Gets next joint values for he drinking trajectory
        break;
      case 4:
        sleepMove(); // Gets next joint values for he sleeping trajectory
        break;
    }
  }
}

void xBeeRead() {
  xbee.updateData(); // Gets the new values from the XBee module (from EMG module)
  checkEmgInput(); // Reads and interprets the data from the accelerometer and EMG module
}

void blinkcode() {
  if (!enterfun && millis() - blinkled > 200) { // all of this is for blinking the leds, happens every 200 ms
    blinkled = millis();
    for(byte i = 0; i < asize; i++) // Sets the leds to low for turning the next one off or on. Eliminates an LED bug
      registers[modearray[i]] = 0;
    TurnPin(blinkpin, StartBlink); // Turns the blinkpin on or off
    StartBlink = !StartBlink; // Goes from 0 to 1 or 1 to 0
  }
}


void setup() {
  delay(3000);
  xbee.begin(Serial4, 115200);
  bot.begin(Serial2, 24);
  delay(10);
  bot.profileVelocity(30); // Sets the robot's profile velocity
  bot.profileAcceleration(10); // Sets the robot's profile acceleration
  bot.enableTorque(1);
  
  bot.setPGain(0x66, 850); // Grains for gripper
  bot.setIGain(0x66, 0);
  bot.setDGain(0x66, 0);
  bot.setPGain(1, 900); // Gains for joint 1
  bot.setIGain(1, 50);
  bot.setDGain(1, 35);
  bot.setPGain(2, 1900); // Gains for joint 2
  bot.setIGain(2, 150);
  bot.setDGain(2, 80);
  bot.setPGain(3, 1300); // Gains for joint 3
  bot.setIGain(3, 120);
  bot.setDGain(3, 35);
  
  xBeeTimer.begin(xBeeRead, 10000); // Makes the xBeeRead function run every 10 ms
  mainloop.begin(maincode, samplerate * 1000); // Makes the maincode function run samplerate ms
  blinkint.begin(blinkcode, 20000); // Makes the blinkcode run every 20 ms
  Serial.begin(9600); // For communication with the computer for debugging purposes
  pinMode(SER_Pin, OUTPUT); // Defining these pins as output
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);
  turnOff(); // Turns off the LEDs that aren't supposed to be turned on
}


void loop() {
  Movement(); //Main decisionmaking function
}


///////////////// For controlSystem /////////////////
void controlSystem(byte axis) { // axis is the DOF (x, z or rotz) that the user wishes to move
  double theta[4]; // Values for storing position of joints
  static double posRef[4]; // Variables for x, z, rotz and the gripper angle. These are the values to increment

 
  if(firstMoveRound){ // Runs first time this mode is called. Calculates the needed information for cartesian movements
    for (byte i = 0; i < 4; i++) // Calculates the angles of the robot joints
      theta[i] = (outData[i] - 2048) * DATA2RAD;
    
  ///////////////// FKIN /////////////////
    posRef[0] = L[1] * cos(theta[1]) + L[2] * cos(theta[1] + theta[2] - 0.0716); // Calculates the x value of the end-effector
    posRef[1] = L[1] * sin(theta[1]) + L[2] * sin(theta[1] + theta[2] - 0.0716); // Calculates the z value of the end-effector
    posRef[2] = theta[0]; // Calculates the rotation about z
    posRef[3] = theta[3]; // Calculates the angle of the gripper
    
    firstMoveRound = false;
  }

  double tempRef = posRef[axis]; // Saving current posRef in case incrementation is outside some boundary

  posRef[axis] += (dir * vel[axis] * sampleperiod * velMultiplier); // Incrementing x, z, rotz or gripper angle

  if(posRef[axis] < posRefLim[0][axis] || posRef[axis] > posRefLim[1][axis]){ // Check if the new posRef value is within boundaries
    posRef[axis] = tempRef; // Resets posRef to old if new is outside boundaries
    return;
  }

  double temp[4] = {theta[0], theta[1], theta[2], theta[3]}; // Saves the angles in case the new angles from the inverse kinematiks are outside boundaries

  ///////////////// IKIN /////////////////
  double pos_2[2] = {pow(posRef[0], 2), pow(posRef[1], 2)}; // Some optimization for calculating inverse kinematiks

  // Doing the inverse kinematiks calculations
  theta[0] = posRef[2];
  
  double c1 = 2 * L[1] * L[2] * cos(alpha),
  c2 = -2 * L[1] * L[2] * sin(alpha),
  c3 = L_2[0] + L_2[1] - pos_2[0] - pos_2[1];

  theta[2] = -2 * atan2(c2 + sqrt(abs(pow(c2, 2) + pow(c1, 2) - pow(c3, 2))), c1 - c3);

  c2 = L[1] + L[2] * cos(theta[2] + alpha);
  c1 = L[2] * sin(theta[2] + alpha);
  c3 = posRef[1];
  
  theta[1] = atan2(c3, sqrt(abs(pow(c1, 2) +pow(c2, 2) - pow(c3, 2))))- atan2(c1, c2);
  theta[3] = posRef[3];

  for(byte i = 0; i < 4; i++) // Checks if angles are poutside boundaries or NaN. If they are, reset them together with the posRef.
    if(theta[i] != theta[i] || theta[i] < outDataLimits[0][i] || theta[i] > outDataLimits[1][i]){
      for(byte j = 0; j < 4; j++)
        theta[j] = temp[j];
      posRef[axis] = tempRef;
      return;
    }

  for (byte i = 0; i < 4; i++)
    outData[i] = theta[i] * RAD2DATA + 2048; // Calculates the positions to send to the robot and stores them in array to send
}

void eatMove(){ // Passes values from current trajectory to outData to send to the robot when executing eat trajectories
  static int index = -1; // Static index to remember value for next call to this function
  index++; // Incrementing index by 1 to access new values for the trajectory
  for(byte i = 0; i < 3; i++) // Gets values for the first three joints
    outData[i] = eatTraj[segment][i][index];

  if(index == eatTrajSize[segment] - 1){ // Checks if the values accessed were the last ones
    BreakFree = true; // Sets this flag to true so that the current eat segment ends
    index = -1; // Resets index for next time this function is called
  }
}

void drinkMove(){ // See eatMove, but for drinking trajectories
  static int index = -1;
  index++;
  for(byte i = 0; i < 3; i++)
    outData[i] = drinkTraj[segment][i][index];

  if(index == drinkTrajSize[segment] - 1){
    BreakFree = true;
    index = -1;
  }
}

void sleepMove(){ // See eatMove, but for the sleep trajectories
  static int index = -1;
  index++;
  for(byte i = 0; i < 3; i++)
    outData[i] = sleepTraj[i][index];
  

  if(index == sleepTrajSize - 1){
    BreakFree = true;
    index = -1;
  }
}


///////////////// EMG /////////////////
void checkEmgInput() {
  unsigned int input[3] = {xbee.getAccX(), xbee.getEMG1(), xbee.getEMG2()}; // Gets the values from the accelerometer and EMG value
  static byte filt_index = 0; // Is used to index the filter array to store the input values
  filt_index++; // Increments to index next filter element
  filt_index = (filt_index < filter_size ? filt_index : 0); // Gets reset if the filt_index has exceeded the number of elements in the filter array

  for(byte i = 0; i < 3; i++) // Resets if the input is above the maximum value (this has fixed an important bug, don't touch!)
    input[i] = (input[i] < 1024 ? input[i] : 0);

  for(byte i = 0; i < 3; i++){ // Saves the inputs in the filter array and calculates the new means
    filt_value[i] = filt_value[i] + input[i] - filter[i][filt_index]; // Removes the oldest data and inserts the new into filt_value. Filt_value holds all values from the filter
    filter[i][filt_index] = input[i]; // Saves the new data
    
    mean[i] = filt_value[i] / filter_size; // Mean is calculated
  }
  
  if (mean[0] > input_thres[0] + 25){ // Checks if the accelerometer is tilting to positive, negative or no side
    dir = -1;
    TurnPin(polarMove[0], 1);
    TurnPin(polarMove[1], 0);
  }
  else if(mean[0] < input_thres[0] - 25){
    dir = 1;
    TurnPin(polarMove[0], 0);
    TurnPin(polarMove[1], 1);
  }else{
    dir = 0;
    TurnPin(polarMove[0], 0);
    TurnPin(polarMove[1], 0);
  }

  for (byte i = 0; i < 2; i++) { // Checks activation conditions for the EMG channels
    if (mean[i + 1] > input_thres[i + 1]) {
      if (!activeEMG[i]) {
        activeEMG[i] = true;
        activeTime[i] = millis();
      }
    } else {
      activeEMG[i] = false;
    }
  }

  bool eligible[2] = { // Checks if the EMG channels have been active for a long enough interval
    millis() - activeTime[0] > activeTimeThresh, millis() - activeTime[1] > activeTimeThresh
  };

  if (activeEMG[0] && eligible[0] && activeEMG[1] && eligible[1] && millis() - activeTimeCooldown > activeTimeFrequency) { // Sets EMG_state to 1 if both eyes are blinking
    EMG_state = 1;
    activeTimeCooldown = millis();
  }
  else if (activeEMG[0] && eligible[0] && activeTime[0] - activeTime[1] > activeTimeThresh) { // Sets EMG_state to 2 if left eye is blinking
    EMG_state = 2;
    if(mean[1] < input_thres[2] * 2.5){ // Decides the multiplier for the speed based on the mean of the EMG channel for the left eye
      velMultiplier = 1;
      bot.profileVelocity(30);
      bot.profileAcceleration(10);
    } else if(mean[1] < input_thres[2] * 5){
      velMultiplier = 1.33;
      bot.profileVelocity(40);
      bot.profileAcceleration(13);
    } else if(mean[1] < input_thres[2] * 7.5){
      velMultiplier = 1.66;
      bot.profileVelocity(50);
      bot.profileAcceleration(17);
    } else{
      velMultiplier = 2;
      bot.profileVelocity(60);
      bot.profileAcceleration(20);
    }
  }
  else if (activeEMG[1] && eligible[1] && activeTime[1] - activeTime[0] > activeTimeThresh && millis() - activeTimeCooldown > activeTimeFrequency) { // Sets EMG_state to 3 if right eye is blinking
    EMG_state = 3;
    activeTimeCooldown = millis();
  }
  else { // If no eyes have been blinking for long enough, the EMG_state is set to 0
    EMG_state = 0;
  }
}


/////////////////////FUNCTIONS FOR LED INTERFACE/////////////////////

void turnOff () {         //All Leds except sleep and the positive, negative tilt indicators are set to 0
  registers[ledpin[1]] = 1;
  registers[ledpin[0]] = 0;
  registers[ledpin[2]] = 0;
  for (byte i = 5; i < 16; i++)
    registers[ledpin[i]] = 0;
  writeRegisters();
}

void exitfunction() {
  TurnPin(ledpin[2], LOW); //Ensure that the moverobot is not on and turns everything in the pins mode to off
  for (byte i = 5; i < 16; i++)
    registers[ledpin[i]] = 0;
  writeRegisters();

  switch (modearray[ledIndex]) { // Exits to main mode depending on which LEDs are off and on
    case 11:
    case 10:
      exitToMode(0); // Exits Feeding mode
      break;
    case 7:
    case 8:
    case 6:
    case 5:
      exitToMode(1); // Exits Free Control mode
      break;
    case 4:
    case 3:
      exitToMode(2); // Exits Config mode
      break;
  }
}

void exitToMode(byte i){ // Exits a submode
  modearray = pinsMode;
  asize = sizeof(pinsMode);
  ledIndex = i;
  enterfun = false;
  StartBlink = true;
  blinkpin = modearray[ledIndex];
  moveMode = 0;
}


void Movement() { //Move through each array using modearray to pass each array
  modeNav();

  if (EMG_state == 1 && dir == 1) { //Enter function
    if (registers[ledpin[1]] == 0) { // If the sleeping LED is off
      enterfun = true; //stops blinking and set the pin we are in to high
      TurnPin(modearray[ledIndex], HIGH);

      switch (modearray[ledIndex]) { //checks the pin we are in and calls the corresponding function
        case 12 :
          feeding();
          break;
        case 11 :
          eat();
          break;
        case 10 :
          drink();
          break;
        case 9 :
          freecontrol();
          break;
        case 7 :
        case 8 :
        case 6 :
        case 5 :
          savepos();
          break;
        case 2 :
          conf();
          break;
        case 4 :
          sleeping();
          break;
        case 3 :
          savepos();
          break;
      }
    } else { // Starts robot from sleeping mode
      for (byte i = 5; i < 16; i++) { 
        TurnPin(ledpin[i], LOW);
      }
      
      bot.enableTorque(1); // Enables torque for the servos
      
      TurnPin(ledpin[0], 1); //Sets On to On 
      TurnPin(ledpin[1], 0); // and Off to Off

      outData[0] = 2048; outData[1] = 2600; outData[2] = 1536; outData[3] = 2048; // Assigns the home position to the servos
      
      ledIndex = 0; // Resets LED index for navigating the LED interface
      enterfun = false;
      blinkpin = pinsMode[ledIndex]; // Sets the first LED to blink
      TurnPin(pinsMode[ledIndex], 1); // Turns on the first LED so that it doesn't start being off

      modearray = pinsMode;
      asize = sizeof(pinsMode);
    }

    EMG_state = 0; // EMG_state is set to 0 to avoid triggering the same statements twice
  }

  if (EMG_state == 1 && dir == -1) { //exit function
    exitfunction(); // Exits current function
    EMG_state = 0;
  }
}

void modeNav(bool gripper){ // Is the movement to the left or the right in the current LED array
  if (EMG_state == 3 && dir == 1 && ledIndex + 1 < (asize - gripper)) {
    ledIndex++;
    blinkpin = modearray[ledIndex];
    TurnPin(modearray[ledIndex - 1], LOW); //sets the previous LED to 0
    axis = ledIndex;
    EMG_state = 0;
  }

  if (EMG_state == 3 && dir == -1 && ledIndex > 0) {
    ledIndex--;
    blinkpin = modearray[ledIndex];
    TurnPin(modearray[ledIndex + 1], LOW);
    axis = ledIndex;
    EMG_state = 0;
  }
}

//Now we declare the functions for each mode and submode
void feeding () { //Feeding mode enter, blinks first eat pin movement and passes the array
  enterfun = false;
  blinkpin = pinEat[ledIndex];
  TurnPin(pinEat[ledIndex], 1);

  modearray = pinEat;
  asize = sizeof(pinEat);
}

void eat() { // The algorithm for eating
  modearray = pinEat;
  asize = sizeof(pinEat);
  registers[pinEat[0]] = 1; // Turns on relevant LEDs
  TurnPin(ledpin[2], HIGH);

  outData[3] = 2348; // Opens gripper

  // These segments go to spoon
  moveMode = 2; // Starts getting points from eating trajectory
  for(segment = 0; segment < 5; segment++) { // Goes through 5 trajectories
    while(!BreakFree) {} // Stops code from running until BreakFree has been set to true in the IRS
    BreakFree = false;
  }
  moveMode = 0; // Stops robot from getting eat trajectory values
  
  delay(500); // Waits half a second for the robot to settle completely
  outData[3] = 2030; // Closes gripper
  delay(1500); // Waits for gripper to close

  // Same as before, but for 7 other segments (goes to bowl)
  moveMode = 2;
  for(segment = 5; segment < 12; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }
  
  while(1) {
    freecontrolEat(); // Starts eat mode with no control of gripper
    modearray = pinEat;
    asize = sizeof(pinEat);
    TurnPin(ledpin[2], HIGH);

    // Goes to mouth
    moveMode = 2;
    for(segment = 12; segment < 14; segment++) {
      while (!BreakFree) {}
      BreakFree = false;
    }

    moveMode = 0;
    while(1){ // Waits for user input before moving to bowl or places spoon back
      if(EMG_state == 1 && dir == 1){
        break;
      }
      else if(EMG_state == 1 && dir == -1){
        goto loopbreak; // Is used to break two while loops at once
      }
    }

    // Goes to bowl
    moveMode = 2;
    for(segment = 14; segment < 16; segment++) {
      while(!BreakFree) {}
      BreakFree = false;
    }
  }

  loopbreak: moveMode = 2;
  // Goes to home
  for(segment = 16; segment < 17; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  // Goes to spoon holder
  for(segment = 0; segment < 5; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  moveMode = 0;
  delay(500);
  outData[3] = 2348; // Opens gripper
  delay(1500);

  // Goes to home
  moveMode = 2;
  for(segment = 5; segment < 10; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }
  
  outData[3] = 2048; // Closes gripper

  moveMode = 0;
  TurnPin(ledpin[2], 0);

  exitfunction(); // Exits function
}


void drink() { //Drinking function, see eat() for details, the functions are mostly the same
  modearray = pinEat;
  asize = sizeof(pinEat);
  TurnPin(pinEat[1], HIGH);
  TurnPin(ledpin[2], HIGH);

  outData[3] = 2500;

  moveMode = 3;
  // Goes to bottle position
  for(segment = 0; segment < 2; segment++){
    while(!BreakFree){}
    BreakFree = false;
  }
  
  moveMode = 0;
  delay(500);
  outData[3] = 2080; // Closes gripper
  delay(1500);

  // Goes to mouth
  moveMode = 3;
  for(segment = 2; segment < 5; segment++){
    while(!BreakFree){}
    BreakFree = false;
  }

  moveMode = 0;
  // Waits until user has signal they're done drinking
  while(1)
    if(EMG_state == 1 && dir == -1)
      break;

  moveMode = 3;
  // Goes home
  for(segment = 5; segment < 6; segment++){
    while(!BreakFree){}
    BreakFree = false;
  }
  // Goes to bottle position
  for(segment = 0; segment < 2; segment++){
    while(!BreakFree){}
    BreakFree = false;
  }

  moveMode = 0;
  delay(1000);
  outData[3] = 2500; // Opens gripper
  delay(1500);

  // Moves away from bottle
  moveMode = 3;
  for(segment = 2; segment < 4; segment++){
    while(!BreakFree){}
    BreakFree = false;
  }

  outData[0] = 2048; // Goes to zero rotation about z
  outData[3] = 2048; // Closes gripper
  
  moveMode = 0;
  TurnPin(ledpin[2], 0);

  exitfunction();
}

void conf() { //Enters Conf mode
  ledIndex = 0;
  enterfun = false;
  blinkpin = pinConf[ledIndex];
  TurnPin(pinConf[ledIndex], 1);

  modearray = pinConf;
  asize = sizeof(pinConf);
}

void sleeping () { //Sets the robot to sleep

  goHome(1);

  moveMode = 4;  
  while(!BreakFree){}
  BreakFree = false;
  moveMode = 0; 

  delay(1500);

  bot.enableTorque(0); // Disables torque
  turnOff();
}


void savepos() { // Saves position of the mouth and recalculates trajectories For information on automatic movement, see eat()
  TurnPin(ledpin[2], 1);
  
  outData[3] = 2348;

  moveMode = 2;
  // Goes to spoon
  for(segment = 0; segment < 5; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  moveMode = 0;
  delay(500);
  outData[3] = 2030; // Closes gripper
  delay(1500);

  moveMode = 2;
  // Goes to home
  for(segment = 5; segment < 10; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  
  freecontrolEat(); // Gives gontrol of robot to user (not gripper)

  modearray = pinConf;
  asize = sizeof(pinConf);

  int32_t endTheta[3] = {0};

  for(byte i = 0; i < 3; i++)
    endTheta[i] = outData[i]; // Gets current position

  // Calculates the new trajectories for the mouth

  // PLATE TO MOUTH
  int v0array[3] = {0};
  int v1array[3] = {0};
  for(byte j = 0; j < 3; j++){
    float tf = (eatTrajSize[13] - 1) / 10;

    int startPos = eatTraj[13][j][0];
    int endPos = endTheta[j];
    int v1 = v1array[j];
    int v0 = v0array[j];
    
    float a0 = startPos,
    a1 = v0,
    a2 = (3/(tf * tf)) * (endPos - startPos) - (2/tf) * v0 - (1/tf) * v1,
    a3 = (-2/(tf * tf * tf)) * (endPos - startPos) + (1/(tf * tf)) * (v1 + v0);
    
    for(byte i = 0; i < eatTrajSize[13]; i++){
      float t = 0.1 * i;
      float point = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
      eatTraj[13][j][i] = point;
    }
  }
  
  // MOUTH TO PLATE
  v0array[3] = {0};
  v1array[3] = {0};
  for(byte j = 0; j < 3; j++){
    float tf = (eatTrajSize[14] - 1) / 10;

    int startPos = endTheta[j];
    int endPos = eatTraj[14][j][eatTrajSize[14] - 1];
    int v1 = v0array[j];
    int v0 = v0array[j];
    
    float a0 = startPos,
    a1 = v0,
    a2 = (3/(tf * tf)) * (endPos - startPos) - (2/tf) * v0 - (1/tf) * v1,
    a3 = (-2/(tf * tf * tf)) * (endPos - startPos) + (1/(tf * tf)) * (v1 + v0);
    
    for(byte i = 0; i < eatTrajSize[14]; i++){
      float t = 0.1 * i;
      float point = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
      eatTraj[14][j][i] = point;
    }
  }

  // MOUTH TO 45 DEGREES
  v0array[3] = {0};
  v1array[3] = {0};
  for(byte j = 0; j < 3; j++){
    float tf = (eatTrajSize[16] - 1) / 10;

    int startPos = endTheta[j];
    int endPos = eatTraj[16][j][eatTrajSize[16] - 1];
    int v1 = v0array[j];
    int v0 = v0array[j];
    
    float a0 = startPos,
    a1 = v0,
    a2 = (3/(tf * tf)) * (endPos - startPos) - (2/tf) * v0 - (1/tf) * v1,
    a3 = (-2/(tf * tf * tf)) * (endPos - startPos) + (1/(tf * tf)) * (v1 + v0);
    
    for(byte i = 0; i < eatTrajSize[16]; i++){
      float t = 0.1 * i;
      float point = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
      eatTraj[16][j][i] = point;
    }
  }
  
  // TO MOUTH
  v0array[3] = {0};
  v1array[3] = {0};
  for(byte j = 0; j < 3; j++){
    float tf = (drinkTrajSize[4] - 1) / 10;

    int startPos = drinkTraj[4][j][0];
    int endPos = endTheta[j];
    int v1 = v0array[j];
    int v0 = v0array[j];
    
    float a0 = startPos,
    a1 = v0,
    a2 = (3/(tf * tf)) * (endPos - startPos) - (2/tf) * v0 - (1/tf) * v1,
    a3 = (-2/(tf * tf * tf)) * (endPos - startPos) + (1/(tf * tf)) * (v1 + v0);
    
    for(byte i = 0; i < drinkTrajSize[4]; i++){
      float t = 0.1 * i;
      float point = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
      drinkTraj[4][j][i] = point;
    }
  }

  // FROM MOUTH
  v0array[3] = {0};
  v1array[3] = {0};
  for(byte j = 0; j < 3; j++){
    float tf = (drinkTrajSize[5] - 1) / 10;

    int startPos = endTheta[j];
    int endPos = drinkTraj[5][j][drinkTrajSize[5] - 1];
    int v1 = v0array[j];
    int v0 = v0array[j];
    
    float a0 = startPos,
    a1 = v0,
    a2 = (3/(tf * tf)) * (endPos - startPos) - (2/tf) * v0 - (1/tf) * v1,
    a3 = (-2/(tf * tf * tf)) * (endPos - startPos) + (1/(tf * tf)) * (v1 + v0);
    
    for(byte i = 0; i < drinkTrajSize[5]; i++){
      float t = 0.1 * i;
      float point = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
      drinkTraj[5][j][i] = point;
    }
  }

  
  moveMode = 2;
  for(segment = 0; segment < 5; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  moveMode = 0;
  delay(500);
  outData[3] = 2348;
  delay(1500);

  moveMode = 2;
  for(segment = 5; segment < 10; segment++) {
    while(!BreakFree) {}
    BreakFree = false;
  }

  outData[3] = 2048;

  exitToMode(2);
  TurnPin(ledpin[15], 0);
  TurnPin(ledpin[2], 0);
}


void freecontrol() { // The free control function that stops the code when using Free control mode
  firstMoveRound = true;
  moveMode = 1;
  TurnPin(ledpin[2], HIGH);
  modearray = pinsMove;
  asize = sizeof(pinsMove);
  
  ledIndex = 0;
  axis = ledIndex;
  TurnPin(pinsMode[1], 1);
  enterfun = false;
  blinkpin = pinsMove[ledIndex];

  while(1){
    modeNav(); // Lets the user navigate between the axes
    if(EMG_state == 1 && dir == -1) // Checks if the user wants to exit Free control mode
      break; 
  }
  moveMode = 0;
  goHome(false);
  exitfunction();
}

void freecontrolEat() { // Is the same as the funcion above but sets enterfun to true at end and doesn't start the move mode main LED
  firstMoveRound = true;
  moveMode = 1;
  modearray = pinsMove;
  asize = sizeof(pinsMove);
  
  ledIndex = 0;
  axis = ledIndex;
  enterfun = false;
  blinkpin = pinsMove[ledIndex];

  while(1){
    modeNav(1);
    if(EMG_state == 1 && dir == -1)
      break; 
  }
  
  moveMode = 0;
  enterfun = true;
  exitfunction();
}

void writeRegisters() { // Writes the new values to the SHIFT registers
  digitalWrite(RCLK_Pin, LOW);

  for (int i = 0; i < numOfRegisterPins; i++) {
    digitalWrite(SRCLK_Pin, LOW);

    digitalWrite(SER_Pin, registers[i]);
    
    digitalWrite(SRCLK_Pin, HIGH);
  }
  digitalWrite(RCLK_Pin, HIGH);
}

void goHome(bool sleeping){
  int16_t target[3] = {2048, 2600, 1536}; // The home position, needed for robot to go to sleep

  int16_t difference[3] = {abs(outData[0] - target[0]), abs(outData[1] - target[1]), abs(outData[2] - target[2])}; // Finds the size of the movements

  float stepVel = 468.8775; // for profileVelocity of 30, used to calculate move timeMove that is the time needed for the movement
  float timeMove = (difference[1] > difference[2] ? difference[1] : difference[2]) / stepVel; // Does biggest move over speed to find time needed for mvoement

  outData[1] = target[1]; // Goes to the home position for joint 2 and 3 in order to not bump into stuff when rotating about z
  outData[2] = target[2];
  outData[3] = 3200; // Opens gripper a lot

  delay(timeMove * 1000);

  timeMove = difference[0] / stepVel; // Does the same thing for joint 1
  outData[0] = target[0];
  
  delay(timeMove * 1000 + 500); // Added half a second for the robot to better settle before turning off torque

  if(!sleeping)
    outData[3] = 2048;
}

//set an individual pin HIGH or LOW
void TurnPin(byte pin, int value) {
  registers[pin] = value; // sets the input pin to 0 to turn off the corresponding LED
  writeRegisters(); // Writes to the registers
}

void sendPosition() { // Sends the new joint values to the servos
  bot.moveJoints(outData[0], outData[1], outData[2], outData[3]);
}
