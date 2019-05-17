
/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 Code from:
 Remo Büchi
 Franziska Bürgler
 Luca Urban
 FS 2019
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "string.h"

//-- Servo stuff -- 05.04.19
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
const int SERVOPIN = 10;

//The position which can be driven at
enum myPositions{
  POSITION_LEFT = 0, 
  POSITION_RIGHT = 180,
  POSITION_IDLE = 90
};

//The speed at which the servo can be driven at
int speedOfDrive = 1; //how many degrees per step will be driven
int waittimePerDegree = 16; //how long will be waited till the next step is done - in milliseconds

int currentPosition = POSITION_IDLE;    // variable to store the current servo position
// -- End of Servo stuff

//Sensor stuff

const int SENSORPIN = 12;

//End of sensor stuff

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  
  //Communication without bluetooth (via Serial monitor)
  while (!Serial);
  delay(500);

  Serial.begin(115200);
  //Serial.println("ready or not, here I come");
  
  //------------ End of Communication setup ----------------
  /*---------------- Setup Servo ----------------- */
  // initialize digital pin 13 as an output.
  pinMode(SERVOPIN, OUTPUT);
  

  myservo.attach(SERVOPIN);//Connect servo at Pin
  myservo.write(currentPosition); //go to currentPosition (should be IDLE Position at this point
  

  /* --------------- Setup Sensor ---------------*/
  pinMode(SENSORPIN, INPUT);
}

/*SECOND TRY*/
//Communicate via Serial
void sendAnswer(char text[]){
    //Comunication without bluetooth
    Serial.println(text);
}

//check sensor, returns true if package detected
bool packageDetected(){
  //Serial.print(" checksensor -- sensor is");
  int sensorInput = 0;
  sensorInput = digitalRead(SENSORPIN);
  //Serial.println(sensorInput);
  
  bool returnvalue = false;
  
  //If sensor returns 0 = LOW --> Something detected
  //Source http://wiki.seeedstudio.com/Grove-Digital_Distance_Interrupter_0.5_to_5cm-GP2Y0D805Z0F_P/#resources
  if(sensorInput == LOW){
    returnvalue = true;
  }
     
  return returnvalue;
}

void sendAnswerPickup(){
  if(packageDetected()){
    sendAnswer("pickupSuccess(1)"); //pickup successfull
  }
  else{
    sendAnswer("pickupSuccess(0)"); //-- error on pickup
  } 
}

//same as sendAnswerPickup but reversed (drop is successfull, if there is no package in anymore)
void sendAnswerDrop(){
  if(!packageDetected()){
    sendAnswer("dropSuccess(1)"); //drop successfull
  }
  else{
    sendAnswer("dropSuccess(0)"); //-- error on drop
  } 
}

//Statemachine which stores the different possible states
enum myStateMachine{
  IDLESTATE = 0,
  PICKUP_RIGHT,
  PICKUP_LEFT,
  DROP_LEFT,
  DROP_RIGHT
  //testing positions
  , CHECK_POSITION_LEFT,
  CHECK_POSITION_RIGHT,
  CHECK_POSITION_IDLE
};
int currentState = IDLESTATE;

//Change state according to the input (from Serial)
void checkforInput(char input[]){ 
  
  if ((strcmp(input,"pickup(0)\n") == 0) || strcmp(input,"pickup(0)") == 0){
      currentState = PICKUP_LEFT;
  }
  if (strcmp(input,"pickup(1)\n") == 0 || strcmp(input,"pickup(1)") == 0){
      currentState = PICKUP_RIGHT;
  }
  if (strcmp(input,"drop(0)\n") == 0 || strcmp(input,"drop(0)") == 0){
      currentState = DROP_LEFT;
  }
  if (strcmp(input,"drop(1)\n") == 0 || strcmp(input,"drop(1)") == 0){
      currentState = DROP_RIGHT;
  }

  //For Testing purpuoses - comment out, once all positions are clear
  if (strcmp(input,"testleft\n") == 0 || strcmp(input,"testleft") == 0){
      currentState = CHECK_POSITION_LEFT;
  }
  if (strcmp(input,"testright\n") == 0 || strcmp(input,"testright") == 0){
      currentState = CHECK_POSITION_RIGHT;
  }
     if (strcmp(input,"testidle\n") == 0 || strcmp(input,"testidle") == 0){
      currentState = CHECK_POSITION_IDLE;
  }
  // --End of Testing
  //Serial.println("CheckforInput was called");
  //Serial.print(currentState);
}

//driving
void drive(int positionToDriveAt){
  //Decide if we have to turn right or left
  if(currentPosition>positionToDriveAt){
    for(currentPosition; currentPosition > positionToDriveAt; currentPosition-=speedOfDrive){
        myservo.write(currentPosition);
        delay(waittimePerDegree); 
      }
  }
  else{
    for(currentPosition; currentPosition < positionToDriveAt; currentPosition+=speedOfDrive){
        myservo.write(currentPosition);
        delay(waittimePerDegree); 
      }
    }
}

// do the working stuff
// what to do at which state
void work(void){
  switch(currentState){
    case IDLESTATE:
      // do nothing, wait for input
      
    break;
    case PICKUP_LEFT:
      drive(POSITION_LEFT);
      
      delay(50); // wait a moment
      drive(POSITION_IDLE);
      delay(50); // wait a moment
      
      sendAnswerPickup();
      currentState = IDLESTATE;
    break;
    case PICKUP_RIGHT:
      drive(POSITION_RIGHT);
      
      delay(50); // wait a moment
      drive(POSITION_IDLE);
      delay(50); // wait a moment
      
      sendAnswerPickup(); //send answer if pickup was successfull
      
      currentState = IDLESTATE;
    break;
    case DROP_LEFT:
      drive(POSITION_LEFT);

      delay(50); // wait a moment
      drive(POSITION_IDLE);
      delay(50); // wait a moment

      sendAnswerDrop(); //send answer if drop was successfull
       
      currentState = IDLESTATE;
    break;
    case DROP_RIGHT:
      drive(POSITION_RIGHT);
      
      delay(50); // wait a moment
      drive(POSITION_IDLE);
      delay(50); // wait a moment
      
      sendAnswerDrop(); //send answer if drop was successfull
      
      currentState = IDLESTATE;
    break;

    //testpositions
     case CHECK_POSITION_LEFT:
      drive(POSITION_LEFT);
      
      currentState = IDLESTATE;
    break;
     case CHECK_POSITION_RIGHT:
      drive(POSITION_RIGHT);
      
      currentState = IDLESTATE;
    break;
     case CHECK_POSITION_IDLE:
      drive(POSITION_IDLE);
      
      currentState = IDLESTATE;
    break;
  }
   
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char inputs[BUFSIZE+1];

  //check if user used one of the known commands like pickup or drop
  if ( getUserInput(inputs, BUFSIZE) )
  {
    //Serial.println("CheckForInputs by inputs: ");
    //Serial.println(inputs);
    checkforInput(inputs);
  }
  //do what has to be done in the current state
  work();
 
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}
