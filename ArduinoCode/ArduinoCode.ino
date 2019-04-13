
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
 Erweiterungen des Besipiels durch:
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

//Servo stuff -- 05.04.19
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
int waittimePerDegree = 16; //in milliseconds

int currentPosition = POSITION_IDLE;    // variable to store the servo position
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
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }


  /*---------------- Setup Servo ----------------- */
  // initialize digital pin 13 as an output.
  pinMode(SERVOPIN, OUTPUT);
  

  myservo.attach(SERVOPIN);//Servo mit Pin verbinden

  /* --------------- Setup Sensor ---------------*/
  pinMode(SENSORPIN, INPUT);
}

/*SECOND TRY*/
//Statemachine which stores the different possible states
enum myStateMachine{
  IDLESTATE = 0,
  PICKUP_RIGHT,
  PICKUP_LEFT,
  DROP_LEFT,
  DROP_RIGHT
};
int currentState = IDLESTATE;

//Change state according to the input (from bluetooth)
void checkforInput(String input){
  if (input == "pickup(0)"){
      currentState = PICKUP_LEFT;
  }
  if(input == "pickup(1)"){
      currentState = PICKUP_RIGHT;
  }
  if(input == "drop(0)"){
      currentState = DROP_LEFT;
  }
  if(input == "drop(1)"){
      currentState = DROP_RIGHT;
  }
  Serial.println("CheckforInput was called");
  Serial.print(currentState);
}

//driving
void drive(int positionToDriveAt){
  //Decide if we have to turn right or left
  if(currentPosition>positionToDriveAt){
    for(currentPosition; currentPosition > positionToDriveAt; currentPosition-=speedOfDrive){
        myservo.write(currentPosition);
        //Serial.print("going to Position left, currently at:");
        //Serial.println(currentPosition);
        delay(waittimePerDegree); 
      }
  }
  else{
    for(currentPosition; currentPosition < positionToDriveAt; currentPosition+=speedOfDrive){
        myservo.write(currentPosition);
        //Serial.print("going to Position , currently at:");
        //Serial.println(currentPosition);
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
       
       Serial.print("going to Position left, currentPosition");
      Serial.println(currentPosition);
      drive(POSITION_LEFT);
      Serial.print("Position left reached, currentPosition");
      Serial.println(currentPosition);
    break;
    case PICKUP_RIGHT:
      //go to position right
      //myservo.write(POSITION_RIGHT);              // tell servo to go to position in variable 'POSITION_RIGHT'
      Serial.print("going to Position right");
      drive(POSITION_RIGHT);
      Serial.println(POSITION_RIGHT);
      delay(50); 
    break;
    case DROP_LEFT:
      //go to position right
      //myservo.write(POSITION_IDLE);              // tell servo to go to position in variable 'POSITION_RIGHT'
      Serial.print("going to Position IDLE");
      drive(POSITION_IDLE);
      Serial.println(POSITION_IDLE);
      delay(50);  
    break;
    case DROP_RIGHT:
    
    break;
  }
  Serial.print(" 0000000000000000000000000 sensor is");
    short sensorInput = 0;
    sensorInput = digitalRead(SENSORPIN);
    Serial.println((int)sensorInput);  
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

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }

  //check if user used one of the known commands like pickup or drop
  checkforInput(ble.buffer);
  Serial.println("currentState");
  Serial.println(currentState);

  //do what has to be done in the current state
  work();
  
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  ble.waitForOK();

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
