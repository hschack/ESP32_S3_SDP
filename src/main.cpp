/*=============================================================================
File Name           : 
Project File Number : v 
Project Name        : 
Author              : HS
Start Date          : 
Chip                : 
Copyright(c) 2023, All Rights Reserved.
-------------------------------------------------------------------------------
Description: Draft Samsø Boiler 4
-----------------------------------------------------------------------------*/
/***************************   Include Files   *******************************/
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <Wire.h>
#include <WiFi.h>
#include "secrets.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
/***************************   Defines   *************************************/
#define LED                       38 // the number of the ALIVELED pin
#define SDP810_I2C_ADDR           0x25 // I2C Address for SDP8xx device
#define FILTER_CONST              0.90f //
#define PRINT_DEBUG_MESSAGES
#define I2C_SDA 4
#define I2C_SCL 5
/***************************   Flags and bit *********************************/
/***************************   sbit ******************************************/
/***************************   Macros   **************************************/
/***************************   Data Types   **********************************/
/***************************   Local Variables   *****************************/
volatile float      FilteredPressure_1       = 0;
volatile float      FilteredTemperatur_1     = 0;
volatile float      pa_low                   = 0;
volatile float      pa_high                  = 0;
//
unsigned int        ResetCounter             = 0;
unsigned int        LEDOnOffTime             = 5000;
unsigned long       previousMillis           = 0; // will store last time LED was updated
unsigned long       SendInterval             = 0;
unsigned long       myChannelNumber          = SECRET_CH_ID;
const char *        myWriteAPIKey            = SECRET_WRITE_APIKEY;

/*****************************************************************************/
char ssid[] = SECRET_SSID;   // your network SSID (name) 
char pass[] = SECRET_PASS;   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;
/*************************** ThingSpeak Settings *****************************/
//String myStatus = "";
/***************************  Enum  ******************************************/
typedef enum 
{ 
    IDLE_STATE = 0, 
    SEND_I2C_READ_SENSOR_STATE = 1, 
    READ_I2C_SENSOR_STATE = 2,
    PRINT_SDP_VALUE_STATE = 3
} sensorState;

static sensorState currentSdp810State = IDLE_STATE;
static sensorState prevState          = IDLE_STATE;
/***************************   Constants   ***********************************/
/***************************   Global Variables   ****************************/
/***************************   Function Prototypes   *************************/
void setup(void);
void Setup_wifi();
void SetupESP32(void);
void loop(void);
void ConnectWiFi(void);
void GetSdPressure(void);
void ResetSensor_1(void);
void SendToServer(void);          // Send PA data to server 
void BlinkLED (void);
/******************************************************************************
Function name : void setup()
         Type : PRIVATE
Description   : Run after start/reset
Notes :
******************************************************************************/
//Serial setup
//                      RX    TX
//HardwareSerial Serial(PA10, PA9);
void setup() {
     SetupESP32();
     Setup_wifi();
     ResetSensor_1();
     delay(100);   
}
/******************************************************************************
Function name : void Loop()
         Type : PRIVATE
Description   :
Notes :
******************************************************************************/
void loop(){
    ConnectWiFi();
    GetSdPressure();
    if (millis() - SendInterval >= 60000) {
        SendToServer();
        pa_low  = 0.0;
        pa_high = 0.0;         
      }
    if (ResetCounter >= 240){
      ESP.restart();
    }  
    BlinkLED ();
} // END loop
/******************************************************************************
Function name : 
         Type :
Description   : 
Notes :
******************************************************************************/
/******************************************************************************
Function name : GetSdPressure(void)
         Type : PRIVATE
Description   : Read Sensor value use FSM
Notes :
******************************************************************************/
//
void GetSdPressure(void){
/***************************   Local Variables   *****************************/
volatile static float      difPressure_1        = 0.0;
volatile static float      temperatur_1         = 0.0;
static unsigned long       interval             = millis();
static unsigned long       printInterval        = millis();
static int                 n                    = 0;
volatile int               pressure_sint;
volatile int               temperatur_sint;
volatile static int        data[8];
//
  switch (currentSdp810State) 
    {
        case IDLE_STATE:{
            // Do nothing here for now
            // Move to next state
            currentSdp810State = SEND_I2C_READ_SENSOR_STATE;
        }  
            break;
        // 
        case SEND_I2C_READ_SENSOR_STATE:{ // ca 280 microsek.
          if (millis() - interval >= 10) {
            // step : instruct sensor command
            Wire.beginTransmission(SDP810_I2C_ADDR); // transmit to device #37 (0x25)
            //
            Wire.write(byte(0x36));      // msb
            Wire.write(byte(0x15));      // lsb command sensor Differential pressure Average till read (0x3615)
            Wire.endTransmission();      // stop transmitting
            interval = millis();
            // Move to next state
            currentSdp810State = READ_I2C_SENSOR_STATE;            
          }  
        }    
            break;
         //
        case READ_I2C_SENSOR_STATE:{
            // step : request reading from sensor
            Wire.requestFrom(SDP810_I2C_ADDR, 8);    // request 8 bytes from slave device #37 (0x25)
            for (n=0;n<8;n++)
            {
                data[n] = Wire.read();          // receive byte (overwrites previous reading)
            }
            pressure_sint = (int16_t)(data[0]*256+data[1]);
            difPressure_1 = (float)(pressure_sint) / data[7]; // Scale Factor for 125pa type
              if((difPressure_1 >= 130) || (difPressure_1 <= -130)){
                Serial.println("DP error");
                break;
               }
            temperatur_sint = (int16_t)(data[3]*256+data[4]);
            temperatur_1 = (float)(temperatur_sint) / data[7]; // Scale Factor "data byte 7"
              if((temperatur_1 >= 85) || (temperatur_1 <= -30)){
                Serial.println("T error");
                break;
               }
            FilteredPressure_1 = FILTER_CONST*FilteredPressure_1 + difPressure_1*(1-FILTER_CONST);
            FilteredTemperatur_1 = FILTER_CONST*FilteredTemperatur_1 + temperatur_1*(1-FILTER_CONST);
              if (difPressure_1 < pa_low)
                {
                  pa_low = difPressure_1;     
                }
              else if (difPressure_1 > pa_high)
               {
                  pa_high = difPressure_1;  
               }
            currentSdp810State = PRINT_SDP_VALUE_STATE;
            break;
        }  
        //    
        case PRINT_SDP_VALUE_STATE:{
          if (millis() - printInterval >= 1000) {
            ResetCounter++;
            printInterval = millis(); 
           }
            // Move to next state
            currentSdp810State = IDLE_STATE;
           }
            break;
        //    
        default:
            currentSdp810State = IDLE_STATE;
            break;
  }
}
/******************************************************************************
Function name : void SetupESP32(void)
         Type :
Description   : 
Notes :
******************************************************************************/
void SetupESP32(){
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);    // set to known state
    Wire.setClock(400000);              // Set I2C clock to 400 kHz
    Serial.begin(115200);
    Wire.begin( I2C_SDA , I2C_SCL );
    ThingSpeak.begin(client);           // Initialize ThingSpeak
    LEDOnOffTime       = 6000;    
    delay(100);
 }
/******************************************************************************
Function name : void ResetSensor_1()
         Type :
Description   : Send reset to SDP810 adr 0x25
Notes :
******************************************************************************/
void ResetSensor_1()
{
   // step : instruct sensor command
   delay(5);
   Wire.beginTransmission(SDP810_I2C_ADDR); // transmit to device #37 (0x25)
   Wire.write(byte(0x00)); // msb
   Wire.write(byte(0x06)); // lsb command sensor Soft Reset (0x0006)
   Wire.endTransmission(); // stop transmitting
}
/******************************************************************************
Function name : void Setup_wifi()
         Type :
Description   : Setup wifimanager
Notes :
******************************************************************************/
void Setup_wifi()
{
  WiFi.mode(WIFI_STA);   
}
/******************************************************************************
Function name : void SendToServer (void)
         Type :
Description   :  
Notes : 
/*****************************************************************************/
 void SendToServer(){
   SendInterval = millis();
   float SendPressure = 0;
   float SendPaLow = 0;
   float SendPaHigh = 0; 
   float SendTemperatur = 0;
  // set the fields with the values
  ThingSpeak.setField(1, SendPressure = (round(FilteredPressure_1 * 100) / 100.00) );
  ThingSpeak.setField(2, SendPaLow = (round(pa_low * 100) / 100.00) );
  ThingSpeak.setField(3, SendPaHigh = (round(pa_high * 100) / 100.00) );
  ThingSpeak.setField(4, SendTemperatur = (round(FilteredTemperatur_1 * 100) / 100.00) );
  ThingSpeak.setField(5, WiFi.RSSI() );
  // set the status
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
   if(x == 200){
    Serial.println(String(x));
    LEDOnOffTime = 500;
    ResetCounter = 0;
  }
  else{
    Serial.println(String(x));
    LEDOnOffTime = 100;
  }
    Serial.println(SendPressure);
    Serial.println(SendPaLow);
    Serial.println(SendPaHigh);
 }
/******************************************************************************
Function name : void ConnectWiFi (void)
         Type :
Description   :  
Notes : 
/*****************************************************************************/
void ConnectWiFi(){
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID:  ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(1000);     
    } 
    Serial.println("\nConnected.");
    LEDOnOffTime       = 2000;
  }  
}
/******************************************************************************
Function name : void BlinkLED (void)
         Type :
Description   :  
Notes : 
/*****************************************************************************/
void BlinkLED(){
static unsigned long       LEDblinkInterval        = millis();

        if (millis() - LEDblinkInterval >= LEDOnOffTime) {
            digitalWrite(LED, !digitalRead(LED));
            LEDblinkInterval = millis();              
         }
}   
/*****************************************************************************/              