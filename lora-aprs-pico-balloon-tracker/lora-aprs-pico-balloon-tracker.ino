#include "SparkFun_Ublox_Arduino_Library.h" //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <MemoryFree.h>; //https://github.com/mpflaga/Arduino-MemoryFree
#include <Wire.h>
#include <LOLIN_HP303B.h> //https://github.com/hakkican/HP303B_Library
#include <RadioLib.h> //https://github.com/jgromes/RadioLib
#include <avr/dtostrf.h>

// SX1268 has the following connections:
#define nssPin 8
#define rstPin 9
#define dio1Pin 3
#define busyPin 2

#define BattPin A5
#define GpsPwr  12
#define GpsON  digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);

SFE_UBLOX_GPS myGPS;
LOLIN_HP303B hp303b;
SX1268 radio = new Module(nssPin, dio1Pin, rstPin, busyPin);

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//************************** General Settings ********************
String callSign ="N0CALL-11";
String symbolCode = "O"; //Balloon symbol
String symbolTable = "/";
String statusMessage = "LoRa APRS LightTracker by TA2MUN & TA2WX";
String aprsComment = "LoRa APRS LightTracker by TA2MUN & TA2WX";
const unsigned txInterval = 60000;  // Schedule TX every this many miliseconds.
boolean digipeaterMode = false; //This mode is experimental, works as a digi along with the tracker if enabled
float digiMinVolt=4.5; //Required minimum voltage to work as digipeater
//************************** LoRa APRS Default (Global) Settings ********************
float loraFrequency = 433.775;
float loraBandWith = 125.0f;
uint8_t spreadingFactor = 12; 
uint8_t codingRate = 5;
int8_t outputPower = 22;
uint16_t preambleLength = 8;
int8_t CRC = 1;
String header = "<\xff\x01";//Header for https://github.com/lora-aprs/LoRa_APRS_iGate compatibility
String wide ="WIDE1-1";
//************************** uBlox GPS  Settings ********************
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
boolean gpsFix=false; //do not change this.
boolean ublox_high_alt_mode_enabled = false; //do not change this.
boolean gpsSetup=false; //do not change this.
//********************************* Power Settings ******************************
int   battWait=60;    //seconds sleep if super capacitors/batteries are below battMin (important if power source is solar panel) 
float battMin=3.5;    // min Volts to TX. (Works with 3.3V too but 3.5V is safer) 
float gpsMinVolt=4.5; //min Volts for GPS to wake up. (important if power source is solar panel) //do not change this
//********************************* Misc Settings ******************************
uint16_t txCount = 1;
uint16_t digipeated_packet_count=0;
float voltage;
boolean packetQueued = false;
boolean  aliveStatus = true; //for tx status message on first wake-up just once.
uint32_t lastPacket = 0; //do not change this. Timestamp of last packet sent.

uint32_t GEOFENCE_APRS_frequency = 433775000; //temp frequency before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_APRS_spreading_factor = 12; //temp spreading factor before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_APRS_coding_rate = 5; //temp spreading factor before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_no_tx = 1;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void setup() {
  
  pinMode(A1,INPUT_PULLUP);
  if(digitalRead(A1)==LOW) while(1);

  delay(5000); //do not change this

  pinMode(GpsPwr, OUTPUT);    
  GpsOFF; 
  
  SerialUSB.begin(115200);
    
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB);

  SerialUSB.println();
  SerialUSB.println(F("Starting"));
  SerialUSB.println();
  freeMem();

  SerialUSB.print(F("APRS CallSign: "));
  SerialUSB.println(callSign);
  SerialUSB.println();

}

void loop() {

voltage = readBatt();

if (((voltage > battMin) && gpsFix) || ((voltage > gpsMinVolt) && !gpsFix)) {

  if(!gpsSetup){
    
    SerialUSB.println(F("GPS setup"));   
    setupGPS();
    SerialUSB.println(F("Searching for GPS fix...")); 
    gpsSetup = true;

    SerialUSB.println(F("LoRa setup"));
    setupLoRa();
    freeMem();
    
  }

  if (aliveStatus) {
    SerialUSB.println(F("Sending status message..."));
    sendStatusMessage();       
    aliveStatus = false;

    while (readBatt() < battMin) {
      delay(battWait * 1000);
    }
   
  }  


  // If txInterval passed send the next packet.
  if ((millis() - lastPacket > txInterval) || !gpsFix){
  
    GpsON;
    delay(500);

    if(!ublox_high_alt_mode_enabled){
      setupUBloxDynamicModel();
    }

     // Calling myGPS.getPVT() returns true if there actually is a fresh navigation solution available.

    if (myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 3)) {
      gpsFix=true;     

      if (txCount == 2 || txCount == 200) {
         configureFreqbyLocation();
      }

      //need to save power
      if (readBatt() < 4.5 || digipeaterMode) {
       GpsOFF;
       ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
       delay(500);      
      }
      
      if (GEOFENCE_no_tx != 1) {
          sendLocationMessage();
          lastPacket = millis();          
      }                 
      freeMem();
    }


      #if defined(DEVMODE)
        printGPSandSensorData();
      #endif    

  }

  if(receivedFlag) {
      // reset flag
      receivedFlag = false;
      //SerialUSB.println(F("[SX1268] Received packet!"));
      digiTX();             
    }

  //this code block protecting serial connected (3V + 3V) super caps from overcharging by powering on GPS module.
  //GPS module uses too much power while on, so if voltage is too high for supercaps, GPS ON.
  if (!digipeaterMode && readBatt() > 6.5) {
       GpsON;
       delay(500);
   }
  
  } else {

    GpsOFF;
    ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.     
    delay(battWait * 1000);
    
  }
  
  //delay(1000);
  
}

void sendAPRSMessage(String aprsMessage){
    radio.clearPacketReceivedAction();//Disable RX interrupt.
    receivedFlag = false;
    SerialUSB.println(aprsMessage);
    int state = radio.transmit(aprsMessage);
    if (state == RADIOLIB_ERR_NONE) {
      // the packet was successfully transmitted
      SerialUSB.println(F("The packet was successfully transmitted..."));
  
      // print measured data rate
      SerialUSB.print(F("[SX1268] Datarate:\t"));
      SerialUSB.print(radio.getDataRate());
      SerialUSB.println(F(" bps"));
  
    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
      // the supplied packet was longer than 256 bytes
      SerialUSB.println(F("The supplied packet was longer than 256 bytes!"));
  
    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
      // timeout occured while transmitting packet
      SerialUSB.println(F("Timeout occured while transmitting packet!"));
  
    } else {
      // some other error occurred
      SerialUSB.print(F("failed, code "));
      SerialUSB.println(state);
  
    }
  }  

void sendLocationMessage(){
    String aprsMessage = header + getTrackerLocationAPRSMessage();
    sendAPRSMessage(aprsMessage);
    txCount++;
    //Enable RX for digipeating if possible.
    enableRX();
  }

String getTrackerLocationAPRSMessage() {
  
  String message;

  //Latitude and Longitude
  String path = "";
  if (wide != "") {
      path = "," + wide;
  }
  message += callSign +">APLIGP"+path+":/";

  String timestamp = encodeHMSTimestamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); //Zulu time DHM timestamp
  message += timestamp;

  String latitude = createLatForAPRS(myGPS.getLatitude() / 10000000.f);
  String longitude = createLongForAPRS(myGPS.getLongitude() / 10000000.f);
  message += latitude + symbolTable+longitude+symbolCode;

  //Course and Speed
  char course_speed_buff[10];

  uint16_t heading = (uint16_t)(myGPS.getHeading() / 100000);
  uint16_t speed = (uint16_t)(myGPS.getGroundSpeed() * 0.00194384f);
  double altitude = (double)(myGPS.getAltitude() * 3.2808399d)  / 1000.d;
  uint16_t sattelite = (uint16_t)(myGPS.getSIV());
  sprintf(course_speed_buff, "%03d", heading);
  course_speed_buff[3] += '/';
  sprintf(course_speed_buff + 4, "%03d", speed);

  String course_speed_str;
  for (uint8_t i = 0; i <7; i++)
  {
    course_speed_str += String((char)course_speed_buff[i]);
  }
  message += course_speed_str;

  //Altitude
  String alt_str="";
  char alt_buff[10]; 
  alt_buff[0] = '/';
  alt_buff[1] = 'A';
  alt_buff[2] = '=';

  if (altitude > 0) {
    //for positive values
    sprintf(alt_buff + 3, "%06lu", (long)altitude);
  } else {
    //for negative values
    sprintf(alt_buff + 3, "%06ld", (long)altitude);
  }
  for (uint8_t i = 0; i <9; i++)
  {
    alt_str += String((char)alt_buff[i]);
  }

  message += alt_str;

  //TXCount, Temperature, Pressure and Voltage

  String txCountComment = " "+String(txCount)+"TXC";
  String digiTXCountComment = " DigiC:"+String(digipeated_packet_count);

  int16_t oversampling = 7;
  int32_t temperature;
  int32_t pressure;
  hp303b.measureTempOnce(temperature, oversampling);   
  hp303b.measurePressureOnce(pressure, oversampling);

  String temperatureComment = " "+String(temperature)+"C";
  
  String pressureComment; 
  String pressure_str = "";
  float pressureF = pressure / 100.0f; //Pa to hPa
  char pressure_buff[6];
  dtostrf(pressureF, 7, 2, pressure_buff);     
  for (uint8_t i = 0; i < 7; i++)
  {
    pressure_str += String((char)pressure_buff[i]);
  }
  pressureComment = " "+pressure_str+"hPa";

  String voltageComment;
  char voltage_buff[3];
  dtostrf(voltage, 3, 2, voltage_buff);

  String voltage_str;
  for (uint8_t i = 0; i < 3; i++)
  {
    voltage_str += String((char)voltage_buff[i]);
  }    
    voltageComment =  " "+voltage_str+"V";

  String satComment = " "+String(sattelite)+"S ";
    
  if (digipeaterMode) {
      message += txCountComment + digiTXCountComment + temperatureComment + pressureComment + voltageComment + satComment + aprsComment;
    } else {
        message += txCountComment + temperatureComment + pressureComment + voltageComment + satComment + aprsComment;
      }
     
  return message;
}

void sendStatusMessage(){
    String aprsMessage = header + getTrackerStatusAPRSMessage();
    sendAPRSMessage(aprsMessage);
    txCount++;
  }

String getTrackerStatusAPRSMessage(){
  
  String message;
  String path = "";
  if (wide != "") {
      path = "," + wide;
  }

  message += callSign +">APLIGP"+path+":>"+statusMessage;

  return message;
}

void setupGPS() {
  GpsON;
  delay(100);
  Wire.begin();
  hp303b.begin(0x76);

  Wire.setClock(400000);

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //myGPS.enableDebugging(); //Enable debug messages over Serial (default)

  myGPS.setNavigationFrequency(2);//Set output to 2 times a second. Max is 10
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  SerialUSB.print("Current update rate for GPS: ");
  SerialUSB.println(rate);

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  
  }

void digiTX(){

      String packet;
      int state = radio.readData(packet);
          
      if (state == RADIOLIB_ERR_NONE) {
               
        if(packet.length() >0) {
          // packet was successfully received
          
          String aprsMessage;
          
          if ((packet.substring(0, 3) == "\x3c\xff\x01") && (packet.indexOf("TCPIP") == -1) && (packet.indexOf("NOGATE") == -1)) {
            String sender = packet.substring(3,packet.indexOf(">"));
            if ((packet.indexOf("WIDE1-1") > 10) && (callSign != sender)) {
              aprsMessage = packet.substring(3);                         
              aprsMessage.replace("WIDE1-1", callSign + "*");
              SerialUSB.print(F("Digipeating: "));
              sendAPRSMessage(header + aprsMessage);
              digipeated_packet_count++;
            }
          } else {
            SerialUSB.println(packet);
            SerialUSB.println("LoRa Packet Ignored (first 3 bytes or TCPIP/NOGATE/RFONLY)");
          }            
         }
        
      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        SerialUSB.println(F("CRC error!"));
      } else {
        // some other error occurred
        SerialUSB.print(F("failed, code "));
        SerialUSB.println(state);
      }
  
  }

void enableRX(){
    if (digipeaterMode && gpsFix && voltage > digiMinVolt && GEOFENCE_no_tx != 1) {
          // set the function that will be called
          // when new packet is received
          radio.setPacketReceivedAction(setFlag);

          SerialUSB.print(F("[SX1268] Starting to listen ... "));
          int state = radio.startReceive();
          if (state == RADIOLIB_ERR_NONE) {
            SerialUSB.println(F("success!"));
          } else {
            SerialUSB.print(F("failed, code "));
            SerialUSB.println(state);
            //while (true);
          }          
      }
  }

void printGPSandSensorData()
{
 
    lastTime = millis(); //Update the timer

    byte fixType = myGPS.getFixType();

    SerialUSB.print(F("FixType: "));
    SerialUSB.print(fixType);    

    int SIV = myGPS.getSIV();
    SerialUSB.print(F(" Sats: "));
    SerialUSB.print(SIV);

    float flat = myGPS.getLatitude() / 10000000.f;
    
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(flat);    

    float flong = myGPS.getLongitude() / 10000000.f;    
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(flong);        

    float altitude = myGPS.getAltitude() / 1000;
    SerialUSB.print(F(" Alt: "));
    SerialUSB.print(altitude);
    SerialUSB.print(F(" (m)"));

    //float speed = myGPS.getGroundSpeed() * 0.0036f;
    //SerialUSB.print(F(" Speed: "));
    //SerialUSB.print(speed);
    //SerialUSB.print(F(" (km/h)"));

    //long heading = myGPS.getHeading() / 100000;
    //SerialUSB.print(F(" Heading: "));
    //SerialUSB.print(heading);
    //SerialUSB.print(F(" (degrees)"));
        
    SerialUSB.print(" Time: ");    
    SerialUSB.print(myGPS.getYear());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getMonth());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getDay());
    SerialUSB.print(" ");
    SerialUSB.print(myGPS.getHour());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getMinute());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getSecond());
    
    int16_t oversampling = 7;
    int16_t ret;
    int32_t temperature;
    int32_t pressure;
    ret = hp303b.measureTempOnce(temperature, oversampling);
    if(ret !=0){
         SerialUSB.print("hp303b fail! ret = ");
         SerialUSB.println(ret);
      }
    
    SerialUSB.print(" Temp: ");
    SerialUSB.print(temperature);
    SerialUSB.print(" C");  

    ret = hp303b.measurePressureOnce(pressure, oversampling);
    if(ret !=0){
         SerialUSB.print("hp303b fail! ret = ");
         SerialUSB.println(ret);
      }
       
    SerialUSB.print(" Press: ");    
    SerialUSB.print(pressure / 100.0);
    SerialUSB.print(" hPa");

    SerialUSB.println();
    delay(500);
}  

float readBatt() {

  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;

  do {    
    value =analogRead(BattPin);
    value +=analogRead(BattPin);
    value +=analogRead(BattPin);
    value = value / 3.0f;
    value = (value * 3.3) / 1024.0f;
    value = value / (R2/(R1+R2));
  } while (value > 20.0);
  return value ;

}

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif

}

void setupUBloxDynamicModel() {
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    //DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters. 
    if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
      SerialUSB.println(F("***!!! Warning: setDynamicModel failed !!!***"));
    }
    else
    {
      ublox_high_alt_mode_enabled = true;
      #if defined(DEVMODE)
        SerialUSB.print(F("Ublox dynamic platform model (DYN_MODEL_AIRBORNE4g) changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
      #endif  
    }
  
  } 

 void setupLoRa() {  

  // initialize SX1262 with default settings
  SerialUSB.print(F("[SX1268] LoRa Radio Module Initializing ... "));  
  int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true);
  }

  if (radio.setFrequency(loraFrequency,true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    SerialUSB.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setBandwidth(loraBandWith) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    SerialUSB.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(spreadingFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    SerialUSB.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(codingRate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    SerialUSB.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  if (radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE) != RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("Unable to set sync word!"));
    while (true);
  }

  // set over current protection limit to 140 mA (accepted range is 45 - 240 mA)
  if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
    SerialUSB.println(F("Selected current limit is invalid for this module!"));
    while (true);
  }   

  // set output power (accepted range is -17, 22 dBm)
  if (radio.setOutputPower(outputPower) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    SerialUSB.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  // set LoRa preamble length (accepted range is 0 - 65535)
  if (radio.setPreambleLength(preambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
    SerialUSB.println(F("Selected preamble length is invalid for this module!"));
    while (true);
  }

  // enable CRC
  if (radio.setCRC(CRC) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    SerialUSB.println(F("Selected CRC is invalid for this module!"));
    while (true);
  }

  radio.setRxBoostedGainMode(true);

  SerialUSB.println(F("All settings succesfully changed!"));  

 }

 String encodeHMSTimestamp(int hour, int minute, int second){

  char timestamp_buff[8];

  sprintf(timestamp_buff, "%02d", hour);
  sprintf(timestamp_buff + 2, "%02d", minute);
  sprintf(timestamp_buff + 4, "%02d", second);
  timestamp_buff[6] = 'h';

  String timestamp;
      for (uint8_t i = 0; i < 7; i++)
      {
        timestamp += String((char)timestamp_buff[i]);
      }  

  return timestamp;
}

String createLatForAPRS(double latitude) {

  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char lat_buff[10];
  int temp = 0;
  double dm_lat = 0.0;

  if (latitude < 0.0) {
    temp = -(int)latitude;
    dm_lat = temp * 100.0 - (latitude + temp) * 60.0;
  } else {
    temp = (int)latitude;
    dm_lat = temp * 100 + (latitude - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, lat_buff);

  if (dm_lat < 1000) {
    lat_buff[0] = '0';
  }

  if (latitude >= 0.0) {
    lat_buff[7] = 'N';
  } else {
    lat_buff[7] = 'S';
  }

  String latStr;
      for (uint8_t i = 0; i < 8; i++)
      {
        latStr += String((char)lat_buff[i]);
      }  

  return latStr;

}

String createLongForAPRS(double longitude) {

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char long_buff[10];
  int temp = 0;
  double dm_lon = 0.0;

  if (longitude < 0.0) {
    temp = -(int)longitude;
    dm_lon = temp * 100.0 - (longitude + temp) * 60.0;
  } else {
    temp = (int)longitude;
    dm_lon = temp * 100 + (longitude - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, long_buff);

  if (dm_lon < 10000) {
    long_buff[0] = '0';
  }
  if (dm_lon < 1000) {
    long_buff[1] = '0';
  }

  if (longitude >= 0.0) {
    long_buff[8] = 'E';
  } else {
    long_buff[8] = 'W';
  }

  String longStr;
      for (uint8_t i = 0; i < 9; i++)
      {
        longStr += String((char)long_buff[i]);
      }  

  return longStr;
}

void configureFreqbyLocation() {

  float tempLat = myGPS.getLatitude() / 10000000.f;
  float tempLong = myGPS.getLongitude() / 10000000.f; 

  GEOFENCE_position(tempLat,tempLong);  
  loraFrequency = GEOFENCE_APRS_frequency / 1000000.f;
  if (radio.setFrequency(loraFrequency,true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    SerialUSB.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(GEOFENCE_APRS_spreading_factor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    SerialUSB.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(GEOFENCE_APRS_coding_rate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    SerialUSB.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

}

int32_t pointInPolygonF(int32_t polyCorners, float * polygon, float latitude, float longitude)
{
  int32_t i;
  int32_t j = polyCorners * 2 - 2;
  int32_t oddNodes = 0;

  for(i = 0; i < polyCorners * 2; i += 2)
  {
    if((polygon[i + 1] < latitude && polygon[j + 1] >= latitude
    || polygon[j + 1] < latitude && polygon[i + 1] >= latitude)
    && (polygon[i] <= longitude || polygon[j] <= longitude))
    {
      oddNodes ^= (polygon[i] + (latitude - polygon[i + 1])
      / (polygon[j + 1] - polygon[i + 1]) * (polygon[j] - polygon[i]) < longitude);
    }

    j = i;
  }

  return oddNodes;
}

void GEOFENCE_position(float latitude, float longitude)
{

      float  UKF[] = {
        -0.65920,   60.97310,
        -7.58060,   58.07790,
        -8.21780,   54.23960,
        -4.76810,   53.80070,
        -5.86670,   49.76710,
        1.30740,    50.85450,
        1.86770,    52.78950,
        -2.04350,   55.97380,
        -0.65920,   60.97310
      }; 
  
      float  PolandF[] = {
        54.439234, 14.103456,
        51.022075, 14.897034,
        49.537732, 18.860178,
        49.113780, 22.481393,
        52.742317, 23.889542,
        54.156275, 23.456098,
        54.439234, 14.103456
      };      
      static float LatviaF[] = {
        26.64180,   55.68380,
        28.17990,   56.20670,
        27.78440,   57.33250,
        25.00490,   58.00230,
        24.14790,   57.17200,
        21.78590,   57.68650,
        20.81910,   56.07200,
        22.19240,   56.44430,
        25.68600,   56.18230,
        26.64180,   55.68380
      };      
      static float YemenF[] = {
        52.20302, 19.48287,
        41.92009, 17.42198,
        43.59620, 12.27820,
        53.68201, 15.80024,
        52.20302, 19.48287
      };
      static float North_KoreaF[] = {
        130.14189, 43.21627,
        124.03574, 39.8949,
        125.18292, 37.39202,
        128.47240, 38.66888,
        130.69478, 42.3274,
        130.14189, 43.21627
      };             

      if(pointInPolygonF(9, UKF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1; //Airborne TX is not allowed in UK
          GEOFENCE_APRS_frequency = 439912500;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;
        } else if(pointInPolygonF(7, PolandF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 0;
          GEOFENCE_APRS_frequency = 434855000;
          GEOFENCE_APRS_spreading_factor = 9;
          GEOFENCE_APRS_coding_rate = 7;          
        } else if(pointInPolygonF(10, LatviaF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;                
        } else if(pointInPolygonF(5, YemenF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5; 
        } else if(pointInPolygonF(6, North_KoreaF, latitude, longitude) == 1){
          GEOFENCE_no_tx = 1;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;              
        } else {
          GEOFENCE_no_tx = 0;
          GEOFENCE_APRS_frequency = 433775000;
          GEOFENCE_APRS_spreading_factor = 12;
          GEOFENCE_APRS_coding_rate = 5;      
        }
}
