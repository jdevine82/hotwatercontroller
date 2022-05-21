/*This is for a modified grid feed inverter. Based around a mega with an in built esp8266.
   Uses fast pwm to perform MPPT of the solar panels. It also uses a second much slower switching output to provide a slow
   100hz Switched DC output of the hot water drive. This allows the mechanical overtemp switchs to work.
   Via the ESP it will connect with MQTT. It has a relay board connected to control 2 motorised ball valves to connect either the tank or the instant hot water.
   It also has a flow sensor input to measure the amoout of hot water being consumed.
   It takes a temperature input from the top of the tank outlet to decide when to turn over to gas.
   It takes a temperature reading from the bottom of the tank to decide when to turn off due to full temp.
   The MPPT algorithm uses the input current and voltage sensing cirtcuitry to calculate the mppt point.


*/
#include "OneWire.h"
#include "DallasTemperature.h"
#include "ELClient.h"
#include "ELClientCmd.h"
#include "ELClientMqtt.h"
#define heatSinkTemp  A15  //sensor for the internal heatsink only.
#define heatSinkTempScale 0.16  //dont touch calibrated
#define heatSinkTempOffset 74.0 //dont touch calibrated.
#define heatSinkSetpoint  65.0  //high temp shutdown setpoint in degrees C.
#define current  A2 //current sensor for solar input.
#define currentOffset 536  //516
#define currentScale 0.177
#define currentFilterSize 5
#define voltageFilterSize 5
#define voltage A1  //Voltage input for solar input.
#define voltageOffset 30.0
#define voltageScale 2.3
#define minVolts 120  //too low and power supply wont run.
#define dutyPot  A0 //for testing only.
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 37
#define TEMPERATURE_PRECISION 9
#define gasManualSwitch 53
#define tankManualSwitch 52

#define tankSetpoint 72.0
#define tankHystersis 0.5
#define tankCutout 40.0
#define tankCutoutHystersis 3.0

#define mpptDrive  12 //high side output of MPPT drive; timer1 a channel
#define diodeDrive 11
#define diodeTime 1 //small time to bootstrap capacitor.
#define tankHighDrive  7
#define tankLowDrive  6
#define gasHighDrive  5
#define gasLowDrive 4
#define tankHighDriveLimit 25
#define tankLowDriveLimit 24
#define gasHighDriveLimit 27
#define gasLowDriveLimit 26
#define deadTime  8 //0-380;
#define  deadTimeHalf deadTime/2
#define   fulldrive 999
#define pulseDrive  2 //high side drive for Pulsed DC. also timer3 a channelB
#define lowDrive  3 //low side drive for Pulsed DC. timer 3 channel C
#define flowSensor  47 //this is the input for the flow sensor. This can be setup for external counting with interrupts on timer 5;
int     voltsTotal, ampsTotal;
unsigned long publishtimer = 0; //for mqtt
unsigned long measureTimer = 0; //for mppt
unsigned long valveTimer, dallasTimer;
unsigned long debugTimer;
#define valveDriveTime 8000 //5 seconds to drive valve open or close.
#define measureInterval 800 //500ms update period for mppt.
#define trackingError 10/100   //% of watts error
unsigned int lasttime,bottomTempErrorCounter,topTempErrorCounter;
int inputDuty, duty,i,topicNumber;
char dutymqtt[4];
char voltsmqtt[4];
char ampsmqtt[4];
char wattsmqtt[4];
char bottomTempmqtt[4];
char topTempmqtt[4];
char Litersmqtt[6];
#define bottomTempErrorLimit 20
#define topTempErrorLimit 20

float amps, volts, watts, previouswatts, bottomTemp, topTemp;
boolean slope, highTempShutdown, tankHot, gasOn, hotWaterEnable, arcQuench,inputOverride,overRide,gasEnable,solarEnable,tankValveState,gasValveState,tankOverRide;
float Liters;
#define flowGain 0.001776///0.02222  //Liters per pulse.
#define prefix "/hotwater/"
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress bottomThermometer, topThermometer;
// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
ELClient esp(&Serial3);

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

ISR(TIMER3_COMPB_vect) {
  if (arcQuench) {
    //  pinMode(mpptDrive, OUTPUT); //this is needed for pwm output
    //pinMode(diodeDrive, OUTPUT); //this is needed for pwm output
    TCCR1A = (1 << COM1A1) ;
    arcQuench = !arcQuench;
    //  OCR3C = 10020;  //this sets the duty cycle permanently to 50%
    OCR3B = 18000;
  }
  else {
    //   pinMode(mpptDrive, INPUT); //this is needed for pwm output
    // pinMode(diodeDrive, INPUT); //this is needed for pwm output
    TCCR1A = (0 << COM1A1) ;
    arcQuench = !arcQuench;
    //  OCR3C = 10020;  //this sets the duty cycle permanently to 50%
    OCR3B = 7000;
  }

}

void debugSerial(void)
{
  
  Serial.print(" Bottom Temp: ");
  Serial.print(bottomTemp);
  Serial.print(" Top Temp: ");
  Serial.print(topTemp);
    Serial.print("current :");
  Serial.print(amps);
  Serial.print(" Volts :");
  Serial.print(volts);
   Serial.print(" Watts :");
  Serial.print(watts);
     Serial.print(" slope: ");
  Serial.print(slope);
  Serial.print(" Duty: ");
  Serial.print(duty);
  Serial.print(" Override: ");
  Serial.print(overRide);
  Serial.print("gasEnable");
  Serial.print(gasEnable);
  Serial.print("solarEnable ");
  Serial.println(solarEnable);
  
}

void wifiCb(void* response)
{
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);
    if (status == STATION_CONNECTING) Serial.println("wifstation_connecting");
    if (status == STATION_WRONG_PASSWORD) Serial.println("wifI_WRONG_PASSWORD");
    if (status == STATION_NO_AP_FOUND) Serial.println("wifINO_ap_FOUND");
    if (status == STATION_CONNECT_FAIL) Serial.println("wifICONNECT_FAIL");
  }
}

void mqttConnected(void* response)
{
 // mqtt.subscribe(prefix"dutyin");
  mqtt.subscribe(prefix"litersReset");
  mqtt.subscribe(prefix"hotWaterEnable");
  mqtt.subscribe(prefix"overRide");
  mqtt.subscribe(prefix"tankOverRide");

 
  //  mqtt.subscribe(prefix"WindShutdown");
 // Serial.println("ARDUINO: connected to mqtt");
} //end mqttConnected

void mqttDisconnected(void* response)
{
  Serial.println("ARDUINO: fail to setup mqtt");
}// end mqttDisconnected

void mqttData(void* response)
{
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial.print("data=");
  String data = res->popString();
  Serial.println(data);

  if (topic.equalsIgnoreCase(prefix"dutyin")) {
    int x = data.toInt();
    inputDuty = constrain(x, 0, 100);
   // Serial.println(inputDuty);
    if (inputDuty>10) {
      inputOverride=true;
      duty=inputDuty;
    }
    else inputOverride=false;
  }

  if (topic.equalsIgnoreCase(prefix"litersReset")) {
    
    if (data == "1") {
      TCNT5 = 0;
      Liters = 0;
    }
  }

    if (topic.equalsIgnoreCase(prefix"gasEnable")) {
    if (data == "1") {
    gasEnable=true;
    }
    if (data == "0") {
    gasEnable=false;
    }
    
  }

    if (topic.equalsIgnoreCase(prefix"solarEnable")) {
    if (data == "1") {
      solarEnable=true;
    
    }
    if (data == "0") {
      solarEnable=false;
    
    }
  }

      if (topic.equalsIgnoreCase(prefix"overRide")) {
    if (data == "1") {
    overRide=true;
    }
    else overRide=false;
  }
  if (topic.equalsIgnoreCase(prefix"tankOverRide")){
    if (data =="1"){
      tankOverRide=true;
    }
    else tankOverRide=false;
  
  }
  if (topic.equalsIgnoreCase(prefix"hotWaterEnable")) {
    if (data == "1") {
      hotWaterEnable = true;
    }
    else {
      hotWaterEnable = false;
    }
  }
} //end mqttData

void mqttPublished(void* response)
{

}

void publishstuff (int y)

{
  topicNumber=8; //change this for each topic addeed.
  switch (i){
  case 0: {
  dtostrf(duty, 3, 0, dutymqtt);
  mqtt.publish(prefix"Duty", dutymqtt);}
  break;
  case 1:{
  dtostrf(volts, 3, 2, voltsmqtt);
  mqtt.publish(prefix"Volts", voltsmqtt);}
  break;
  case 2: {
  dtostrf(amps, 3, 0, ampsmqtt);
  mqtt.publish(prefix"Amps", ampsmqtt);}
  break;
  case 3: {
  dtostrf(watts, 3, 0, wattsmqtt);
  mqtt.publish(prefix"Watts", wattsmqtt);}
  break;
  case 4: {
  dtostrf(Liters, 3, 0, Litersmqtt);
  mqtt.publish(prefix"Liters", Litersmqtt);}
  break;
  case 5 : {
  dtostrf(bottomTemp, 3, 0, bottomTempmqtt);
  mqtt.publish(prefix"bottomTemp", bottomTempmqtt);}
  break;
  case 6 :{
  dtostrf(topTemp, 3, 0, topTempmqtt);
  mqtt.publish(prefix"topTemp", topTempmqtt);}
  break;
  case 7: {
    if (tankValveState) {mqtt.publish(prefix"tankValveState","ON");} else {mqtt.publish(prefix"tankValveState","OFF");}
  }
  break;
  case 8: {
     if (gasValveState) {mqtt.publish(prefix"gasValveState","ON");} else {mqtt.publish(prefix"gasValveState","OFF");}
  break;
  }

  default : i=0;
  break;}
  publishtimer = millis();
}//end publishstuff

void sendDuty (int inDuty)
//this simply takes a duty of 0-100 and converts it the output drive module for the pwm. This keeps the deadtime of around 480ns.
//also need to shutdown if temp is too high.
{
  if (highTempShutdown) inDuty = 0;

  if (tankHot) inDuty = 0;
  int val = map(inDuty, 0, 100, 1, fulldrive); // max value for pwm. cant go to 100% as bootstrap will not work. takes inputDuty as a 0-100% input
  //if (val>=(deadTimeHalf+1)){
  //OCR1A=val;-//deadTimeHalf;
  OCR1A = val; //+deadTimeHalf;
  //}
  //else {
  // OCR1A=deadTimeHalf;
  // OCR1B=5+deadTimeHalf;
  //  } //end else
}

float readVolts (void)
//read voltage input
{
  int x = analogRead(voltage);
  voltsTotal = voltsTotal + x;
  int y = voltsTotal / (voltageFilterSize+1);
  voltsTotal=voltsTotal-y;
  float z= (((float)y - voltageOffset) * voltageScale); //will need to adjust this for the actual values.
  if (z<0.0) z=0.0;
  return(z);
}

float readAmps (void)
//read current input
{
  int x = analogRead(current);

  ampsTotal = ampsTotal + x;
  int y = ampsTotal / (currentFilterSize+1);
    ampsTotal = ampsTotal - y;


  float z = (((float)y - currentOffset) * currentScale);
 if (z<0.0) z=0.0; //constrain
 //Serial.println(z);
  return (z); 
}


void openTank (void)
{
 // return;  //only for testing with no valves.
  valveTimer = millis(); //time valve opening/closing
  /* while timing the opening/closing of the valve to make sure its time is not exceded we watch for the limit switch*/
  while ((digitalRead(tankLowDriveLimit)) && (millis() < valveTimer + valveDriveTime))
  {
    digitalWrite(tankHighDrive, 1);
    digitalWrite(tankLowDrive, 0);
  }//end while drive loop
  digitalWrite(tankHighDrive, LOW); //turn off.
  if (digitalRead(tankLowDriveLimit)==0) tankValveState=1; //used to feed mqtt state.
}

void closeTank (void)
{
  //return;  //only for testing with no valves.
  valveTimer = millis(); //time valve opening/closing
  /* while timing the opening/closing of the valve to make sure its time is not exceded we watch for the limit switch*/
  while ((digitalRead(tankHighDriveLimit)) && (millis() < valveTimer + valveDriveTime))
  {
    digitalWrite(tankHighDrive, 0);
    digitalWrite(tankLowDrive, 1);
  }//end while drive loop
  digitalWrite(tankLowDrive, LOW); //turn off.
    if (digitalRead(tankHighDriveLimit)==0) tankValveState=0; //used to feed mqtt state.
}


void closeGas (void)
{
  //return;  //only for testing with no valves.
  valveTimer = millis(); //time valve opening/closing
  /* while timing the opening/closing of the valve to make sure its time is not exceded we watch for the limit switch*/
  while ((digitalRead(gasHighDriveLimit)) && (millis() < valveTimer + valveDriveTime))
  {
    digitalWrite(gasHighDrive, 1);
    digitalWrite(gasLowDrive, 0);
  }//end while drive loop
  digitalWrite(gasHighDrive, LOW); //turn off.
  if (digitalRead(gasHighDriveLimit)==0) gasValveState=0; //used to feed mqtt state.
}

void openGas (void)
{
 
// return;  //only for testing with no valves.
  valveTimer = millis(); //time valve opening/closing
  /* while timing the opening/closing of the valve to make sure its time is not exceded we watch for the limit switch*/
  while ((digitalRead(gasLowDriveLimit)) && (millis() < valveTimer + valveDriveTime))
  {
    digitalWrite(gasHighDrive, 0);
    digitalWrite(gasLowDrive, 1);
  }//end while drive loop
  digitalWrite(gasLowDrive, LOW); //turn off.
  if (digitalRead(gasLowDriveLimit)==0) gasValveState=1; //used to feed mqtt state
}

float readHeatSinkTemp (void)
{
  return (((float)analogRead(heatSinkTemp) - heatSinkTempOffset) * heatSinkTempScale);
}

void mpptRoutine (void)
{
  volts = readVolts();
  amps = readAmps();
  watts = amps * volts;

if (!inputOverride){
  /**this is the mppt tracking part******/
  if (watts > previouswatts) {
    //ok slope is correct so leave slope alone.
    if (slope)
    {
      if (duty < 100) duty++;
      if (volts < minVolts) slope = !slope; //change direction of voltage is too low.

    }
    else {
      if (duty > 15) duty--;
    }
  }

  if (watts < previouswatts) {
    //ok so going wrong way with power now so change slope.
     float result=watts-previouswatts;
  if (result<0.0) result=result*-1.0; //make positive.
    float trackingErrorPercent=watts*(float)trackingError;
    if (result>trackingErrorPercent) slope = !slope;
    if (slope)
    {
      if (duty < 100) duty++;
      //if (volts<minVolts) slope=!slope; //change direction of voltage if too low.
    }
    else
    {
      if (duty > 15) duty--;
    }
  }
 }//end inputDuty<10;

  previouswatts = watts; //save for next time.
  /*************************************/

  /*for testing*************/
 int val2 = analogRead(dutyPot);  //this is just for testing
  val2=constrain(val2,0,1000);
  if (val2<980) duty=map(val2,1000,0,0,100);  //just to enable a pot for testign to override the mqtt data.
  /*****************/
  duty = constrain(duty, 0, 100);

  //ok lets adjust the output to suit.
  sendDuty(duty);
} //end mppt routine

// function to print the temperature for a device
float getTemperature(DeviceAddress deviceAddress)
{
  
  //return(65.0);
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data from ds18b. Heating disabled.");
    return 120.0;  //return a really high temp that will drive the heating element off if this is returned continually.
  }

  return (tempC);
}


void measureTemps (void)
{
  sensors.requestTemperatures();
  float x = getTemperature(bottomThermometer);
 float y = getTemperature(topThermometer);
  if (x==120.0) {
      bottomTempErrorCounter++;
      if (bottomTempErrorCounter>bottomTempErrorLimit) bottomTemp=120.0;
  }

   if (y==120.0) {
      topTempErrorCounter++;
      if (topTempErrorCounter>topTempErrorLimit) topTemp=120.0;
 
  }

   if (x<120.0) {bottomTemp=x; bottomTempErrorCounter=0;}
   if (y<120.0) {topTemp=y; topTempErrorCounter=0;}

//  bottomTemp = 10.0; //testing only
 // topTemp = 65.0; //testing only

  //here we will will do some logic with the temperature data. hotWaterEnable allows mqtt control of valves.
  if (bottomTemp > tankSetpoint) tankHot = true;
  if (bottomTemp < (tankSetpoint - tankHystersis)) tankHot = false;
  if (topTemp < (tankCutout-tankCutoutHystersis)) gasOn = true;;
  if (topTemp > (tankCutout + tankCutoutHystersis)) gasOn = false;

  /*this section allows overriding control via mqtt of valves.*/
  if (overRide==true ) gasOn=true; //this will force the operation of the gas if set.
  if (tankOverRide==true) gasOn=false; //this will force operation of tank from mqtt.
  //check manual switch for override;
  if (digitalRead(gasManualSwitch)==0) gasOn=true;
  if (digitalRead(tankManualSwitch)==0) gasOn=false;
  
  if (gasOn)
  {
   if (hotWaterEnable) openGas(); else closeGas();
    closeTank();
  }//end if gasOn

  else
  {
   if (hotWaterEnable) openTank(); else closeTank();
    closeGas();
  }//end else if gasON
}//end measureTemps
void setup() {

  // put your setup code here, to run once:
  Serial3.begin(115200); //esplink
  Serial.begin(115200); //for debugging
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
   pinMode(gasManualSwitch,INPUT_PULLUP);
   pinMode(tankManualSwitch,INPUT_PULLUP);

  // initialize timer1
  noInterrupts();           // disable all interrupts
  /***setup high frequency drive*****/
  TCNT1  = 0;
  ICR1 = 1000;            // compare match register this gives a frequency of about 21khz as a phase matched output.
  TCCR1A = 176;          //options for timer 1
  TCCR1B = 17;        //options for timer 1
  //  TIMSK1 |= (1<<OCIE1A); //interrupt flag.
  //OCR1A= duty 0-380max
  //pinMode(mpptDrive, OUTPUT); //this is needed for pwm output
  pinMode(diodeDrive, OUTPUT); //this is needed for pwm output
  /***Setup Low freqency drive*****/

  TCNT3  = 0;
  ICR3 = 20000;           // compare match register
  TCCR3C = 0;
  TCCR3B = 0;
  TCCR3B = (1 << WGM33) | (1 << CS31) | (1 << WGM32) | (1 << WGM33) | (1 << WGM32);
  TCCR3A = (1 << WGM31);
  //TCCR3A = 44;          //options for timer 3
  //TCCR3B = 18;
  OCR3B = 10020;  //this sets the duty cycle permanently to 50%

  // OCR3C = 200; //small drive to make sure bootstrap gets properly charged.
  TIMSK3 |= (1 << OCIE3B); //interrupt flag.
  // pinMode(pulseDrive, OUTPUT); //
  // pinMode(lowDrive, OUTPUT); //low side
  /***setup external counter for flow sensor****/
  TCNT5 = 0; //reset counter to 0 as the flow sensor runs up the value can be read from this register for what has passed in a period.
  TCCR5B = 135; //external trigger for counter rising edge.
  TCCR5A = 0;
  interrupts();             // enable all interrupts
  pinMode(tankHighDrive, OUTPUT);
  pinMode(tankLowDrive, OUTPUT);
  pinMode(gasLowDrive, OUTPUT);
  pinMode(gasHighDrive, OUTPUT);
  pinMode(tankHighDriveLimit, INPUT_PULLUP);
  pinMode(tankLowDriveLimit, INPUT_PULLUP);
  pinMode(gasHighDriveLimit, INPUT_PULLUP);
  pinMode(gasLowDriveLimit, INPUT_PULLUP);
  pinMode(heatSinkTemp, INPUT);
  // Start up the ds18b library
  sensors.begin();
  if (!sensors.getAddress(bottomThermometer, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(topThermometer, 1)) Serial.println("Unable to find address for Device 1");
  // set the resolution to 9 bit per device
  sensors.setResolution(bottomThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(topThermometer, TEMPERATURE_PRECISION);
  hotWaterEnable = true;
  duty=40;
  ampsTotal=currentFilterSize*520;
}

void loop() {

  //see if time elapsed to do mppt check
  if (measureTimer + (measureInterval) < millis()) {
    mpptRoutine();
    measureTimer = millis();
  }



  /*this section deals with the temperaures of both the heatsink and the hotwater. It will control the high frequency switch and the motorised valves.*/

  if ((millis()) > (dallasTimer + 2000)) {  //measure temps every 10 second
    measureTemps();
    dallasTimer = millis();
  }


 if ((millis()) > (debugTimer + 2000)) {  
    debugSerial();
   debugTimer = millis();
  }

  //check temp of device and shutdowm if too hot
  if (readHeatSinkTemp() > heatSinkSetpoint) highTempShutdown = 1; else highTempShutdown = 0;


  /**this section is to take care of adding up how many liters have passed thorough the water flow sensor. Only needs to be called periodically. Hardware counter keeps track.
     The counter will be reset if the micro is reset or if a mqqt command comes to reset.

  */
  Liters = Liters + TCNT5 * flowGain;
  TCNT5 = 0; //reset hardware counter


  /*
     This section deals with making sure the mqtt comms happens.


  */
  esp.Process();  //background stuff.
  if ((millis()) > (publishtimer + 1000)) {  //output mqtt every second
    
    publishstuff(i);
    i++;
    if (i>(topicNumber))i=0;
  }

  //heatbeat
  if (millis() - lasttime < 1000) digitalWrite(13, HIGH);
  if (millis() - lasttime > 1000) digitalWrite(13, LOW);
  if (millis() - lasttime > 2000) lasttime = millis();
}// end main loop
