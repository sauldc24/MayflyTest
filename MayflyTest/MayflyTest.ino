#include <DallasTemperature.h>
#include <OneWire.h>
#include <SD.h>
#include <CayenneLPP.h>
#include <Sodaq_PcInt.h>
#include <Sodaq_DS3231.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RAK811.h>
#include <LowPower.h>
#define DEBUG

//switched power lines pin
#define SWITCHED_POWER 22

//Error flag and error LED pin
#define ERROR_LED 9
bool error;

//Operation Success pin
#define SUCCESS_LED 8
bool success;

//DateTime variable for the time
DateTime now;

//create a string buffer for SD card and printing info to the debugging serial port
String string_buffer;

//File object variable
File file;
//COnfiguration of battery voltage variables
int batteryPin = A6;    // on the Mayfly board, pin A6 is connected to a resistor divider on the battery input; R1 = 10 Mohm, R2 = 2.7 Mohm
int batterysenseValue = 0;  // variable to store the value coming from the analogRead function
float batteryvoltage;       // the battery voltage as calculated by the formula below
// formula for battery level calculation is batteryvoltage = (3.3/1023.) * 4.7 * batterysenseValue; 

//create and configure CayenneLPP buffer wit a buffer size of 51
CayenneLPP lpp(51);

//Create and configure DHT21 object and variables for temperature and humidity
#define DHTPIN 10
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);
float temperature2;
float humidity;

// Configure DS18B20 sensor and variable for temperature
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
float temperature = 0;

//declare LoRa Radio
RAK811 lora(Serial1, Serial);

//pin for the RTC alarm interrupt
int interruptPin = A7;

//pin to handle the tipping bucket interrupt and variables for antidebouncing
#define tippingBucketPin 6
bool prevState;
bool newState;
bool tippingcounts;

// Interrupt service routine for RTC alarm
void INT0_ISR()
{
  //nothig here, ,just an interrupt to awake the device
}

//interrupt service routine for Tipping bucket
void TB_ISR()
{
  //wait 1 ms for switch debouncing
  delayMicroseconds(1000);
  //check the pin new state after debouncing and compare with previous state
  newState = digitalRead(tippingBucketPin);
  if (newState != prevState)
  {
    //increment tipping  bucket counts
    tippingcounts++;
    //set previous state with the value of new state
    prevState = newState;
  }
  //at the end of this interruption always go back to power down state
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void setup ()
{
    //configure switched power, error and success pins
    pinMode(SWITCHED_POWER, OUTPUT);
    digitalWrite(SWITCHED_POWER, LOW);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(ERROR_LED, LOW);
    pinMode(SUCCESS_LED, OUTPUT);
    digitalWrite(SUCCESS_LED, LOW);
    
    //initialize ds18b20 library
    ds18b20.begin();

    //initialize debugging serial
    #if defined DEBUG
    Serial.begin(9600);
    #endif
    
    //initialize the RAK811 serial port
    Serial1.begin(9600);
    if (lora.rk_begin())
    {
      #if defined DEBUG
      Serial.println(F("LoRa initialization OK")); 
      #endif
      blink(SUCCESS_LED);
    }
    else 
    {
      #if defined DEBUG
      Serial.println(F("LoRa initialization Error"));
      #endif
      blink(ERROR_LED);
    }
    while (!SD.begin(12)) {//si no se logra inicializar la SD
      #if defined DEBUG
      Serial.println("No SD");
      #endif
      digitalWrite(ERROR_LED, HIGH);
      delay(500);//espera antes de volver a intentarlo 
    }
    digitalWrite(ERROR_LED, LOW);
    //adjust RTC if neccesary
    //char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    //DateTime dt(2020, 1, 10, 15, 18, 0, 5);
    //rtc.setDateTime(dt); //Adjust date-time as defined 'dt' above

    //initialize the dht
    //dht.begin();
    //atach tipping bucket interrupt
    PcInt::attachInterrupt(tippingBucketPin, TB_ISR);
    //Configure the interrupt to awake the device
    pinMode(interruptPin, INPUT_PULLUP);
    PcInt::attachInterrupt(interruptPin, INT0_ISR);
    //initialize the rtc
    rtc.begin();
    #if defined DEBUG
    rtc.enableInterrupts(EveryMinute);
    #else
    rtc.enableInterrupts(EveryHour);
    #endif
    }


void loop ()
{
    //turn the switched source on
    digitalWrite(SWITCHED_POWER, HIGH);
    //initialize DHT library
    dht.begin();
    //read DS18B20
    ds18b20.requestTemperatures();
    temperature = ds18b20.getTempCByIndex(0);
    //delay for DHT21 startup
    delay(500);
    //read DHT21
    temperature2 = dht.readTemperature();
    humidity = dht.readHumidity();
    //power sensor off as soon as it finishes reading
    digitalWrite(SWITCHED_POWER, LOW);
    string_buffer = "";
    batterysenseValue = analogRead(batteryPin); 
    batteryvoltage = (3.3/1023.) * 4.7037 * batterysenseValue; 
    now = rtc.now();
    //
    string_buffer = String(now.date()) + "_" + String(now.month()) + ".csv";

    // section to verify if the file exists, if it doesn't then create it and generate the text file header
    if (!SD.exists(string_buffer))
    {
      file = SD.open(string_buffer, FILE_WRITE);
      if (file)
      {
        file.println(F("Fecha/hora, Temperatura 1, Temperatura 2, Humedad, Bateria, Radiacion, Pluvial"));
        file.close();
      }
      else error = true;
    }
    file = SD.open(string_buffer, FILE_WRITE);
    string_buffer = ""; 
    now.addToString(string_buffer);
    string_buffer.concat("," + String(temperature) + "," + String(temperature2) + "," + String(humidity) + "," + String(batteryvoltage));
    #if defined DEBUG
    Serial.println(string_buffer);
    #endif
    if (file)
    {
      file.println(string_buffer);
      file.close();
      #if defined DEBUG
      Serial.println(F("Succesfully written to the SD"));
      #endif
      error = false;
    }
    else error = true;
    SendCayenne();
    //update error status pin
    if (error)
    {
      digitalWrite(ERROR_LED, HIGH);
    } 
    //This section clears the alarm flag of the RTC and puts the device in deep sleep
    rtc.clearINTStatus();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}


void SendCayenne()
{
    lora.rk_wake();
    lpp.reset();
    lpp.addTemperature(1, temperature);
    lpp.addTemperature(2, temperature2);
    lpp.addRelativeHumidity(3, humidity);
    lpp.addAnalogInput(10, batteryvoltage);
    lora.rk_sendBytes(2, lpp.getBuffer(), lpp.getSize());
    lora.rk_sleep();
}

void blink(int pin)
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(pin, HIGH);
    delay(1000);
    digitalWrite(pin, LOW);    
    delay(1000);
  }
}








/*
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  Serial1.print(F("at+send=lora:2:"));
  for (int i=0; i<length; i++) {
    if (data[i]<0x10) {Serial1.print("0");}
    Serial1.print(data[i],HEX);
  }
  Serial1.print(F("\r\n"));
  delay(200);
  while (Serial1.available() > 0)
  {
    Serial1.read();
  }
}*/
