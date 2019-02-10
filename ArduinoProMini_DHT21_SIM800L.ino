/*
  GPRS Humidity & Temp Sensor by refe
  using sketch: GPRS_HTTP_THINGSPEAK_v4.ino from https://github.com/T00ManySecrets/water-gprs-temp
  Arduino Pro Mini 3.3V 8Mhz + SIM800L + DHT21/AM2301
  DHT Connected to pin 9 of Arduino Pro Mini 3.3V 8Mhz
  
  GPRS Library - Seeeduino_GPRS
  https://github.com/Seeed-Studio/Seeeduino_GPRS
  SIM800L modified  "\Arduino\libraries\Seeeduino_GPRS-master\sim800.h" 
  line 40 & 41 from
      //#define SIM800_TX_PIN           8
      //#define SIM800_RX_PIN           7
  to
      #define SIM800_TX_PIN           10
      #define SIM800_RX_PIN           11
  //SIM800 TX is connected to Arduino D10 (Arduino RX)
  #define SIM800_TX_PIN 10
 
  //SIM800 RX is connected to Arduino D11 (Arduino RX)
  #define SIM800_RX_PIN 11
  
  Upload readings to Thingspeak.com - https://thingspeak.com/channels/90109

  Added readVcc() function from - https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
*/
#include <gprs.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#define DHTPIN 9    // what digital pin we're connected to



//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
DHT dht(DHTPIN, DHTTYPE);


float temp1;
float batt2;
float batt1;
float hum;


char buffer[512];
// initialize GPRS Module
GPRS gprs;

void setup() {
  
  Serial.begin(9600);
  Serial.println("GPRS - HTTP Connection Test...");

  dht.begin();
 
}

void loop()
{
  
  readSensors();  // updates temp1, hum, batt1
  gprs.preInit();
  
  while (0 != gprs.init()) {
    delay(1000);
    Serial.println("init error");
  }
  
  while (!gprs.join("net")) { //change "cmnet" to your own APN
    Serial.println("gprs join network error");
    delay(2000);
  }
  
  // successful DHCP
  Serial.print("IP Address is ");
  Serial.println(gprs.getIPAddress());

  Serial.println("Init success, start to connect Thingspeak...");

  if (0 == gprs.connectTCP("api.thingspeak.com", 80)) {
    Serial.println("connect api.thingspeak.com success");
  } else {
    Serial.println("connect error");
    // while(1);
  }
  Serial.println(batt1);
  Serial.println("waiting to fetch...");

  char temp1_convert[10];
  dtostrf(temp1,1,2,temp1_convert);

  char hum_convert[10];
  dtostrf(hum,1,2,hum_convert);
 
  char batt1_convert[10];
  dtostrf(batt1,1,2,batt1_convert);
 
 // change your Thingspeak key bellow
  
 snprintf(
    buffer,
    sizeof(buffer),
    //"GET /update?api_key=YOUR API KEY HERE&field1=%s&&field2=%s&&field3=%s&&field4=%s HTTP/1.0\r\n\r\n",
    "GET /update?api_key=YOUR API KEY HERE&field1=%s&&field2=%s&&field3=%s HTTP/1.0\r\n\r\n",
    temp1_convert,
    hum_convert,
    batt1_convert
);
  if (0 == gprs.sendTCPData(buffer));
  
  else {
    Serial.println("connect error");
    return;
  }
  Serial.println(buffer);
  {
    // gprs.serialDebug();

    Serial.println("Reset GSM module!!");
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    delay(2000);
    digitalWrite(2, LOW);
    delay(40000); //delay 40 seconds
  }

delay(40000); //dalay 40 seconds
}

// updates temp1, temp2, hum, batt1
void readSensors(){
  delay(2000);
  hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temp1 = dht.readTemperature();

  if (isnan(hum) || isnan(temp1)) {
    Serial.println("Failed to read from DHT sensor!");
    //return;
  }
  // you can use analogRead on A2, but you must have a 5.0V stable supply
  //(float(analogRead(A2)) / 1023.0) * 5.0;  // range: 0.0 - 5.0V
  batt2 = readVcc(); 
  batt1 = batt2 / 1000;
  Serial.println("Air Temp:");
  Serial.println(temp1);
  Serial.println("Humidity:");
  Serial.println(hum);
  Serial.println("Battery:");
  Serial.println(batt1);
  Serial.println("Battery2:");
  Serial.println(batt2);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //default constant 1125300
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //if you want to chance measure Vcc on Arduino ProMini pins
  //scale_constant = internal1.1Ref * 1023 * 1000
  //internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
  result = 1122627L / result;
  return result; // Vcc in millivolts
}
