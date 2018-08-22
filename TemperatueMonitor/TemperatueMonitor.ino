// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

#define NODEID        3    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1  //central node to report data to

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "ABCDABCDABCDABCD" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED           9  // Moteinos have LEDs on D9
#define GLED          A0 // LED for temperature in range
#define RLED		  A3 // LED for temperature out of range
#define TEMP_SENSOR   A7 // temperature sensor is plugged into A0
#define HALFCYCLEDELAY  SLEEP_1S  //this will cause transmissions every 2s
//#define CELSIUS       1  //uncomment this line to transmit in Celsius instead of Fahrenheit
float MAX_TEMP = 45; // max temperature threshold

//#define SERIAL_EN               //uncomment this line to enable serial IO (when you debug Moteino and need serial output)
#define SERIAL_BAUD   115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

char sendBuf[32];
byte sendLen;
//****************************************************************************************************************

RFM69 radio;
SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network
byte readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=50);

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL! (is chip present?)");
}

byte temperatureCounter = 0;
float temperature = 0;

void loop()
{
  String tempstr;
  
  //report temperature every 30 transmit cycles
  temperature += getTemperature();
  if (++temperatureCounter%30==0)
  {
    temperature /= 30;
    int whole = temperature;
    int decimal = ((int)(temperature * 100)) % 100;
    	if (decimal < 0)  {  decimal *= -1;  }
    tempstr=String("TMP:");
    tempstr += NODEID;
    //tempstr += "_T";
    tempstr += ':';
    tempstr += whole;
    tempstr += '.';
    tempstr += decimal;
    tempstr.toCharArray(sendBuf, RF69_MAX_DATA_LEN);
    sendLen = tempstr.length();
    DEBUG(tempstr);
    
    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
      DEBUG(" ACK:OK! RSSI:");
      DEBUGln(radio.RSSI);
    }
    else DEBUGln(" ACK:NOK...");
    radio.sleep();
    
    // toggle LED if temp is in/out of range
    if (temperature > MAX_TEMP) {
      pinMode(GLED, OUTPUT);
      digitalWrite(GLED, LOW);
      pinMode(RLED, OUTPUT);
      digitalWrite(RLED, HIGH);
    }
    else {
      pinMode(RLED, OUTPUT);
      digitalWrite(RLED, LOW);
      pinMode(GLED, OUTPUT);
      digitalWrite(GLED, HIGH);
    }
    
    temperatureCounter = temperature = 0;
  }
  
  Blink(9, 3); //Blink half cycle later (to offset LED current drain)
  LowPower.powerDown(HALFCYCLEDELAY, ADC_OFF, BOD_ON);
}

void Blink(byte PIN, int DELAY_MS) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

float getTemperature() {
  int reading = analogRead(TEMP_SENSOR);
  // converting TMP36 reading to voltage, for 3.3v use ~3.3
  float voltage = reading * 3.297;
  voltage /= 1024.0;
  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
												//to degrees ((volatge - 500mV) times 100)
  #ifdef CELSIUS
	return temperatureC;
  #else
	float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
	DEBUGln(temperatureF);
	return temperatureF;
  #endif
}
