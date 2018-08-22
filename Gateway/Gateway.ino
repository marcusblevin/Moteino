// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GARAGENODEID  4
#define FRONTSMID	  7
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "ABCDABCDABCDABCD" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED           9  // Moteinos have LEDs on D9
#define SERIAL_BAUD   115200
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

byte ackCount=0;
byte inputLen=0;
char input[64];
void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    //char input = Serial.read();
    input[0] = Serial.read();
    
    if (input[0] == 'r') //d=dump all register values
      radio.readAllRegs();
    if (input[0] == 'E') //E=enable encryption
      radio.encrypt(ENCRYPTKEY);
    if (input[0] == 'e') //e=disable encryption
      radio.encrypt(null);
    if (input[0] == 'p')
    {
      promiscuousMode = !promiscuousMode;
      radio.promiscuous(promiscuousMode);
      Serial.print("Promiscuous mode ");Serial.println(promiscuousMode ? "on" : "off");
    }
    
    if (input[0] == 'd') //d=dump flash area
    {
      Serial.println("Flash content:");
      int counter = 0;

      while(counter<=256){
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while(flash.busy());
      Serial.println();
    }
    if (input[0] == 'D')
    {
      Serial.print("Deleting Flash chip content... ");
      flash.chipErase();
      while(flash.busy());
      Serial.println("DONE");
    }
    if (input[0] == 'i')
    {
      Serial.print("DeviceID: ");
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
    }
    if (input[0] == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println('F');
    }
  }
  
  //process any serial input
  inputLen = readSerialLine(input);
      
  if (inputLen >= 5)
  {
  	// GARAGEMOTE status checks/commands
    if (input[0]=='G' && input[1]=='R' && input[2]=='G' && input[3]=='O' && input[4]=='P' && input[5]=='N')
    {
      Serial.print("OPN ... ");
      if (radio.sendWithRetry(GARAGENODEID, "OPN", 3))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='G' && input[1]=='R' && input[2]=='G' && input[3]=='C' && input[4]=='L' && input[5]=='S')
    {
      Serial.print("CLS ... ");
      if (radio.sendWithRetry(GARAGENODEID, "CLS", 3))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='G' && input[1]=='R' && input[2]=='G' && input[3]=='S' && input[4]=='T' && input[5]=='S')
    {
      Serial.print("STS ... ");
      if (radio.sendWithRetry(GARAGENODEID, "STS", 3))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    
    //SWITCHMOTE status checks/commands
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='0' && input[4]==':' && input[5]=='0')
    {
      Serial.print("BTN0:0 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN0:0", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='0' && input[4]==':' && input[5]=='1')
    {
      Serial.print("BTN0:1 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN0:1", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='1' && input[4]==':' && input[5]=='0')
    {
      Serial.print("BTN1:0 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN1:0", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='1' && input[4]==':' && input[5]=='1')
    {
      Serial.print("BTN1:1 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN1:1", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='2' && input[4]==':' && input[5]=='0')
    {
      Serial.print("BTN2:0 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN2:0", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='B' && input[1]=='T' && input[2]=='N' && input[3]=='2' && input[4]==':' && input[5]=='1')
    {
      Serial.print("BTN2:1 ... ");
      if (radio.sendWithRetry(FRONTSMID, "BTN2:1", 6))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    // switch the main SSR
    if (input[0]=='S' && input[1]=='S' && input[2]=='R' && input[3]==':' && input[4]=='0')
    {
      Serial.print("SSR:0 ... ");
      if (radio.sendWithRetry(FRONTSMID, "SSR:0", 5))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    if (input[0]=='S' && input[1]=='S' && input[2]=='R' && input[3]==':' && input[4]=='1')
    {
      Serial.print("SSR:1 ... ");
      if (radio.sendWithRetry(FRONTSMID, "SSR:1", 5))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }
    // get status of SSR
    if (input[0]=='S' && input[1]=='S' && input[2]=='R' && input[3]=='S' && input[4]=='T')
    {
      Serial.print("SSRST ... ");
      if (radio.sendWithRetry(FRONTSMID, "SSRST", 5))
        Serial.println("ok ... ");
      else Serial.println("nothing ... ");
    }

  }

  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    
    if (radio.ACK_REQUESTED)
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
      
    }
    Serial.println();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

// reads a line feed (\n) terminated line from the serial stream
// returns # of bytes read, up to 255
// timeout in ms, will timeout and return after so long
byte readSerialLine(char* input, char endOfLineChar, byte maxLength, uint16_t timeout)
{
  byte inputLen = 0;
  Serial.setTimeout(timeout);
  inputLen = Serial.readBytesUntil(endOfLineChar, input, maxLength);
  input[inputLen]=0;//null-terminate it
  Serial.setTimeout(0);
  //Serial.println();
  return inputLen;
}