#include <SoftwareSerial.h>
#include <string.h>
#include <util/crc16.h>
#include <TinyGPS.h>

#include <OneWire.h> 

int DS18S20_Pin = 7; //DS18S20 Signal pin on digital 7
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 7


/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 3(rx) and 4(tx).
*/
// http://ukhas.org.uk/guides:common_coding_errors_payload_testing
TinyGPS gps;
SoftwareSerial ss(4, 5);

int msgcount = 0;

int RADIO_SPACE_PIN=10;
int RADIO_MARK_PIN=11; 

char DATASTRING[200];

boolean setgpsmode = false;

int heading;
int battery;
int velocity;
int sats;

byte gps_set_sucess = 0 ;


//dim vars
char timechara[9];
int alt;
    char latstr[10] = "0";
    char lonstr[10] = "0";


//chris stubbs awful 1.1 internal aref voltage reader
const int voltPin = 0;
float denominator;



void setup()
{
  
  //chris stubbs awful 1.1int aref voltage reader   Loads of code stolen from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  analogReference(INTERNAL); //sets 1.1v internal ref RUN BEFORE USING ANALOGREAD OR ARDUINO WILL POP!!!!
denominator = (float)4700 / (47000 + 4700); 
  
  
  delay(5000);// let the GPS settle down
  
  Serial.begin(9600);
  
  
  ss.begin(9600);
  
  //change GPS baud rate to 4800
  ss.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
    delay(1000); //pause after changing baud rate
 ss.begin(4800);
  ss.flush();
  
  delay(1000); //pause after changing baud rate
  
  Serial.println("Setting uBlox nav mode: ");

  
  
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println("Heavily edited for use with uBlox GPS and NTX2 as a GPS tracker by Chris Stubbs");
  Serial.println("NTX2 code from UPU");
  Serial.println("Checksum code from Lunar_Lander");
  Serial.println("http://bit.ly/113dGMR");
  Serial.println();
  
  pinMode(RADIO_SPACE_PIN,OUTPUT);
  pinMode(RADIO_MARK_PIN,OUTPUT);
  
    sprintf(DATASTRING,"$$NSE POWER ON \n");
    noInterrupts();
    rtty_txstring (DATASTRING);
    interrupts();
    
    delay(1000); //power up wait
    
    pinMode(6, OUTPUT);// GND
    pinMode(8, OUTPUT); // 5v
    digitalWrite(6, LOW); // - power for temp
    digitalWrite(8, HIGH); // + power for temp

}

void loop()
{
  
  
  
  msgcount = msgcount + 1;
    
    //if mscount is getting cloase to uper limit of Int, then set it back to 1
    if(msgcount > 32000)
    {
      msgcount = 1;
    }
  
  
 // delay(1000);              // wait for a second
  
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
       //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    
    if(setgpsmode == false)
    {
        uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
  setgpsmode = true;
  
    sprintf(DATASTRING,"FLIGHT MODE SET \n");
    noInterrupts();
    rtty_txstring (DATASTRING);
    interrupts();
    
    }
    
    
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
      /*  
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    */
    
    

    dtostrf(flat,9,6,latstr); // convert lat from float to string
    dtostrf(flon,9,6,lonstr); // convert lon from float to string
    
    
      
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long fix_age;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);

    //build up time stamp
    String tmstr;
    if(hour < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(hour, DEC);
   // tmstr = tmstr + ":";
    if(minute < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(minute, DEC);
   // tmstr = tmstr + ":";
    if(second < 10)
    {
       tmstr = tmstr + "0";
    }
    tmstr = tmstr + String(second, DEC);
    
    tmstr.toCharArray(timechara, 9) ;
    
   //correct no of 0's in long, (output: 00.575950/-0.575950)
    if (lonstr[0] == ' ') {
    lonstr[0] = '0';
    }
   
   alt = gps.f_altitude(); // +/- altitude in meters
   
   velocity = gps.f_speed_mps()*10;
   heading = gps.f_course();
   sats = gps.satellites();
  }
  
   if(newData == false)
   {
    // Serial.println("No new data");
     /*
    sprintf(DATASTRING,"$$NSE,NOGPS");
    noInterrupts();
    rtty_txstring (DATASTRING);
    interrupts();
    */
    
    //flash led for no new data
     digitalWrite(13, HIGH);   // set the LED on
     delay(10);              // wait for a second
     digitalWrite(13, LOW);    // set the LED off
     Serial.println("NND F.LED");
        sats = 0;
   }
  
  
  gps.stats(&chars, &sentences, &failed);
  /*
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  */
  
  
  float voltage;
  analogReference(INTERNAL); //just incase...
  voltage = analogRead(A0);
  voltage = (voltage / 1024) * 1.1;
  voltage = voltage / denominator;
  int battvolts = voltage *10;

  
  
 // float temperature1 = getTemp();
 // int tempr1 = temperature1 + 0.5; // bodged round
  int tempr1 = setgpsmode;
  int tempr2 = normalizeTemperature(readTemp());

    Serial.println("SENDING DATA via 434:");
    
    sprintf(DATASTRING,"$$$$NSE,%i,%s,%s,%s,%i,%i,%i,%i,%i",msgcount,timechara,latstr,lonstr,alt,sats,battvolts,tempr1,tempr2); //put together all var into one string //now runs at end of loop()
    crccat(DATASTRING + 4); //add checksum (lunars code)
    
    Serial.println(DATASTRING);//output data to serial
    noInterrupts();
    rtty_txstring (DATASTRING);
    interrupts();
}

void rtty_txstring (char * string)
{

	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/

	char c;

	c = *string++;

	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}


void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	*/

	int i;

	rtty_txbit (0); // Start bit

	// Send bits for for char LSB first	

	for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
	{
		if (c & 1) rtty_txbit(1); 

			else rtty_txbit(0);	

		c = c >> 1;

	}

	rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    digitalWrite(RADIO_MARK_PIN, HIGH);
                    digitalWrite(RADIO_SPACE_PIN, LOW);
		}
		else
		{
		  // low
                    digitalWrite(RADIO_SPACE_PIN, HIGH);
                    digitalWrite(RADIO_MARK_PIN, LOW);

		}
//                delayMicroseconds(1680); // 600 baud unlikely to work.
//                  delayMicroseconds(3370); // 300 baud
                delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
                delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.

}

void callback()
{
  digitalWrite(RADIO_SPACE_PIN, digitalRead(RADIO_SPACE_PIN) ^ 1);
}



// calc CRC-CCITT (0xFFFF)
// Datastring CRC16 Checksum Mechanism from Lunar_Lander
uint16_t crccat(char *msg)
{
  uint16_t x;
  for(x = 0xFFFF; *msg; msg++)
  x = _crc_xmodem_update(x, *msg);
  snprintf(msg, 8, "*%04X\n", x);
  return(x);
}




//external thermometer
float getTemp(){
      
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
  
}


//internal thermometer
float readTemp() {
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = (result - 125) * 1075;
return result;
}

int normalizeTemperature(long rawData) { 
  // replace these constants with your 2 data points
  // these are sample values that will get you in the ballpark (in degrees C)
  float temp1 = -18.7;
  long data1 = 193500;
  float temp2 = 60.1;
  long data2 = 298850;
  // calculate the scale factor
  float scaleFactor = (temp2 - temp1) / (data2 - data1);
  // now calculate the temperature
  float temp = scaleFactor * (rawData - data1) + temp1;
  int output = temp + 0.5;
  output = output - 2;
  return output;
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    ss.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  ss.println();
}





// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (ss.available()) {
      b = ss.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}
