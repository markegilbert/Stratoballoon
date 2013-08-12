#include <OneWire.h> 
#include <Wire.h>
#include <SoftwareSerial.h>


// *********************************************************************
// General settings and variables
int SAMPLING_DELAY = 1000;

boolean HasGoodGPSReading;
boolean HasGoodTempReading;


// *********************************************************************
// Temperature Sensor settings
int TEMP_SENSOR_PIN = 2; //DS18S20 Signal pin on digital 2


// *********************************************************************
// Pressure Sensor settings/variables
float RawTemp, CurrentTemp;
float CurrentPressure;
float CurrentAltitude;

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5; 

//Temperature chip i/o
OneWire ds(TEMP_SENSOR_PIN);  // on digital pin 2


// *********************************************************************
// Data logger settings
SoftwareSerial DataLogger(11, 12); // RX, TX


// *********************************************************************
// GPS receiver settings/variables
SoftwareSerial GPSReceiver(10, 9); // RX, TX (TX not used)
const int sentenceSize = 80;
char GPSRawData[sentenceSize];
float CurrentLatitude;
float CurrentLongitude;
int CurrentTimeStamp;


void setup(void) {
  Serial.begin(9600);
  
  Wire.begin();
  RawTemp = -1000.0;
  CurrentTemp = 0.0;
  
  CalibratePressureSensor();
  CurrentPressure = 0.0;
  CurrentAltitude = 0.0;

  DataLogger.begin(9600);

  GPSReceiver.begin(9600);
  CurrentLatitude = 0.0;
  CurrentLongitude = 0.0;
  CurrentTimeStamp = 0;

  HasGoodGPSReading = false;
  HasGoodTempReading = false;
}


void loop(void) {
  
  if(!HasGoodGPSReading) 
  {
    static int i = 0;
    if(!GPSReceiver.isListening()) { GPSReceiver.listen(); }
    if (GPSReceiver.available())
    {
      char ch = GPSReceiver.read();
  
      if(ch == '$') { i = 0; }
  
      if (ch != '\n' && i < sentenceSize)
      {
        GPSRawData[i] = ch;
        i++;
      }
      else
      {
        GPSRawData[i] = '\0';
        i = 0;
    
        char GPSComponent[20];
        ExtractGPSComponent(GPSComponent, 0);
        if(strcmp(GPSComponent, "$GPRMC") == 0)
        {
          ExtractGPSComponent(GPSComponent, 1);
          CurrentTimeStamp = atoi(GPSComponent);
          ExtractGPSComponent(GPSComponent, 3);
          CurrentLatitude = atof(GPSComponent);
          ExtractGPSComponent(GPSComponent, 5);
          CurrentLongitude = atof(GPSComponent);
          
          HasGoodGPSReading = true;
        }
      }
    }
  }


  // Now that we have a good GPS reading, grab the temperature data
  if(HasGoodGPSReading && !HasGoodTempReading) 
  {
    RawTemp = GetTemp();
    if(RawTemp > -1000) 
    { 
      CurrentTemp = RawTemp; 
      HasGoodTempReading = true;
    }
  }
  
  
  // Now that we have a good GPS and temperature reading, grab
  // the rest of the data, and log it.
  if(HasGoodGPSReading && HasGoodTempReading) 
  {
    CurrentPressure = GetPressure(bmp085ReadUT(), bmp085ReadUP());
    CurrentAltitude = CalculateAltitude(CurrentPressure);
    
    PrintToSerialOutput(CurrentTimeStamp, CurrentTemp, CurrentPressure, CurrentAltitude, CurrentLatitude, CurrentLongitude);
    WriteDataToLogger(CurrentTimeStamp, CurrentTemp, CurrentPressure, CurrentAltitude, CurrentLatitude, CurrentLongitude);

    // TODO: What data do we write to the radio?
    // TODO: How frequently should we write data to the radio?
    // TODO: Write data to radio
    
    HasGoodGPSReading = false;
    HasGoodTempReading = false;
  }

}




void PrintToSerialOutput(int TimeStamp, float Temperature, float Pressure, float Altitude, float Latitude, float Longitude) {

  // We print each component of the data line using separate calls instead of using something like sprintf to
  // format a single string.  This is done because the version of sprintf available to us for the Arduino does
  // not support floats.  This was the simplest Plan B we could come up with.
  
  Serial.print(TimeStamp);
  Serial.print(",");
  Serial.print(Temperature, 2);
  Serial.print(",");
  Serial.print(Pressure, 0);
  Serial.print(",");
  Serial.print(Altitude, 1);
  Serial.print(",");
  Serial.print(Latitude, 4);
  Serial.print(",");
  Serial.print(Longitude, 4);
  Serial.println();
}


void WriteDataToLogger(int TimeStamp, float Temperature, float Pressure, float Altitude, float Latitude, float Longitude) {
  if(!DataLogger.isListening()) { DataLogger.listen(); }

  DataLogger.print(TimeStamp);
  DataLogger.print(",");
  DataLogger.print(Temperature, 2);
  DataLogger.print(",");
  DataLogger.print(Pressure, 0);
  DataLogger.print(",");
  DataLogger.print(Altitude, 1);
  DataLogger.print(",");
  DataLogger.print(Latitude, 4);
  DataLogger.print(",");
  DataLogger.print(Longitude, 4);
  DataLogger.println();
}


// *********************************************************************
// Temperature sensor subroutines
float GetTemp(){
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


// *********************************************************************
// Pressure sensor subroutines

void CalibratePressureSensor()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Value returned will be pressure in units of Pa.
long GetPressure(unsigned long ut, unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;


  // These three lines were originally from the bmp085GetTemperature function.
  // These are used merely to calculate the "b5" variable.
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;
  

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

float CalculateAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}



// *********************************************************************
// GPS Receiver subroutines
void ExtractGPSComponent(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (GPSRawData[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = GPSRawData[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 

