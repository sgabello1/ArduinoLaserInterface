#include <Tone.h>

//Laser through serial

#include <ros.h>
#include <stdio.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <SPI.h>

// ROS stuff
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Float32 ack_msg;
ros::Publisher chatter("acknowledgment", &ack_msg);
//ros::Publisher chatter_verbose("acknowledgment_verbose", &str_msg);

//Tone STUFF
Tone tone1;

//LASER stuff
// set pin 10 as the slave select for the digital pot:
const int slaveSelectPin = 10;
const int sixtykHzPin = 4;
const int HVenablePin = 2;
unsigned int DAC_ZERO_LEVEL=32767;
float MIRROR_INC_MAX_DEG=4.5;
int datasize = 0; 

void messageCb( const std_msgs::Float32MultiArray& msg){
  
  datasize = msg.data[0];
  ack_msg.data = datasize;
  chatter.publish( &ack_msg );

 
  if(msg.data[1] == 10 && msg.data[2] == 10){ //since 0 position is the size

    WritePicoAmpXY(0,0); // put HVpin down
    delay(10);
    
    
    digitalWrite(HVenablePin,LOW);


  } else
    {
  
  
  // LASER stuff
  

  // ENABLE HIGH VOLTAGE!

  digitalWrite(HVenablePin,HIGH);

  delay(1);
  // DRAW VECTOR STUFF

  for(int i=1;i<datasize;i = i + 2)

  {
   // str_msg.data = "Start drawing";
   // chatter_verbose.publish( &str_msg );


    /*WritePicoAmpXY(msg.data[0],msg.data[1]);

    delay(4);

    WritePicoAmpXY(msg.data[2],msg.data[3]);

    delay(4);

    WritePicoAmpXY(msg.data[4],msg.data[5]);

    delay(4);

    WritePicoAmpXY(msg.data[6],msg.data[7]);

    delay(4);*/

    //str_msg.data = "Finish draw";
    //chatter_verbose.publish( &str_msg );

    WritePicoAmpXY(msg.data[i],msg.data[i+1]);
    ack_msg.data = msg.data[i]*100 + msg.data[i+1];//i;
    chatter.publish( &ack_msg );

    delay(4);

  }

    }
  
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("coordinates", &messageCb );


void setup()
{
  //pinMode(9, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  //nh.advertise(chatter_verbose);

  digitalWrite(HVenablePin,LOW);
  pinMode (HVenablePin, OUTPUT);
  digitalWrite(HVenablePin,LOW);
  
  // INITIALIZE SPI:
  //pinMode (slaveSelectPin, OUTPUT); // set the slaveSelectPin as an output:
  SPI.setBitOrder(MSBFIRST); // or MSBFIRST according to the AD5664R
  SPI.setDataMode(SPI_MODE1); // SPI_MODE0 up to SPI_MODE3 see also http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus clocl polarity and phase
  //SPI.setClockDivider(SPI_CLOCK_DIV64); // Uno runs at 16MHz which gives a SPI  clock of 1MHz
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Uno runs at 16MHz which gives a SPI  clock of 1MHz
  SPI.begin();
  delay(100);
  
//  Recommended register downloads for Pico Amp:
//  2621441 Dec = 280001 Hex FULL RESET
//  3670017 Dec = 380001 Hex ENABLE INTERNAL REFERENCE
//  2097167 Dec = 20000F Hex ENABLE ALL DAC CHANNELS
//  3145728 Dec = 300000 Hex ENABLE SOFTWARE LDAC
  digitalAmpWrite(0x28,0x00,0x01);
  delay(100);
  
  digitalAmpWrite(0x38,0x00,0x01);
  delay(100);
  
//  digitalAmpWrite(0x20,0x00,0x08); //enable all dac channels
//  delay(100);
//  digitalAmpWrite(0x20,0x00,0x07);
//  delay(100);
  digitalAmpWrite(0x20,0x00,0x0F); //enable all dac channels
  delay(100);


  // Set all channels to 1.25V bias (direct write to DAC channel register)
  digitalAmpWrite(0x1F,0x7F,0xFF);   
  delay(10000);
  
  //digitalAmpWrite(0x30,0x00,0x0F); // Set LDAC mode 1111
  // After DAC is enabled, send 32768 digital input to all four channels:
  // 32768 DEZ = 0x8000
  //Serial.begin(9600);

  tone1.begin(8);
  tone1.play(60000);
}

void loop()
{
  //ROS stuff
  nh.spinOnce();
  delay(1);
  
}

void WritePicoAmpXY(float x_deg, float y_deg)

{

  unsigned int DAC_OUT_VALUE_X_PLUS=  DAC_ZERO_LEVEL;

  unsigned int DAC_OUT_VALUE_X_MINUS= DAC_ZERO_LEVEL;

  unsigned int DAC_OUT_VALUE_Y_PLUS=  DAC_ZERO_LEVEL;

  unsigned int DAC_OUT_VALUE_Y_MINUS= DAC_ZERO_LEVEL;

  

  if(fabs(x_deg)/MIRROR_INC_MAX_DEG <= 1.0)

  {

    DAC_OUT_VALUE_X_PLUS  =  DAC_ZERO_LEVEL + int(x_deg/MIRROR_INC_MAX_DEG*32767.0);

    DAC_OUT_VALUE_X_MINUS =  DAC_ZERO_LEVEL - int(x_deg/MIRROR_INC_MAX_DEG*32767.0);

  

    if(DAC_OUT_VALUE_X_PLUS < 65535 && DAC_OUT_VALUE_X_PLUS > 0 && DAC_OUT_VALUE_X_MINUS < 65535 && DAC_OUT_VALUE_X_MINUS > 0)

    {

      digitalAmpWrite(0x18,Int16ToHighByte(DAC_OUT_VALUE_X_PLUS),Int16ToLowByte(DAC_OUT_VALUE_X_PLUS));

      digitalAmpWrite(0x19,Int16ToHighByte(DAC_OUT_VALUE_X_MINUS),Int16ToLowByte(DAC_OUT_VALUE_X_MINUS));

    }

  }

  

  if(fabs(y_deg)/MIRROR_INC_MAX_DEG <= 1.0)

  {

    DAC_OUT_VALUE_Y_PLUS  =  DAC_ZERO_LEVEL + int(y_deg/MIRROR_INC_MAX_DEG*32767.0);

    DAC_OUT_VALUE_Y_MINUS =  DAC_ZERO_LEVEL - int(y_deg/MIRROR_INC_MAX_DEG*32767.0);

  

    if(DAC_OUT_VALUE_Y_PLUS < 65535 && DAC_OUT_VALUE_Y_PLUS > 0 && DAC_OUT_VALUE_Y_MINUS < 65535 && DAC_OUT_VALUE_Y_MINUS > 0)

    {

      digitalAmpWrite(0x1A,Int16ToHighByte(DAC_OUT_VALUE_Y_PLUS),Int16ToLowByte(DAC_OUT_VALUE_Y_PLUS));

      digitalAmpWrite(0x1B,Int16ToHighByte(DAC_OUT_VALUE_Y_MINUS),Int16ToLowByte(DAC_OUT_VALUE_Y_MINUS));

    }

  }

  

}

byte Int16ToHighByte(unsigned int Integer16high)

{

  unsigned int tmp_int=Integer16high >> 8;

  byte tmp_highbyte=(byte)tmp_int;

  return tmp_highbyte;

}

byte Int16ToLowByte(unsigned int Integer16low)

{

  byte tmp_lowbyte=byte(Integer16low);

  return tmp_lowbyte;

}



// SPI Write 24-Bit Command interface

//====================================

void digitalAmpWrite(char byte2, char byte1, char byte0)

{

  // take the SS pin low to select the chip:

  digitalWrite(slaveSelectPin,LOW);

  //  send in the address and value via SPI:

  SPI.transfer(byte2);

  SPI.transfer(byte1);

  SPI.transfer(byte0); // Change order to fulfill MSB first convention

  // take the SS pin high to de-select the chip:

  digitalWrite(slaveSelectPin,HIGH); 

}
