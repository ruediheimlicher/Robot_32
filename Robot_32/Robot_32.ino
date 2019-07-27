///
/// @mainpage	Robot_32
///
/// @details	PWM for Servos
///
/// @file		Robot_32.ino
/// @brief		Main sketch
///
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
///
/// @copyright	(c) Ruedi Heimlicher, 2019
////// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu

#include "Arduino.h"

// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
//#define STARTWERT  2840 // Mitte
#define STARTWERT  2080 // Nullpunkt
#define MAXWERT  4096 // Nullpunkt

byte buffer[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

volatile uint8_t usbtask = 0;

volatile uint8_t teensytask = 0;

uint16_t pot0 = 0;
uint16_t pot1 = 0;
uint16_t pot2 = 0;
uint16_t pot3 = 0;

#define  ACHSE0_BYTE_H  4
#define  ACHSE0_BYTE_L  5

#define  ACHSE1_BYTE_H  6
#define  ACHSE1_BYTE_L  7


#define ACHSE0_START  0x680 // Startwert low
#define ACHSE0_MAX  0xFFF // Startwert high


uint16_t achse0_start = ACHSE0_START;
uint16_t achse0_max = ACHSE0_MAX;

#define ACHSE1_START  0x780 // Startwert low
#define ACHSE1_MAX  0xFFF // Startwert high

uint16_t achse1_start = ACHSE1_START;
uint16_t achse1_max = ACHSE1_MAX;


#define  ACHSE2_BYTE_H  16
#define  ACHSE2_BYTE_L  17


#define  ACHSE3_BYTE_H  18
#define  ACHSE3_BYTE_L  19



#define SET_0  0xA1
#define SET_1   0xB1
#define SET_ROB 0xA2
//let GET_U:UInt8 = 0xA2
//let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;
float sinpos = 0;
#define pi 3.14
#define SIN_START   0xC0
#define SIN_STOP   0xE1


#define LOOPLED 13

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions


// Add setup code
void setup()
{
   Serial.begin(9600);
   analogWriteResolution(16); // 32767
   
   pinMode(LOOPLED, OUTPUT);
   
   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4   
   // FTM2   25, 32
   analogWriteFrequency(5, 50);
   Serial.println(F("RawHID Example"));
   for (int i=0; i<7; i++) 
   {
      pinMode(i, OUTPUT);
   }
   analogWrite(5, 0x700 + achse0_start);
   
   analogWrite(6, 0x700 + achse0_start);
   
}

// Add loop code
void loop()
{
   if (sinceblink > 1000)
   {
      sinceblink = 0;
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));
   }
   int n;
   n = RawHID.recv(buffer, 10); // 0 timeout = do not wait
   if (n > 0) 
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //Serial.print(F("Received packet, erstes byte: "));
      //Serial.println((int)buffer[0]);
      for (int i=0; i<8; i++) 
      {
  //       int b = buffer[0] & (1 << i);
  //       Serial.print((int)buffer[i]);
  //       Serial.print("\t");
         //digitalWrite(i, b);
      }
 //     Serial.println();
      uint16_t hb = (uint16_t)buffer[4];
      uint16_t lb = (uint16_t)buffer[5];
 //     Serial.print(hb);
 //     Serial.print("\t");
 //     Serial.print(lb);
 //     Serial.println();
      usbtask = buffer[0];
      Serial.print("usbtask ");
      Serial.println(usbtask);
      
      //usbtask = SET_0;
      switch (usbtask)
      {
         case SET_0: // data
         {
            teensytask = 0;
            pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
            analogWrite(5, pot0 + achse0_start);
            
   //         pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
   //         analogWrite(6, pot1 + achse0_start);
            //pot0 = (buffer[4])<<8 + buffer[5];
            
            Serial.print("Pot 0: ");
            Serial.print((int)pot0);
            Serial.print("\t");
            Serial.print("Pot 1: ");
            Serial.print((int)pot1);
            
            //    Serial.print(buffer[4]);
            //    Serial.print("\t");
            //    Serial.print(buffer[5]);
            //    Serial.print("\t");
            Serial.println();

         }break;
            
         case SET_1: // data
         {
            teensytask = 0;
  //          pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
  //          analogWrite(5, pot0 + achse0_start);
            
            pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
            analogWrite(6, pot1 + achse0_start);
            //pot0 = (buffer[4])<<8 + buffer[5];
            
            Serial.print("Pot 0: ");
            Serial.print((int)pot0);
            Serial.print("\t");
            Serial.print("Pot 1: ");
            Serial.print((int)pot1);
            
            //    Serial.print(buffer[4]);
            //    Serial.print("\t");
            //    Serial.print(buffer[5]);
            //    Serial.print("\t");
            Serial.println();
            
         }break;
            
         case SET_ROB:
         {
            teensytask = 0;
            pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
            analogWrite(5, pot0 + achse0_start);
            
            pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
            analogWrite(6, pot1 + achse1_start);
            
            Serial.print("Pot 0: ");
            Serial.print((int)pot0);
            Serial.print("\t");
            Serial.print("Pot 1: ");
            Serial.print((int)pot1);
            Serial.println();

         }break;
            
         case SIN_START: // sinus
         {
            sinms = 0;
            teensytask = SIN_START;
            Serial.print("teensytask: ");
            Serial.println((int)teensytask);
         }break;
         case SIN_STOP:
         {
            teensytask = SET_0;
         }
      }// switch
   } // n>0
   
   
   if (teensytask == SIN_START)
   {
      if (sinms > 10)
      {
         sinms = 0;
         //float tempsin = 0x800 + achse0_start + 1000 * sin(sinpos/180*pi);
        float tempsin0 = 0x800 + achse0_start + 0x800 * sin(sinpos/180*pi);
         Serial.print("Sin Pot0: ");
         //Serial.print((int)sinpos);
         Serial.print("\t");
         Serial.print((int)tempsin0);
         
         
         analogWrite(6, (int)tempsin0);
         
         float tempsin1 = 0x800 + achse0_start + 0x800 * sin(1.7*sinpos/180*pi);
         analogWrite(5, (int)tempsin1);
         Serial.print("\t");
         Serial.print((int)tempsin1);
      
         Serial.println();
         sinpos += 1;
         
      }
   }
    
   
   // every 2 seconds, send a packet to the computer
   if (msUntilNextSend > 4000) 
   {
      msUntilNextSend = msUntilNextSend - 2000;
      /*
      // first 2 bytes are a signature
      buffer[10] = 0xAB;
      buffer[11] = 0xCD;
      // next 24 bytes are analog measurements
      for (int i=5; i<12; i++) 
      {
         int val = analogRead(i);
         buffer[i * 2 + 2] = highByte(val);
         buffer[i * 2 + 3] = lowByte(val);
      }
      // fill the rest with zeros
      for (int i=26; i<62; i++) 
      {
         buffer[i] = 0;
      }
      // and put a count of packets sent at the end
      buffer[62] = highByte(packetCount);
      buffer[63] = lowByte(packetCount);
      */
      // actually send the packet
      n = RawHID.send(buffer, 100);
      if (n > 0) 
      {
         Serial.print(F("Transmit packet "));
         Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
   }
} // loop
