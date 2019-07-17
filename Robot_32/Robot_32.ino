///
/// @mainpage	Robot_32
///
/// @details	PWM for Servos
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2019
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Robot_32.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2019
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu

#include "Arduino.h"

// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
#define STARTOFFSET  2840

byte buffer[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;


uint16_t pot0 = 0;
uint16_t pot1 = 0;
uint16_t pot2 = 0;
uint16_t pot3 = 0;


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions


// Add setup code
void setup()
{
   Serial.begin(9600);
   analogWriteResolution(16); // 32767
   
   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4   
   // FTM2   25, 32
   analogWriteFrequency(5, 50);
   Serial.println(F("RawHID Example"));
   for (int i=0; i<7; i++) 
   {
      pinMode(i, OUTPUT);
   }
   analogWrite(5, 0x800 + STARTOFFSET);
   analogWrite(6, 0x800 + STARTOFFSET);
   
}

// Add loop code
void loop()
{
   int n;
   n = RawHID.recv(buffer, 10); // 0 timeout = do not wait
   if (n > 0) 
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //Serial.print(F("Received packet, erstes byte: "));
      //Serial.println((int)buffer[0]);
  //    for (int i=0; i<16; i++) 
  //    {
  //       int b = buffer[0] & (1 << i);
 //        Serial.print((int)buffer[i]);
 //        Serial.print("\t");
 //        digitalWrite(i, b);
  //    }
      //Serial.println();
      uint16_t hb = (uint16_t)buffer[4];
      uint16_t lb = (uint16_t)buffer[5];
 //     Serial.print(hb);
 //     Serial.print("\t");
 //     Serial.print(lb);
 //     Serial.println();
      
      pot0 = hb <<8 | lb;
      analogWrite(5, pot0 + STARTOFFSET);
      
      pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
      analogWrite(6, pot1 + STARTOFFSET);
      //pot0 = (buffer[4])<<8 + buffer[5];
      Serial.print("Pot0: ");
      Serial.print((int)pot0);
      Serial.print("\t");
      Serial.print((int)pot1);
//           Serial.print(buffer[4]);
 //          Serial.print("\t");
 //         Serial.print(buffer[5]);
      //    Serial.print("\t");
      
    
      Serial.println();
   }
   // every 2 seconds, send a packet to the computer
   if (msUntilNextSend > 2000) {
      msUntilNextSend = msUntilNextSend - 2000;
      // first 2 bytes are a signature
      buffer[0] = 0xAB;
      buffer[1] = 0xCD;
      // next 24 bytes are analog measurements
      for (int i=0; i<12; i++) {
         int val = analogRead(i);
         buffer[i * 2 + 2] = highByte(val);
         buffer[i * 2 + 3] = lowByte(val);
      }
      // fill the rest with zeros
      for (int i=26; i<62; i++) {
         buffer[i] = 0;
      }
      // and put a count of packets sent at the end
      buffer[62] = highByte(packetCount);
      buffer[63] = lowByte(packetCount);
      // actually send the packet
      n = RawHID.send(buffer, 100);
      if (n > 0) {
         Serial.print(F("Transmit packet "));
         Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
   }
} // loop
