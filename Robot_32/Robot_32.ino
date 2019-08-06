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

#include "rRingbuffer.c"
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

#define ACHSE1_START  0x680 // Startwert low
#define ACHSE1_MAX  0xFFF // Startwert high

uint16_t achse1_start = ACHSE1_START;
uint16_t achse1_max = ACHSE1_MAX;


#define  ACHSE2_BYTE_H  16
#define  ACHSE2_BYTE_L  17

#define ACHSE2_START  0x680 // Startwert low
#define ACHSE2_MAX  0xFFF // Startwert high

uint16_t achse2_start = ACHSE2_START;
uint16_t achse2_max = ACHSE2_MAX;

#define ACHSE3_START  0x780 // Startwert low
#define ACHSE3_MAX  0xFFF // Startwert high

#define  ACHSE3_BYTE_H  18
#define  ACHSE3_BYTE_L  19

uint16_t achse3_start = ACHSE3_START;
uint16_t achse3_max = ACHSE3_MAX;

#define   HYP_BYTE_H  22 // Hypotenuse
#define   HYP_BYTE_L  23

#define   INDEX_BYTE_H  24
#define   INDEX_BYTE_L  25

#define   STEPS_BYTE_H  26
#define   STEPS_BYTE_L  27


#define SET_0     0xA1
#define GOTO_0    0xAA
#define SET_1   0xB1
#define GOTO_1 0xBA
#define SET_2   0xC1
#define GOTO_2 0xCA
#define SET_3   0xD1
#define GOTO_3 0xDA
#define SET_ROB 0xA2
#define SET_RING  0xA3
#define CLEAR_RING  0xA4


//let GET_U:UInt8 = 0xA2
//let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;


float sinpos = 0;
#define pi 3.14
#define SIN_START   0xE0
#define SIN_STOP   0xE1


#define LOOPLED 13

char* buffercode[4] = {"BUFFER_FAIL","BUFFER_SUCCESS", "BUFFER_FULL", "BUFFER_EMPTY"};

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities
Buffer rb;
elapsedMillis sinceringbuffer;

uint16_t schrittecount = 0;
uint16_t anzschritte = 0;
uint16_t schrittweite = 32;
uint16_t wegindex = 0;

robotposition aktuellepos;
float deltax;
float deltay;
float deltaz;
robotposition lastpos;

uint16_t abschnittindex = 0; // aktuelles Element in positionsarray

uint8_t wegstatus = 0; // 
#define WEG_OK 1
#define WEG_END   2
#define WEG_ERR   7
// Functions
uint8_t ringbufferIn(struct robotposition pos);
uint8_t ringbufferClear(void);


/*
uint8_t ringbufferIn(struct robotposition pos)
{
   uint8_t next = ((ringbuffer.write + 1) & BUFFER_MASK);
   if (ringbuffer.read == next)
      return BUFFER_FAIL; // voll
   ringbuffer.position[ringbuffer.write] = pos;
   ringbuffer.write = next;
   return BUFFER_SUCCESS;
}
*/

struct robotposition erstepos = {0,0,0,0,0,0};


void printHex8(uint8_t data) // prints 8-bit data in hex with leading zeroes
{
   Serial.print("0x"); 
  // for (int i=0; i<length; i++) 
   { 
      if (data<0x10) {Serial.print("0");} 
      Serial.print(data,HEX); 
      Serial.println(" "); 
   }
}


// Add setup code
void setup()
{
   Serial.begin(9600);
   analogWriteResolution(16); // 32767
   
   /*
    struct robotposition
    {
    uint16_t x;
    uint16_t y;
    uint16_t z;
   
    uint16_t hyp;
   uint16_t index;
    };
    
    */
   
   
   ringbuffer.position[0].x = 0;
   ringbuffer.position[0].y = 0;
   ringbuffer.position[0].z = 0;
   ringbuffer.position[0].hyp = 0;
   ringbuffer.write = 0;
   ringbuffer.read = 0;
   ringbuffer.position[0].task = 0;
   
     
   /*
   uint8_t erfolg = ringbufferIn(erstepos);
   
   uint16_t anz = ringbufferCount();
   Serial.print("erfolg: ");
   Serial.print(erfolg);
   Serial.print(" anzahl: ");
   Serial.print(anz);
   */
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
   #define  ACHSE3_BYTE_H  18
   #define  ACHSE3_BYTE_L  19

   #define  achse0_PIN 5
   #define  achse1_PIN 6
   #define  achse2_PIN 9
   #define  achse3_PIN 10
   
   pinMode(achse0_PIN, OUTPUT);
   pinMode(achse1_PIN, OUTPUT);
   
   pinMode(achse3_PIN, OUTPUT);
    
//   analogWrite(achse0_PIN, 0x700 + achse0_start);
   analogWrite(achse0_PIN,0xCCC + achse0_start); // Mitte
   
//   analogWrite(achse1_PIN, 0x700 + achse1_start);
   analogWrite(achse1_PIN, 0xCCC + achse1_start); // Mitte
   
 //  analogWrite(achse3_PIN, 0xCCC + achse3_start);
   
}

// Add loop code
void loop()
{
   if (sinceblink > 1000)
   {
      sinceblink = 0;
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));
      
      /*
      uint8_t erfolg = ringbufferIn(erstepos);
      
      uint16_t anz = ringbufferCount();
      Serial.print("erfolg: ");
      Serial.print(erfolg);
      Serial.print(" anzahl: ");
      Serial.print(anz);
      Serial.print(" read: ");
      Serial.print(ringbuffer.read);
      Serial.print(" write: ");
      Serial.println(ringbuffer.write);
      */
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
      Serial.println(" ");
      Serial.print("******************  usbtask *** ");
      printHex8(usbtask);
      
      //usbtask = SET_0;
      switch (usbtask)
      {
         case SET_0: // data
         {
            teensytask = 0;
            pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
            analogWrite(achse0_PIN, pot0 + achse0_start);
            
   //         pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
   //         analogWrite(achse1_PIN, pot1 + achse0_start);
            //pot0 = (buffer[4])<<8 + buffer[5];
            
            Serial.print("Pot 0: ");
            Serial.print((int)pot0);
            Serial.print("\t");
//            Serial.print("Pot 1: ");
//            Serial.print((int)pot1);
            
            //    Serial.print(buffer[4]);
            //    Serial.print("\t");
            //    Serial.print(buffer[5]);
            //    Serial.print("\t");
            Serial.println();

         }break;
            
         case GOTO_0:
         {
            uint16_t goto0x = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
            analogWrite(achse0_PIN, goto0x + achse0_start);

            uint16_t goto0y = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
            analogWrite(achse1_PIN, goto0y + achse1_start);
           
            
            // achse0_PIN
  
            
         }break;
 
#pragma mark SET_RING
         case SET_RING:
         {
            /*
             struct robotposition
             {
             uint16_t x;
             uint16_t y;
             uint16_t z;
             uint16_t hyp;
             uint16_t index;
             uint8_t usbtask;
             };
             
             */
            Serial.println(" SET_RING ");
            struct robotposition p;
            p.x = (uint16_t)buffer[ACHSE0_BYTE_H] << 8 | (uint16_t)buffer[ACHSE0_BYTE_L];
            p.y = (uint16_t)buffer[ACHSE1_BYTE_H] << 8 | (uint16_t)buffer[ACHSE1_BYTE_L];
    //        p.z = (uint16_t)buffer[ACHSE2_BYTE_H] << 8 | (uint16_t)buffer[ACHSE2_BYTE_L];
            p.task = SET_RING; 
            // HYP_BYTE_H
            p.hyp = (uint16_t)buffer[HYP_BYTE_H] << 8 | (uint16_t)buffer[HYP_BYTE_L];
            
            p.steps = (uint16_t)buffer[STEPS_BYTE_H] << 8 | (uint16_t)buffer[STEPS_BYTE_L];
            
            // Anzahl Pfadelemente im Bezierpath
            p.index = (uint16_t)buffer[INDEX_BYTE_H] << 8 | (uint16_t)buffer[INDEX_BYTE_L];
            
            // pos in ringbuffer
            Serial.println(" >>>>>>>>>>>>>>>>>>>>>>>>> vor  ringbufferIn");
             Serial.print(" read: ");
            Serial.print(ringbuffer.read);
            Serial.print(" write: "); // VOR ringbufferIn
            Serial.print(ringbuffer.write);
            Serial.print(" index: ");
            Serial.print(p.index);
            Serial.print(" p steps: ");
            Serial.print(p.steps);

            Serial.print(" hyp: ");
            Serial.println(p.hyp);

            // einsetzen in ringbuffer
            uint8_t erfolg = ringbufferIn(p); 
            
             Serial.println(" >>>>>>>>>>>>>>>>>>>>>>>>> nach  ringbufferIn");
            Serial.print(" in erfolg: ");
            Serial.print(buffercode[erfolg]);
            
            //Serial.print(" nach  ringbufferIn");
            uint16_t anz = ringbufferCount();
            
           
            Serial.print(" anzahl: ");
            Serial.print(anz);
            Serial.print(" read: ");
            Serial.print(ringbuffer.read);
            Serial.print(" write: "); // NACH ringbufferIn
            Serial.print(ringbuffer.write); // = ringbuffer.write
            
  
            //           if (ringbuffer.write > 1) // mindestens zwei Punkte: Zwischenschritte berechnen
            
            // Schritte berechnen
            
            if (p.hyp == 0) // Start Polynom
            {
               wegstatus = 0;
               Serial.println("");
               Serial.print("********************   ");
               Serial.print("ringbuffer start");
               // pos 0 lesen: lastx Anfang
               uint8_t erfolg = ringbufferOut(&lastpos);
               Serial.print(" out erfolg: ");
               Serial.print(buffercode[erfolg]);
               if (erfolg == 1)
               {
                  wegstatus |= (1<<WEG_OK);
               }
              else
              {
                 wegstatus |= (1<<WEG_ERR);
              }
               uint16_t lastx = lastpos.x;
               uint16_t lasty = lastpos.y;
               uint16_t lastz = lastpos.z;
               uint16_t lasthyp = lastpos.hyp;
               
               wegindex = lastpos.index;
               
               abschnittindex  = lastpos.index;
               
               uint16_t lastanzsteps = lastpos.steps;
               
               
               Serial.print(" last x: ");
               Serial.print(lastx);
               Serial.print(" last y: ");
               Serial.print(lasty);
               Serial.print(" last z: ");
               Serial.print(lastz);
               Serial.print(" lasthyp: ");
               Serial.print(lasthyp);
               Serial.print(" lastanzsteps: ");
               Serial.println(lastanzsteps);
 
               /*
               Serial.println("hyp=0");
               Serial.print(" ****************** read: ");
               Serial.print(ringbuffer.read);
               Serial.print(" ****************** write: ");
               Serial.println(ringbuffer.write);
               
               
               
               */
 //              Serial.print(" abschnittindex: ");
 //              Serial.println(abschnittindex);
               
               

               
               //lastpos = ringbuffer.position[0];
               
               analogWrite(achse0_PIN, lastx + achse0_start);
               analogWrite(achse1_PIN, lasty + achse1_start);
    //           analogWrite(achse2_PIN, lastz + achse2_start);
               schrittecount = 0;
            }
            else
               
            {
               Serial.println("");
               Serial.print("============================   ");
               Serial.println("ringbuffer set punkt");
               //anzschritte = p.hyp / schrittweite;
               uint16_t lastx = ringbuffer.position[ringbuffer.write-2].x;
               uint16_t lasty = ringbuffer.position[ringbuffer.write-2].y;
               uint16_t lastz = 0;//ringbuffer.position[ringbuffer.write-2].z;
               uint16_t lastelementindex = ringbuffer.position[ringbuffer.write-2].index;
               
               uint16_t newx = ringbuffer.position[ringbuffer.write-1].x; // neuer Wert ist schon in ringbuffer, write ist incrementiert
               uint16_t newy = ringbuffer.position[ringbuffer.write-1].y;
               uint16_t newz = 0;//ringbuffer.position[ringbuffer.write-1].z;
               
               uint16_t newelementindex = ringbuffer.position[ringbuffer.write-1].index;
      //         wegindex = lastpos.index;
     
               Serial.print("last x: ");
               Serial.print(lastx);
               Serial.print(" last y: ");
               Serial.print(lasty);
               Serial.print(" last z: ");
               Serial.print(lastz);
               Serial.print(" lastelementindex: ");
               Serial.print(lastelementindex);
               
               Serial.print(" * new x : ");
               Serial.print(newx);
               Serial.print(" new y: ");
               Serial.print(newy);
               Serial.print(" new z: ");
               Serial.print(newz);
               Serial.print(" newelementindex: ");
               Serial.print(newelementindex);
               
               /*
                // new
               Serial.print("x: ");
               Serial.print(p.x);
               Serial.print(" y: ");
               Serial.print(p.y);
               Serial.print(" z: ");
               Serial.println(p.z);
               */
 
               Serial.print(" hypotenuse: ");
               Serial.print(p.hyp);

               Serial.print(" *** usbtask: ");
               Serial.print(usbtask);
               
               Serial.print(" p anzschritte: ");
               Serial.println(p.steps);
               anzschritte = p.steps;
               /*
               Serial.println("hyp  not 0");
               Serial.print(" ****************** read: ");
               Serial.print(ringbuffer.read);
               Serial.print(" ****************** write: ");
               Serial.println(ringbuffer.write);
                */
            }
            
            
         }break;

#pragma mark CLEAR_RING            
         case CLEAR_RING:
         {
            ringbufferClear();
 
            //schrittecount = 0;
            deltax = 0;
            deltay = 0;
            deltaz = 0;
            
            anzschritte = 0;
            
            /*
            struct robotposition p;
            p.x = (uint16_t)buffer[ACHSE0_BYTE_H] << 8 | (uint16_t)buffer[ACHSE0_BYTE_L];
            p.y = (uint16_t)buffer[ACHSE1_BYTE_H] << 8 | (uint16_t)buffer[ACHSE1_BYTE_L];
            p.z = (uint16_t)buffer[ACHSE2_BYTE_H] << 8 | (uint16_t)buffer[ACHSE2_BYTE_L];
            
            // HYP_BYTE_H
            p.hyp = (uint16_t)buffer[HYP_BYTE_H] << 8 | (uint16_t)buffer[HYP_BYTE_L];
            
            // Position in Ringbuffer
            p.index = (uint16_t)buffer[INDEX_BYTE_H] << 8 | (uint16_t)buffer[INDEX_BYTE_L];
   //         lastpos = p;
  //          aktuellepos = p;
            // pos in ringbuffer
            
            //
            
            ringbuffer.position[0].x = 0;
            ringbuffer.position[0].y = 0;
            ringbuffer.position[0].z = 0;
            ringbuffer.position[0].hyp = 0;
            ringbuffer.write = 0;
            ringbuffer.read = 0;
            ringbuffer.position[0].task = 0;

            //
    //        uint8_t erfolg = ringbufferIn(p); //
             
            
            uint16_t anz = ringbufferCount();
            Serial.print("ringbuffer clear anz: ");
            Serial.print(anz);
            Serial.print(" p x: ");
            Serial.print(p.x);
            Serial.print(" p y: ");
            Serial.print(p.y);
            Serial.print(" p z: ");
            Serial.print(p.z);
             */
          
         }break;
            
         case SET_1: // data
         {
            teensytask = 0;
  //          pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
  //          analogWrite(5, pot0 + achse0_start);
            
            pot1 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
            analogWrite(6, pot1 + achse1_start);

            pot2 = (uint16_t)buffer[6] << 8 | (uint16_t)buffer[7];
    //        analogWrite(6, pot2 + achse1_start);
            //pot0 = (buffer[4])<<8 + buffer[5];
            
  //          Serial.print("Pot 0: ");
 //           Serial.print((int)pot0);
 //           Serial.print("\t");
            Serial.print("Pot 1: ");
            Serial.print((int)pot1);
            
            //    Serial.print(buffer[4]);
            //    Serial.print("\t");
            //    Serial.print(buffer[5]);
            //    Serial.print("\t");
            Serial.println();
            
         }break;

         case SET_3: // data
         {
            teensytask = 0;
            //          pot0 = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];
            //          analogWrite(5, pot0 + achse0_start);
            
            pot3 = (uint16_t)buffer[ACHSE3_BYTE_H] << 8 | (uint16_t)buffer[ACHSE3_BYTE_L];
            analogWrite(achse3_PIN, pot3 + achse3_start);
            //pot0 = (buffer[4])<<8 + buffer[5];
            
            Serial.print("Pot 3: ");
            Serial.print((int)pot3);
            
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
      if (sinms > 5)
      {
         sinms = 0;
         //float tempsin = 0x800 + achse0_start + 1000 * sin(sinpos/180*pi);
        float tempsin0 = 0xCCC + achse0_start   + 0x400 * sin(0.5*sinpos/180*pi);
         Serial.print("Sin Pot0: ");
         //Serial.print((int)sinpos);
         Serial.print("\t");
         Serial.print((int)tempsin0);
         
         
         analogWrite(6, (int)tempsin0);
         
         float tempsin1 = 0xCCC + achse0_start + 0x400 * sin((0.2*sinpos)/180*pi);
         analogWrite(achse3_PIN, (int)tempsin1);
         Serial.print("\t");
         Serial.print((int)tempsin1);
      
         Serial.println();
         sinpos += 1;
         
      }
   }
#pragma mark sinceringbuffer    
   if (sinceringbuffer > 32) // naechster Schritt
   {
      sinceringbuffer = 0;
      //     Serial.print(" abschnittindex: ");
      //     Serial.print(abschnittindex);
      
      //     Serial.print(" aktuellepos index: ");
      //    Serial.println(aktuellepos.index);
      
      //if (schrittecount == 0)
      if (usbtask == SET_RING)
      {
          //    Serial.print(" abschnittindex: ");
          //    Serial.print(abschnittindex);
         
          //    Serial.print(" aktuellepos index: ");
          //   Serial.println(aktuellepos.index);
     //    Serial.print(" lastpos steps: ");
     //    Serial.print(lastpos.steps);
    //    Serial.print(" aktuellepos steps: ");
    //     Serial.println(aktuellepos.steps);

         if ((schrittecount < anzschritte )   && (wegstatus & (1<<WEG_OK)))//&& ((abschnittindex+1) == aktuellepos.index))
      //   if ((schrittecount < lastpos.steps ) )//  && (wegstatus & (1<<WEG_OK)))//&& ((abschnittindex+1) == aktuellepos.index))
         {
            
            Serial.print(" abschnittindex: ");
            Serial.print(abschnittindex);
            
            Serial.print(" aktuellepos index: ");
            Serial.println(aktuellepos.index);
            
             Serial.print(" schrittecount: ");
             Serial.print(schrittecount);
             Serial.print(" anzschritte: ");
             Serial.println(anzschritte);
             
             
             
             Serial.println("loop A");
             Serial.print(" ****************** read: ");
             Serial.print(ringbuffer.read);
             Serial.print(" ****************** write: ");
             Serial.println(ringbuffer.write);
             
            if (schrittecount == 0)
            {
               uint8_t erfolg = ringbufferOut(&aktuellepos);
               Serial.print("\n *** schrittecount = 0 out-erfolg: ");
               Serial.println(buffercode[erfolg]);
               if ((erfolg == 1))
               {
                  wegstatus |= (1<<WEG_OK);
                  
                  
                  
                  Serial.print(" aktuellepos x: ");
                  Serial.print(aktuellepos.x);
                  Serial.print(" aktuellepos y: ");
                  Serial.print(aktuellepos.y);
                  Serial.print(" aktuellepos z: ");
                  Serial.print(aktuellepos.z);
                  Serial.print(" aktuellepos hyp: ");
                  Serial.print(aktuellepos.hyp);
                  Serial.print(" aktuellepos task: ");
                  Serial.print(aktuellepos.task);
                  anzschritte = aktuellepos.hyp / schrittweite;
                  Serial.print(" anzschritte: ");
                  Serial.print(anzschritte);
                  Serial.print(" aktuellepos steps: ");
                  Serial.print(aktuellepos.steps);
                  
                  
                  Serial.print(" aktuellepos index: ");
                  Serial.println(aktuellepos.index);
                  
                  
                  // Schrittweiten:
                  deltax = float(aktuellepos.x - lastpos.x) / anzschritte;
                  deltay = float(aktuellepos.y - lastpos.y) / anzschritte;
                  deltaz = float(aktuellepos.z - lastpos.z) / anzschritte;
                  Serial.print(" delta x: ");
                  Serial.print(deltax);
                  Serial.print(" delta y: ");
                  Serial.print(deltay);
                  Serial.print(" delta z: ");
                  Serial.println(deltaz);
               }
               else
               {
                  wegstatus &= ~(1<<WEG_OK);
                  wegstatus |= (1<<WEG_END);
                  usbtask == END_TASK;
                  
               }
               
            }
            
            schrittecount++;
            
      //      if ((schrittecount < 5) || (schrittecount > (anzschritte - 5)))
            {
               //        Serial.println("");
               //Serial.print("ringbuffer cont schrittecount: ");
               Serial.print(schrittecount);
               //        Serial.print(" lastpos x: ");
               //        Serial.print(lastpos.x);
               //        Serial.print(" lastpos y: ");
               //        Serial.print(lastpos.y);
               //        Serial.print(" lastpos z: ");
               //        Serial.print(lastpos.z);
               
               //        Serial.print("lastpos hyp: ");
               
               //         Serial.print(lastpos.hyp);
               Serial.print("\tlpos index:\t");
               Serial.print(lastpos.index);
               Serial.print(" \ttask:\t");
               Serial.print(lastpos.task);
               
               
               //      robotposition temppos = {lastpos.x + schrittecount * deltax, lastpos.y + schrittecount * deltay, lastpos.z + schrittecount * deltaz,}
               
               
               
               //        Serial.println("***");
               Serial.print("\tdx:\t");
               Serial.print(lastpos.x + schrittecount * deltax);
               Serial.print("\tdy:\t");
               Serial.print(lastpos.y + schrittecount * deltay);
               Serial.print("\tdz:\t");
               Serial.println(lastpos.z + schrittecount * deltaz);
               
                
            }
            
            analogWrite(achse0_PIN, lastpos.x + schrittecount * deltax + achse0_start);
            analogWrite(achse1_PIN, lastpos.y + schrittecount * deltay + achse1_start);
            //      analogWrite(achse2_PIN, lastpos.z + schrittecount * deltaz + achse2_start);
            
           // if ((schrittecount == lastpos.steps) && (wegstatus & (1<<WEG_OK)))
            if ((schrittecount == anzschritte) && (wegstatus & (1<<WEG_OK)))
            {
               lastpos = aktuellepos;
               Serial.print(" ende Element  ");
               Serial.println(lastpos.index);
               
               Serial.print(" lastpos daten  ");
               Serial.print(" lastpos x: ");
               Serial.print(lastpos.x);
               Serial.print(" lastpos y: ");
               Serial.print(lastpos.y);
               Serial.print(" lastpos z: ");
               Serial.print(lastpos.z);
               Serial.print(" lastpos hyp: ");
               Serial.print(lastpos.hyp);
               Serial.print(" lastpos task: ");
               Serial.print(lastpos.task);
               Serial.print(" *** lastpos steps: ");
               Serial.print(lastpos.steps);

               Serial.print(" lastpos index: ");
               Serial.println(lastpos.index);
               
               
               
               /*
               uint8_t erfolg = ringbufferOut(&aktuellepos);
               Serial.print("\n *** schrittecount = 0 erfolg: ");
               Serial.println(buffercode[erfolg]);
               Serial.print(" aktuellepos daten  ");
               Serial.print(" aktuellepos x: ");
               Serial.print(aktuellepos.x);
               Serial.print(" aktuellepos y: ");
               Serial.print(aktuellepos.y);
               Serial.print(" aktuellepos z: ");
               Serial.print(aktuellepos.z);
               Serial.print(" aktuellepos hyp: ");
               Serial.print(aktuellepos.hyp);
               Serial.print(" aktuellepos task: ");
               Serial.print(aktuellepos.task);
               
               Serial.print(" aktuellepos index: ");
               Serial.println(aktuellepos.index);
               
               anzschritte = aktuellepos.hyp / schrittweite;
               */
               anzschritte = lastpos.steps;
               schrittecount = 0;
               
            }
            
         }
         else
         {
 //           Serial.print(" ende element schrittecount: ");
 //           Serial.println(schrittecount);
 //           schrittecount = 0;
            
            /*
             robotposition r;
             uint8_t erfolg = ringbufferOut(&r);
             
             Serial.print("erfolg: ");
             Serial.print(erfolg);
             Serial.print("x: ");
             Serial.print(r.x);
             Serial.print(" y: ");
             Serial.print(r.y);
             Serial.print(" hyp: ");
             Serial.print(r.hyp);
             Serial.print(" index: ");
             Serial.println(r.index);
             */
         }
      }
      
   }
   
   // every 4 seconds, send a packet to the computer
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
 //        Serial.print(F("Transmit packet "));
 //        Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
   }
} // loop
