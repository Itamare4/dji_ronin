os#include <SPI.h>
byte address = 0x11;
uint8_t CS= 10;
uint8_t STOPANGLE = 77;
uint8_t angle,cmd,varspeed;

void setup() 
{ 
  Serial.begin(9600);
  Serial.println("###########################");
  Serial.println("Starting DJI Ronin M Gimbal");
  Serial.println("###########################");

  pinMode (CS, OUTPUT);
  SPI.begin();
  digitalPotWrite(STOPANGLE);
  
} 
 
void loop() 
{ 
 while (Serial.available()) {
    varspeed = Serial.parseInt();
    digitalPotWrite(STOPANGLE + varspeed);
    }
}

int digitalPotWrite(int value)
 {
   digitalWrite(CS, LOW);
   SPI.transfer(address);
   SPI.transfer(value);
   digitalWrite(CS, HIGH);
 }

