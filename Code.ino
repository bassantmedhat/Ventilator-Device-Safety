#include <Wire.h>
#include <SFE_BMP180.h>
SFE_BMP180 bmp180;
#include "DHT.h"//calling library of DHT 
#define DHTPIN 6 // connevting the DHT to bin num 2 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#include <LiquidCrystal.h>
// lcd
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// mobile connection
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 1); // RX, TX
// temp
int tempPin = A3;   
int read_ADC= 0;
double temp = 0;
double tempC = 0;


// alarm
int buzzer=10;
int ledR =8;
int ledB =9;
int ledG =13;
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600); 
  mySerial.println("Ready");
   dht.begin();
   lcd.begin(16,2);  
   pinMode(tempPin, INPUT);
  pinMode (ledR, OUTPUT);
  pinMode (ledB, OUTPUT);
  pinMode (ledG, OUTPUT);
   bool success = bmp180.begin();
  if (success) {
    Serial.println("BMP180 init success");
  }
}
void loop() {
// tempreure calculations
read_ADC =analogRead(tempPin); 
temp = (read_ADC / 1023.0) * 5000; // 5000 to get millivots.
tempC = (temp*0.1);

// measure humidity 
float h = dht.readHumidity();

// measure air pressure
  char status;
  double T, P;
  bool success = false;

  status = bmp180.startTemperature();

  if (status != 0) {
    delay(1000);
    status = bmp180.getTemperature(T);

    if (status != 0) {
      status = bmp180.startPressure(3);

      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);

        if (status != 0) {
          Serial.print("Pressure: ");
          Serial.print(P);
          Serial.println(" hPa");
        }
      }
    }
  }
// lcd printing setup
if(h >=40 && tempC <38 && P<1016) {
lcd.setCursor(0,0);    
lcd.println("Normal"); 
digitalWrite(ledR, LOW); 
digitalWrite(ledG, LOW); 
digitalWrite(ledB, LOW); 
Serial.println("Normal");
mySerial.print("Normal"); 

}

// condtions for any Dangerous situation

if(tempC>=38) {

 tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(1000);        // ...for 1sec
    digitalWrite(ledG,HIGH);
    
   lcd.setCursor(0,0);
   lcd.print("Danger");
   Serial.print("Danger");
   mySerial.print("Danger");
  
}
if( h < 36){
 tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(1000);        // ...for 1sec
    digitalWrite(ledB, HIGH);
    lcd.setCursor(0,1);
   lcd.print("Danger");
   Serial.print("Danger");
   mySerial.print("Danger");

}
if(P>1016){ 
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(1000);        // ...for 1sec
    digitalWrite(ledR, HIGH);
    lcd.setCursor(0,0);
   lcd.print("Danger");
   Serial.print("Danger");
   mySerial.print("Danger");
  }
// mySerial.print("S");
// mySerial.print(";");
mySerial.print(tempC); //send distance to MIT App
mySerial.print(";");
mySerial.print(h); //send distance to MIT App
mySerial.println(";");
mySerial.print(P); //send distance to MIT App
mySerial.println(";");
Serial.print("tempreture");
Serial.println(tempC);
Serial.print ("Humidity: ");
Serial.println (h);
delay(1000);
}
