/**********************************************************************************************************************
 *  HOME AUTOMATION PROJECT
 *  JECRC University
 *  by: Jitender Singh Virk, Dhawal Mathur, Chanchal Nama, Kamal Kumar Tripathi
 *      3rd year, 6th semester, B.Tech, CSE A2.
 **********************************************************************************************************************/

/* 16x2 LCD connection to Arduino uno
 * The circuit:
   * LCD VSS pin to ground
   * LCD VDD pin to 5V    
   * LCD V0 pin to center pin of potentiometer
   * LCD RS pin to digital pin 12
   * LCD R/W pin to ground
   * LCD E(Enable) pin to digital pin 11
   * LCD D4 pin to digital pin 5
   * LCD D5 pin to digital pin 4
   * LCD D6 pin to digital pin 3
   * LCD D7 pin to digital pin 2
   * LCD A pin to 5V with green resistor
   * LCD K pin to ground
   * Potentiometer: center pin to V0, one to ground and leave third not connected
 */
 
#include<LiquidCrystal.h>                          // LCD library
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);             // initialize the library with the numbers of the interface pins

int LEDPin = 7;                                    // Arduino Pin 7 is connected to +ve of LED
int buzzerPin = 6;                                 // Arduino Pin 6 is connected to out pin of Buzzer
int PIRPin = 10;                                   // Arduino Pin 8 is connected to out pin of PIR Sensor
int SoundPin = 9;                                  // Arduino Pin 9 is connected to out pin of Sound Sensor
int tempraturePin = A0;                            // Arduino pin A0 is connected to out pin of Temprature sensor
int gasPin = A1;                                   // Arduino pin A1 is connected to out pin of Gas sensor 
int lightPin = A2;                                 // Arduino pin A2 is connected to out pin of Light sensor 
int motionDetected = LOW;                          // Start MotionDetected as low (No motion detected)
int soundDetected = LOW;                           // Start SoundDetected as low (No Sound Detected)
//const int trigPin = 12;
//const int echoPin = 13;


void setup() 
{
    lcd.begin (16, 2);                             // set up the LCD's number of columns and rows:
    lcd.print ("HOME AUTOMATION");                 // Print a message to the LCD.
    Serial.begin (9600);                           // opens serial port, sets data rate to 9600 bps
    pinMode (LEDPin, OUTPUT);                      // initialize LEDPin 7 as output
    pinMode (buzzerPin, OUTPUT);                   // initialize BuzzerPin 6 as output
    pinMode (PIRPin, INPUT);                       // initialize PIRSensorPin 8 as input
    pinMode (SoundPin, INPUT);                     // initialize SoundSensorPin 9 as input
    delay (2000);                                  // Allow time for the PIR Sensor to callibrate
// pinMode(echoPin, INPUT);
// pinMode(trigPin, OUTPUT);
// Serial.begin(9600);
}

unsigned long motionSensor()
{   
    /* This code controls PIR Motion Sensor, Buzzer and a LED light:
     *  START-----------------------------------------------------------------------------------------------------
     */
   motionDetected = digitalRead (PIRPin);                 // Read the PIR sensor
    if (motionDetected == HIGH)                            // If motion detected
    {            
        digitalWrite (LEDPin, HIGH);
        analogWrite (buzzerPin, 250);                      // set buzzer sound intensity to 250(0 to 254)
        delay (100);                                  
        digitalWrite (LEDPin, LOW);
        analogWrite (buzzerPin, 25);                       // set buzzer sound intensity to 25(0 to 254)
        delay (100);
    }
    soundDetected = digitalRead (SoundPin);                // read the sound sensor
    if (soundDetected == HIGH)                             // if sound detected
    {
        digitalWrite (LEDPin, HIGH);
        analogWrite (buzzerPin, 25);
        delay (100);
        digitalWrite (LEDPin, LOW);
        analogWrite (buzzerPin, 50);
        delay (100);
        digitalWrite (LEDPin, HIGH);  
        analogWrite (buzzerPin, 75);
        delay (100);
        digitalWrite (LEDPin, LOW);
        analogWrite (buzzerPin, 100);
        delay (100);    
    }
    digitalWrite (LEDPin, LOW);
    digitalWrite (buzzerPin,LOW);
}
    /*  END--------------------------------------------------------------------------
     * 
     
     * Next code controls temprature sensor and display it on 16x2 LCD 
     *  START------------------------------------------------------------------------------------------------------ 
     */
     unsigned long tempSensor()
     {
    //Serial.println (analogRead (tempraturePin));
    int Tvalue = analogRead (tempraturePin);                // read from A0 analog pin 
    float volts = (Tvalue / 1024.0) * 5.0;                  // conversion to volts
    float tempC =  (volts * 100.0);                         // conversion to temp Celsius   
    lcd.setCursor (0,1);                                    // set the cursor to column 0, line 1
    lcd.print ("TEMP= ");
    lcd.print (tempC);
    lcd.print ("'C");
    delay (100);
     }
    /*  END--------------------------------------------------------------------------
     * 
     * Next code controls Gas sensor and Buzzer and lED will go ON when Gas is detected
     *  START------------------------------------------------------------------------------------------------------ 
     */
     unsigned long gasSensor()
   {
    //Serial.println (analogRead (gasPin));
    //delay (1000);                                         // Print value every 1 sec.
    if (analogRead (gasPin) > 160)
    {
        digitalWrite (LEDPin, HIGH);                        // LED ON
        analogWrite (buzzerPin, 250);                       // Buzzer ON with 250 sound intensity
        delay (200);
        analogWrite (buzzerPin, 100);                       // Buzzer ON with 100 sound intensity
        delay (200);
    }
    else
    {
        digitalWrite (LEDPin, LOW);                         // LED OFF
        digitalWrite (buzzerPin,LOW);                       // Buzzer OFF
    }
   }
    /*  END--------------------------------------------------------------------------
     * 
     * Next code controls Light sensor and Buzzer and LED will go ON when Light is detected
     *  START------------------------------------------------------------------------------------------------------ 
     */
    unsigned long lightSensor()
    {
     //Serial.println (analogRead (lightPin));
     int Lvalue = analogRead (lightPin);
     if (Lvalue > 850)
     {
         digitalWrite (LEDPin, HIGH);
         delay (300);
     }
     else
     {
         digitalWrite (LEDPin, LOW);
         digitalWrite (buzzerPin, LOW);
     }
    }
     /*  END--------------------------------------------------------------------------
     * 
     * Next code controls UltraSonic sensor
     *  START------------------------------------------------------------------------------------------------------ 
     */
 /*unsigned long ultraSound()
 {
   // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(1000);
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
*/

void loop()
{
    
    int ts = 0;
    ts = tempSensor();

    int ms = 0;
    ms = motionSensor();

    int gs = 0;
    gs = gasSensor();

    int ls = 0;
    ls = lightSensor();

    /*int us = 0;
    us = ultraSound();*/

  
}

