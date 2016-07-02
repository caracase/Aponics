/*   This sketch is designed to control an aquaponics prototype 
 *   which regulates fish tank water temperature and turns a light
 *   on and off in response to light levels.
 *   
 *   The temperature control portion of the sketch takes 
 *   temperature measurements from a temperature sensor and
 *   triggers a 100 watt aquarium heater to turn on when the 
 *   temperature drifts too low.  The five most recent 
 *   temperature values (in Fahrenheit) will be averaged together 
 *   to help smooth erratic measurements from the sensor.
 *   
 *   The light control portion of the sketch receives an analog
 *   input signal voltage between 0 and +5 VDC from a photoresistor, 
 *   which is given as a number between 0 and 1023.  When the light 
 *   level yields a value greater than 200, the light turns on. When
 *   the value is less than 200, it turns off.  Once per hour the 
 *   light will be turned off regardless of light level to break any
 *   positive feedback loops in which the lamp shines on the sensor
 *   and keeps it from turning off.
 *   
 *   This sketch is designed for use with the ENV-TMP temperature
 *   sensor, a photoresistor, and the Songle 2-relay module 
 *   (SRD-05VDC-SL-C). The relay is single-pole, double-throw, non-
 *   latching type, rated for switching 10 amps at 28VDC or 10 amps 
 *   at 125VAC. The temperature sensor is suitable for leaving 
 *   submersed in water indefinitely, ideal for aquarium, aquaponics 
 *   or hydroponics applications.    
*/

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

char auth[] = "f15ec80963e34d428a75950062a49a51";

// or Software Serial on Uno, Nano...
#include <SoftwareSerial.h>
SoftwareSerial EspSerial(8, 9); // RX, TX

// Your ESP8266 baud rate:
  #define ESP8266_BAUD 9600
  #define Relay1 6              //Arduino Digital I/O pin number
  #define Relay2 7
  #define Relay_ON 0
  #define Relay_OFF 1

  #define PIN_TEMPSET V0
  #define PIN_AVGTEMPF V1
  #define PIN_LIGHTLEVEL V2

  WidgetLED lightStatusLED(V3);

  ESP8266 wifi(&EspSerial);

const int tempSetPoint = 65;    //set temp threshold in degrees F

const int temperaturePin = A0;
const int photoPin = A1;

//define temperature array with 5 positions, and initialize variables:
float smoothing[] = {66,66,66,66,66};
float avgTempF; 

// Declare a global variable for light level. Notice that
// high light level has the lower resistance, so there
// will be less of a voltage drop in light conditions.
int lightLevel; // high = 0, low = 1023

// Declare a global variable to increment while the light is on
// to prevent a positive feedback loop:
int lightTimer = 0;

long looptimer = 0;


void setup() {
  
  // Keep relays OFF or inactive during initialization
  digitalWrite(Relay1, Relay_OFF);     
  digitalWrite(Relay2, Relay_OFF);     
  
  // Set relay pins as OUTPUT
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  
  // Set console baud rate
  Serial.begin(9600);
  delay(10);
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(auth, wifi, "apartment", "albopilosum");

  // Check that all relays are inactive at Reset
  delay(4000);
}


void loop() {

  Blynk.run();

  // Items inside this if only run every 2.5s
  if ((millis() - looptimer) >= 2500){
    looptimer = millis();
    
    // get current average degrees F:
    avgTempF = (getSmoothF(temperaturePin));
  
    // Call function for reading light level
    lightLevel = getLightLevel(photoPin);
  
    // Turn relay1 on/off based on average temp F:    
    if(avgTempF < tempSetPoint)
    {
      digitalWrite(Relay1,Relay_ON);      //turn ON relay 1
    }
  
    if(avgTempF > (tempSetPoint + 1))  
    {
      digitalWrite(Relay1,Relay_OFF);   //turn OFF relay 1
    } 
    
  
    // If the light level is below a set point, turn LED off.
    if(lightLevel < 200)
    {
      digitalWrite(Relay2,Relay_OFF);       // turn OFF relay 2
      lightStatusLED.off();
    }
    else
    {
      digitalWrite(Relay2,Relay_ON);        // turn ON relay 2
      lightStatusLED.on();
    }

    Blynk.virtualWrite(PIN_TEMPSET, tempSetPoint);
    Blynk.virtualWrite(PIN_AVGTEMPF, avgTempF);
    int percentLight = 100 * (lightLevel / 1023.0);
    Blynk.virtualWrite(PIN_LIGHTLEVEL, percentLight);
      
  }//delayed loop
  
} //loop


//float getTempVoltage(int pin)
//{

  //return (analogRead(pin) * 0.004882814);

  //This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.

//}

float getTempC(int pin)
{
  //establish two floating-point variables:
  float tempVoltage, degreesC;

  tempVoltage = analogRead(pin) * 0.004882814;

  //convert voltage to Celsius (see tmp sensor data sheet);
  //Uncomment the appropriate equation for the specific sensor:
  // degreesC = (voltage - 0.5) * 100.0;   //for TMP36 (Sparkfun)
  // degreesC = (voltage - 0.6) * 100.0;   //for LM61 (National Semiconductor)
     degreesC = tempVoltage * 51.2 - 20.5128;  //for ENV-TMP (Atlas Scientific)

  return degreesC; 
}

float getTempF(int pin)             
{
  float degreesC = getTempC(pin);
  
  //convert Celsius to Fahrenheit:
  float degreesF = degreesC * (9.0/5.0) + 32.0;

  return degreesF;
}

float getSmoothF(int pin)
{
  // shift all temp values up in index:
  smoothing[4] = smoothing[3];
  smoothing[3] = smoothing[2];
  smoothing[2] = smoothing[1];
  smoothing[1] = smoothing[0];

  // add new temperature reading to smoothing[0]:
  smoothing[0] = getTempF(pin);

  // compute average of Fahrenheit readings:
  float sum = (smoothing[0]+smoothing[1]+
  smoothing[2]+smoothing[3]+smoothing[4]);
  float smoothF = sum / 5;

  return smoothF;
}

int getLightLevel(int pin)
{
  int lightReading = analogRead(pin);
  float photoVoltage = lightReading * 0.004882814;

  ++lightTimer;
  if(lightTimer >= 720)    // 720 x 5 second delay from void(loop) = 1 hour check
  {
    // turn off light to break any positive feedback loop:
    digitalWrite(Relay2,Relay_OFF);

    // delay to allow photosensor to respond:
    delay(4000);

    // set light increment timer back to zero:
    lightTimer = 0;
  }

  // report light level to main/loop function:
  return lightReading;
}
  






