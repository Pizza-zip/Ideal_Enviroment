#include <dht.h>
#include <LiquidCrystal.h>

// set the DHT Pin
#define DHTPIN 7

dht DHT;				//dht is for our humidity and temp sensor which will measur air temp and humid.
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 8, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


#define moistpin A0		//this will defin pin for sensor we need
#define termopin A1
#define temphupin 7

extern volatile unsigned long timer0_millis;

//*** Time associated variable definitions and declarations
unsigned long CurrentTime;      // ms
unsigned long PreviousTime = 0; // ms
unsigned long DeltaTime;        // ms
unsigned long MaxTime = 1440;		//24hour is our max time
unsigned long Lighting = 840;		// light will be on for 14 h


const int lightpin = 3; // this is the pin that we are using for our system, which is hocked up on relay.
const int waterpin = 4;
const int humidpin = 5;
const int gatePin = 9; // PWM pin to control MOSFET

float waterVal;
float humd,temp,humdsum,tempsum;
float lighttime,lightoff;

//*** Constants 
const float Tset = 33.0;  // set temperature in degrees celcius (ideally between 40 and 65)
const float Band = 3.0;   // Temperature band for hysteresis: not used yet!
const float Tmax = 50.0;  // Setting a maximum temperature. For safety, heater will be turned off when exceeding this temperature.
const long R_series = 100000; // series resistance for thermistor's voltage divider

//*** Variable definitions  
float temp_average; // average measured temperature
float Vo; // ADC voltage (digital) measured across thermistor
float Rt, lnRt; // thermistor resistance, natural log of thermistor resistance
float T_kelvin, T_celsius; // temperature (Kelvin), temperature (degrees Celsius)

//*** Variable declarations
float temp_sum = 0.0; // sum of measured temperatures
float A = 0.7141095422e-3, B = 2.143243110e-4, C = 1.129790692e-7; // Steinhart-Hart coefficients

//*** Some variables that are not used now, but might become useful when you implement PID control
float Error;    // The error (Tset - temp_average)
float Previous_Error = 0.0; // necessary for the PID controller when calculating the integral and derivative
float Kp = 0.3; // proportional gain - you can  
float Ki = 0.00001; // integral gain, can be zero 
float Kd = 0.01; // derivative gain, can be zero
float Integral = 0.0; // Integral value: sum of the areas
float Derivative = 0.0; // Derivative value: slope
int Power;    // When implementing PID, Power will be calcuated and this value will be the PWM output between 0-255)
              // Power = (Kp*Error + Ki*Integral + Kd*Derivative) * 255;
float I; 
float Delta;             
int Px;
float lb = Tset - Band;		//lower and upper lim of our water temp
float ub = Tset + Band;
int counter = 0;
int light;
bool hyst = true;
int idk = 1;


void setup(){  
  
  lcd.begin(16, 2);
  Serial.begin(9600);
    
  pinMode(gatePin, OUTPUT);  
  pinMode(lightpin,OUTPUT);
  pinMode(waterpin,OUTPUT);
  pinMode(humidpin,OUTPUT);

  lcd.print("Temp:");

}


void loop(){				//most of our function is seperated by part and called in main loop
    tempsystem();
    lightcheck();  
    getMoist();
    getTempHu();
    humdsy();
    lcd.setCursor(0, 1);
    lcd.print(temp_average);
    printing();

  
}

void get_time(){		//this is our time control, we used the code that was given on last lab section then changed to fit in to our lab.
  
  CurrentTime = millis();     // Time elapsed (in milliseconds) since the beginning
  DeltaTime = CurrentTime - PreviousTime;  // This "might" be useful when you implement integral or derivative control!
  PreviousTime = CurrentTime; // ready for next loop!
   if (CurrentTime <= Lighting* 60 *1000){// this part will track the time and make sure that light is on
    light = 1;
   }
   if (CurrentTime >= Lighting* 60 *1000){// light will be off after 14h
    light = 0;
   }
    if(CurrentTime >= MaxTime * 60 * 1000){// time will be reset after 24h, 
      noInterrupts ();
      timer0_millis = 0;
      interrupts ();
      Serial.print("time has been rested");
      light = 0;
    }
}


void lightcheck(){		//this will turn the light on and off by looking on current time
  
 if (light ==1){		// the "light" value will change depend on time, and this value will be changed on our gettime function.
    digitalWrite(lightpin,HIGH);
    lighttime = CurrentTime;		//this used to show our light on time.
  }

  if (light ==0){		// light will be off
    delay(500);
    digitalWrite(lightpin,LOW);
    lightoff = CurrentTime - lighttime ;  //to show our light off time
  }
}



void getMoist(){		//this will measure moisture value
 for(int i = 0; i <= 100; i++){
   waterVal = waterVal + analogRead(moistpin);		//will measure 100 time and get an average value
   delay(100);
 }

 waterVal = waterVal/100.0;
 watercheck();		// calling watercheck function
 
}



void watercheck(){		

  if(waterVal >= 400){		// if soil moisture value is more then 400 (this value is resistance value between two metal inside our soil moisture sensor, higher = dry)
    pinMode(waterpin,OUTPUT);
    digitalWrite(waterpin,HIGH);
    delay(4000);			// will water the plant for 4 second, which means it will turn on the pump for 4 second.
    digitalWrite(waterpin,LOW);
  }

  else{
    delay(500);					//else it will be off
    pinMode(waterpin,OUTPUT);
    digitalWrite(waterpin,LOW); 
  }
}


void tempsystem(){		// this part of the code was from lab4, we twicked a bit but its basicaly same with our derivative PID from last lab.
  Previous_Error = Error;
  get_time();
  measureTemperature();   // This custom function is defined below. It measures the temperature and stores the average in the variable "temp_average"
  Error = (Tset - temp_average)/Band;

  dv(); 
    
  if (temp_average <= lb && idk == 1){
    Power = 245;
    analogWrite(gatePin,Power); 
    if(temp_average >= lb){
      idk = 0;
    }
  }
  
  else{
    //*** This IF statement is the actual ON-OFF control
    if (hyst == true){
      powerlim();
      analogWrite(gatePin,Power); 
        if (temp_average >= ub){
          hyst = false;
          }
      }
    else{
      powerlim();
      analogWrite(gatePin,Power);
        if (temp_average <= lb){
          hyst = true; 
          }
        
       }  
  integrate(); 
  }
delay(1000);  
}

// Below, we define several functions that are used in the main LOOP()
// We use function to make the main LOOP() a little tidier

//* This function computes the average temperature for better results
void measureTemperature(){
  for(int m=1;m<=20;m++){
    Vo = analogRead(termopin); // read analog to digital converter value
    Rt = R_series * (Vo / (1023 - Vo)); 
    lnRt = log(Rt);
    T_kelvin = (1.0 / (A + B*lnRt + C*pow(lnRt, 3)));
    T_celsius = T_kelvin - 273.15; 
    temp_sum += T_celsius;
  }
  temp_average = temp_sum/20.0;
  temp_sum = 0.0;


// For safety: The following turns off the heater if the temperature exceed the pre-defined MAX temperature
  if(temp_average >= Tmax){
    while(1){         // Notice that this while condition is always true, so this will permanently stop the code until you reset the Arduino
      analogWrite(gatePin,0);   //turning off the heater
      Serial.println("Maximum temperature has been exceeded. Heater is turned OFF. Reset Arduino to restart the program");
      delay(5000);
    }
  }  
}


void powerlim(){		// this will control our power value for heat source 

  Power = ((Kp*Error+(Integral*Ki)+Delta*Kd)*255);
  Power = constrain(Power,0,245);
  }

void integrate (){// will calculate integral 
  
  I = DeltaTime * Previous_Error;
  Integral += I;
  
}


void dv(){		//getting derivative value
  Delta = (Error-Previous_Error)/DeltaTime;
  
}


void getTempHu()		// this will read value from the sensor,(temperature and humidity of air) and get sum of them to calculate average
{ 
  temp = 0;
  humd = 0;
  tempsum =0;
  humdsum =0;
  
  DHT.read11(temphupin);
  for (int i=0; i <=100; i++)
  { 
    tempsum += DHT.temperature;
    humdsum += DHT.humidity; 
    delay(10);
  }

  temp = tempsum/100.0;
  humd = humdsum/100.0;

  
}

void humdsy(){			// this part is to control air humidity, will turn on for 30 second if our system have less then 70% of air humidity 
   pinMode(humidpin,OUTPUT);
  
  if(humd <= 70.0){
    digitalWrite(humidpin,HIGH);
    delay(30000);
    digitalWrite(humidpin,LOW);
  }

  else{
    delay(500);
    digitalWrite(humidpin,LOW); 
  }
}

void printing(){							// this part is for monitoring, we tried to put most of usefull information that we want to track.
  Serial.println("target set up        ");
  Serial.println("system temp:30 C,      water temp: 33C,      air humd: 75%,       light time:14 H,       light off time: 10H,       soil mosit : less then 400 ");

  Serial.print("system temp: ");
  Serial.println(temp);
  Serial.print("air humd: ");
  Serial.println(humd);
  Serial.print("water temp: ");
  Serial.println(temp_average);
  Serial.print("power used for heating: ");
  Serial.println(Power);
  Serial.print("light time: ");
  Serial.println(lighttime/(60.0*1000.0));
  Serial.print("light off time: ");
  Serial.println(lightoff/(60.0*1000.0));
  Serial.print("Run time(reset ever 24 hour): ");
  Serial.println(CurrentTime/(60.0*1000.0));
  Serial.print("soil moist: ");
  Serial.println(waterVal);

}
