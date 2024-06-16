#include <Adafruit_MAX31856.h>

#define moist_pin A0

int led_1_pin = 10;
int led_2_pin = 12;
int pump_pin_1 = 13;
int water_cond;

float moist_val;
float MOIST_MAX = 430.0;
float MOIST_MIN = 220;

unsigned long time;
unsigned string;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(6, 5, 4, 7);

void setup() {

  Serial.begin(115200);

  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }

  pinMode(led_1_pin, OUTPUT);
  pinMode(led_2_pin, OUTPUT);
  pinMode(pump_pin_1, OUTPUT);
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_J);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
}

void loop() {

  moist_val = getMoist();

  if (water_cond == 120){
    //
    pumpStatus(moist_val);
    water_cond = 0;
  }
  //checkWater(moist_val);
  time = millis();

  Serial.print(time * 1e-3);
  Serial.print(" ");
  Serial.print(moist_val);
  Serial.print(" ");
  Serial.print(maxthermo.readThermocoupleTemperature());
  Serial.print("\t");
  Serial.println();

  delay(30000);

  water_cond += 1;
}


double getMoist() {  //this will measure moisture value

  for (int i = 0; i <= 100; i++) {
    moist_val = moist_val + analogRead(moist_pin);  //will measure 100 time and get an average value
    delay(5);
  }
  moist_val = (1.0/(((moist_val / 100.0))/(MOIST_MAX)) - 1) *100;
  
  //moist_val = moist_val / 100.0;
  return moist_val;
}


void moistLight() {

  if (moist_val <= 60) {

    digitalWrite(led_2_pin, HIGH);
    digitalWrite(led_1_pin, LOW);
  } else {
    digitalWrite(led_1_pin, HIGH);
    digitalWrite(led_2_pin, LOW);
  }
}

void waterPump(char State) {

  if (strcmp(State,"on"))
  {
    digitalWrite(pump_pin_1, HIGH);
  }
  else  
  {
   digitalWrite(pump_pin_1, LOW); 
  }
}

void pumpStatus(double moist_val){
    if (moist_val <= 50 )
    {
      digitalWrite(pump_pin_1, HIGH);
      delay(20000);
    }
    else
    {
      digitalWrite(pump_pin_1, LOW); 
    } 
  }