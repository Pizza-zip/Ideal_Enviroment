
#define moist_pin A0	

int led_1_pin = 13;
int led_2_pin = 12;

float moist_val;


void setup() {

  Serial.begin(9600);
  pinMode(led_1_pin, OUTPUT);
  pinMode(led_2_pin, OUTPUT);
 
}

void loop() {

  getMoist();
  
  Serial.println(moist_val);

  moistLight();

}


void getMoist(){		//this will measure moisture value
 
 for(int i = 0; i <= 100; i++){
   moist_val = moist_val + analogRead(moist_pin);		//will measure 100 time and get an average value
   delay(100);
 }
 moist_val = moist_val/100.0;
}


void moistLight(){

  if (moist_val <= 250){

    digitalWrite(led_2_pin, HIGH); 
    digitalWrite(led_1_pin, LOW); 
  }
  else{
    digitalWrite(led_1_pin, HIGH);
    digitalWrite(led_2_pin, LOW);
  }
}