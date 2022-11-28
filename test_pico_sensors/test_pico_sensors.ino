
#define PIN 26

int val;

void setup() {
  Serial.begin(9600);
}


void loop() {
  pot_test();

  //piezo_test();

  //piezo_threshold();
}


void pot_test(){
  val = analogRead(PIN);
  //val = map(val, 4, 1023, 0, 1023);   //testing found low at 4 and high at 1023, so map to proper range
  Serial.println(val);
  delay(200);
}


void piezo_test(){
  delay(1);   //delay needed for ADC
  val = analogRead(PIN);
  if(val <= 20){
    return;
  }
  
  Serial.println(val);
}


void piezo_threshold(){
  val = analogRead(PIN);
  while(val < 20){
    delay(1);
    val = analogRead(PIN);
  }
  Serial.println(val);
  delay(1);
}
