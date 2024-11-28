char userInput;

void setup(){
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
  if(Serial.available()>0){
    userInput = Serial.read();
    if(userInput == 'x'){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("ON");
    }
    if(userInput == 'z'){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("OFF");
    }
  }
}