const int switchpin=4;
int switchstate;
int sensorpin=2;
int mainsupply=8;
int buzzer=9;
const unsigned long eventinterval=10000;
unsigned long previoustime=0;
void setup() {
  Serial.begin(9600);
  pinMode(switchpin,INPUT_PULLUP);
  pinMode(sensorpin,INPUT);
  pinMode(mainsupply,OUTPUT);
  pinMode(buzzer,OUTPUT);
}

void loop() {
  switchstate=digitalRead(switchpin);
  if(switchstate==LOW)
  {
    int sensorvalue=digitalRead(sensorpin);
    if(sensorvalue==LOW)
    {
     digitalWrite(mainsupply,HIGH);
     unsigned long currenttime=millis();
     if(currenttime-previoustime>=eventinterval)
     {
     digitalWrite(buzzer,HIGH);
     digitalWrite(mainsupply,LOW);
     previoustime=currenttime;
     }
     
     
    }
    if(sensorvalue==HIGH)
    {
      digitalWrite(mainsupply,LOW);
    }
    
  }
digitalWrite(buzzer,LOW);
}
