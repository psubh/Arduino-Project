#define buzzerpin 10
#define trigpin 11
#define echopin 12
int duration;
int distance;
void setup()
{
  Serial.begin(9600);
  pinMode(trigpin,OUTPUT);
  pinMode(echopin,INPUT);
  pinMode(buzzerpin,OUTPUT);

}

void loop() 
{   
  digitalWrite(trigpin,HIGH);
  delay(10);
  digitalWrite(trigpin,LOW);
   duration = pulseIn(echopin, HIGH);
  distance=duration*0.034/2;
  Serial.println(distance);
  if(distance<=50)
  {
    tone(buzzerpin, 1000); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  noTone(buzzerpin);     // Stop sound...
  delay(100); 
    //digitalWrite(buzzerpin,HIGH);
    //delay(10);
    //digitalWrite(buzzerpin,LOW);
  }
  else
  { digitalWrite(buzzerpin,LOW);
   // delay(2000);
    //digitalWrite(buzzerpin,HIGH);
    }

}
