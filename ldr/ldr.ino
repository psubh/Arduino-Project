#define sensorvalue A0
void setup ()
{
Serial.begin(9600);
}
void loop()
{
int sensorvalue=analogRead(A0);
Serial.println(sensorvalue);
/*delay(1);
if (sensorvalue>=600)
{
digitalWrite(13,HIGH);
}
else
{
digitalWrite(13,LOW);
}*/
}
