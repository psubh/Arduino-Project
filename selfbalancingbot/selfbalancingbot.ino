#include <Wire.h>
#include <Event.h>
#include <Timer.h>

#define lp 7
#define ln 4
#define el 5
#define rp 8
#define rn 12
#define er 6



//----------------------------------------------MPU READING-------------------------------------------------
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float Acceleration_angleX, Acceleration_angleY;
//float Gyro_angle[2];
float Total_angleX, Total_angleY;
//-----------------------------------------------------------------------------------------------------------

//int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

//------------------------------------TIME-------------------------------------------------------------------
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
//-----------------------------------------------------------------------------------------------------------

//------------------------------------PID VARIABLES----------------------------------------------------------
float errorX,errorY,P_errorX,P_errorY,D_errorX,D_errorY,I_errorX,I_errorY,previous_errorX,previous_errorY;
float req_angleX = 0,req_angleY=0;
float PID_X,PID_Y;
float FPWM;
int  FPWM_X,FPWM_Y;

double setspeed=00;

//-------------------------------------PID CONSTANT-----------------------------------------------------------
double kp=1;
double ki=0.00;
double kd=0;
//------------------------------------------------------------------------------------------------------------


//long integral[4],derivative,err,perr[4],out,fpwm[4];
//long pulses[4],ppulse[4],i;
//float RPM[4];
//void count1()
//{
//  pulses[0]++;
//}
//
//void count2()
//{
//  pulses[1]++;
//}
//
//void count3()
//{
//  pulses[2]++;
//}
//
//void count4()
//{
//  pulses[3]++;
//}


void setupMPU()
{
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}
void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  time = millis(); //Start counting time in milliseconds 
  
  for(int i=22;i<=25;++i)
    pinMode(i, OUTPUT);
  for(int i=2;i<=5;++i)
    pinMode(i, OUTPUT);
  for(int i=18;i<=21;++i)
    pinMode(i, INPUT_PULLUP);
  for(int i=33; i<=34; ++i)
    pinMode(i, OUTPUT);

   setupMPU();
                                      //Delay 3us to simulate the 250Hz program loop
}

// calibrartion mpu6050
//set gyro - flase
//
//------------INTERRUPT INITIALISATION-----------------
//  attachInterrupt(2,count1,CHANGE);
//  attachInterrupt(3,count2,CHANGE);
//  attachInterrupt(4,count3,CHANGE);
//  attachInterrupt(5,count4,CHANGE);
//  t.every(100,rpm,0);




void loop() 
{
  //Serial.print("hello");
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
    
    recordAccelRegisters();      
    recordGyroRegisters();
  
    //complementery filter
    /*---X axis angle---*/
    Total_angleX = 0.98 *(Total_angleX + rotX*elapsedTime) + 0.02*Acceleration_angleX;
    /*---Y axis angle---*/
    Total_angleY = 0.98 *(Total_angleY + rotY*elapsedTime) + 0.02*Acceleration_angleY;
//      Serial.print(" Total_angles_filter: X  Y:  ");
//      Serial.print(Total_angleX);
//      Serial.print(" ");
//      Serial.println(Total_angleY);

    pid_mpu();
  
    //printData();
    //delay(1000);
}


void recordAccelRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
  /*---X---*/
  Acceleration_angleX = atan((gForceY)/sqrt(pow((gForceX),2) + pow((gForceZ),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angleY = atan(-1*(gForceX)/sqrt(pow((gForceY),2) + pow((gForceZ),2)))*rad_to_deg;
//  Serial.print(" Acceleration_angles: X Y ");
//  Serial.print(Acceleration_angleX);
//  Serial.print(" ");
//  Serial.print(Acceleration_angleY);
//  Serial.println(" ");
}

void recordGyroRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() 
{
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
//  Serial.print(" Gyro_angles: X Y \t");
//  Serial.print(rotX);
//  Serial.print(" ");
//  Serial.print(rotY);
//  Serial.println(" ");
}

//void rpm()
//{
//  long double p[4];
//  for(i = 0; i < 4; i++)
//  {
//    p[i] = pulses[i];
//    RPM[i] = ((p[i]-ppulse[i])/140.0)*600.0;
//    ppulse[i]=p[i];
//  }
//  Serial.print(RPM[0]);
//  Serial.print(" ");
//  Serial.print(RPM[1]);
//  Serial.print(" ");
//  Serial.print(RPM[2]);
//  Serial.print(" ");
//  Serial.println(RPM[3]);
//}
//}


void pid_mpu()
{
  //errorX=Total_angleX-req_angleX;
  errorY=Total_angleY-req_angleY;
  
  //P_errorX=kp*errorX;
  P_errorY=kp*errorY;
  
  
  //D_errorX=kd*((errorX-previous_errorX)/elapsedTime);
  D_errorY=kd*((errorY-previous_errorY)/elapsedTime);

  
  
  /*if(-3<errorX<3)
  {
    I_errorX=I_errorX+(ki*errorX);
  }
  
  if(-3<errorY<3)
  {
    I_errorY=I_errorY+(ki*errorY);
  }*/
  
  //PID_X=P_errorX+D_errorX+I_errorX;
  PID_Y=P_errorY+D_errorY+I_errorY;
  
 
  
//  if(PID_X < 0)
//    PID_X=0;
//    
//  if(PID_X >255)
//    PID_X=255;
//
//  if(PID_Y < 0)
//    PID_Y=0;
//    
//  if(PID_Y >255)
//    PID_Y=255;
// Serial.print("pid_X values ");
//  Serial.print(PID_X);
//  Serial.print(" ");
 // Serial.print("pid_Y values ");
  // Serial.println(PID_Y);
  //delay(800);
  
//FPWM_X=(setspeed+PID_X)*255/478;
FPWM_Y=(50+PID_Y);//*255/60;
 Serial.println(FPWM_Y);
//if(FPWM_X < 0)
  //  FPWM_X=0;
    
  //if(FPWM_X >255)
   // FPWM_X=255;

  if(FPWM_Y < 0)
    FPWM_Y=0;
    
  if(FPWM_Y >255)
    FPWM_Y=255;

 //Serial.print(FPWM_X);
 //Serial.print("  ");


if(PID_Y<0)
{
  FPWM=FPWM_Y;
  motor_rotate(2,2);
}

if(PID_Y>0)
{
  FPWM=FPWM_Y;
  motor_rotate(1,1);
  
}

  //previous_errorX=errorX;
  previous_errorY=errorY;
}

//void motor_control(int m1d,int m2d)
//{
//  motor_rotate(,FPWM);
//  motor_rotate(m2d,d2,m2pwm,FPWM);
//}

void motor_rotate(int left, int right)
{
 switch(left)
 {
  case 0://stop
      digitalWrite(lp,LOW);
      digitalWrite(ln,LOW);
      break;
  case 1:
      digitalWrite(lp,LOW);
      digitalWrite(ln,HIGH);
      analogWrite(el,FPWM);
      break;
  case 2:
      digitalWrite(lp,HIGH);
      digitalWrite(ln,LOW);
      analogWrite(el,FPWM);
      break;
 }
 switch(right)
 {
  case 0://stop
      digitalWrite(rp,LOW);
      digitalWrite(rn,LOW);
      break;
  case 1:
      digitalWrite(rp,LOW);
      digitalWrite(rn,HIGH);
      analogWrite(er,FPWM);
      break;
  case 2:
      digitalWrite(rp,HIGH);
      digitalWrite(rn,LOW);
      analogWrite(er,FPWM);
      break;
 }
}

void printData() 
{
  Serial.print("Gyro (deg)");
  Serial.print("\tX=");
  Serial.print(rotX);
  Serial.print("\tY=");
  Serial.print(rotY);
  Serial.print("\tZ=");
  Serial.print(rotZ);
  Serial.print("\t\tAccel(g)");
  Serial.print("\tX=");
  Serial.print(gForceX);
  Serial.print("\tY=");
  Serial.print(gForceY);
  Serial.print("\tZ=");
  Serial.println(gForceZ);
  delay(100);
}
