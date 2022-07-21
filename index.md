# 3 Joint Robotic Arm With Claw
Hi, my name is Sasha! The project that I chose to build was the 3 joint robotic arm with claw and potentiometers. This robotic arm will function through the use of micro servo and potentiometers. This robotic claw will be able to hold different small objects. 

<a href="https://ibb.co/9vMxJqc"><img src="https://i.ibb.co/4fQbcjK/IMG-3397.jpg" alt="IMG-3397" border="0" height="450" width="570"></a>

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Sasha | Marin Academy | Biomedical Engineering | Incoming Senior

# My project
Below is a picture of my completed robotic arm. This picture shows the actual robotic arm as well as the circuit board that have the potentiometers which control the robotic arm. 

<a href="https://ibb.co/k6v3M14"><img src="https://i.ibb.co/rfNbvsx/IMG-3598.jpg" alt="IMG-3598" border="0"></a>
  
# Final Milestone
My final milestone showcases my whole project. In this video I demonstrate both the regular robotic arm functioning as well as the robotic arm with modifications that I added. The modification that I added to the robotic arm was using GUI or graphic user interface to control the arm. This means that I can control my robotic arm using a program on my computer instead of the potentiometers. To do this I used python and arduino to control the robotic arm via GUI. During this one challenge thatI faced was the python interface not working properly. To solve this problem I went through and checked all my connections and restarted the program a couple times and I was able to get it working again. Throughout this project I experienced different challenges but I was able to work through them and I gained great probelm solving skills from the challenges. 

[![Third Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1658339257/video_to_markdown/images/youtube--r5V2tHF2FLQ-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/r5V2tHF2FLQ "Third Milestone"){:target="_blank" rel="noopener"}

# Second Milestone
My second milestone was building the actual robotic arm. For this I used all the pieces in the kit and put together the robotic arm. This arm is controlled by potentiometers and servos so I had to make sure all the connections were properly working. Another key component in building this arm was the arduino code being sent to the arm. In this code it established the servos and the rotation of each servo. The code also outlined the connections between the potentiometer and servo so the potentiometer could correctly control the servos. For this milestone one of the challenges that I faced was the servo was not properly functioning and rotating. This was a big challenge but after carefully checking my connections and making sure all the actual pieces were properly connected I was able to get the arm to properly work. 

[![Second Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1657906263/video_to_markdown/images/youtube--Gr-YLulEjl0-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/Gr-YLulEjl0 "Second Milestone"){:target="_blank" rel="noopener"}

# First Milestone
  
My first milestone was connecting a micro servo and potentiometer controlled via arduino. The micro servo and potentiometer were connected through a breadboard and the arduino. I used code written out in Arduino on my laptop to control the potentiometer and servo. The code essentially translated the potentiometer movements to the servo movemens so that these two seperate pieces were synched together. To do this there were several connections that I had to make either directly from potentiometer to servo or servo to arduino to potentiometer. Although I was successful in making these connnections at first the servo was not working properly. After carefully reviewing all my connections and making sure there were no small errors the servo was properly functioning.


[![First Milestone](https://res.cloudinary.com/marcomontalbano/image/upload/v1657298926/video_to_markdown/images/youtube--COM9trcONng-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=COM9trcONng "First Milestone"){:target="_blank" rel="noopener"}

# Bill of Materials 

This is a table containing all the items needed for milestone one and the robotic arm. It contains the item, quanitity, price and where to buy the item.  

| Item | Qty | Price | Where to Buy |
| ------------- | ------------- | ------------- | ------------- |
| Adeept Robotic Arm Kit  | 1  | $64.99  | https://www.amazon.com/dp/B087R8DLG6 |
| Breadboard  | 1 |  $6.75  | https://www.amazon.com/BB400-Solderless-Plug-BreadBoard-tie-points/dp/B0040Z1ERO |
| Potentiometer  | 1 | $1.85  |  https://www.tubesandmore.com/products/potentiometer-alpha-linear-38-bushing  |
| Micro Servo | 1 | $5.95 | https://www.adafruit.com/product/169  |
| Jumper Wires  | 6 | 10¢/wire  |  https://www.amazon.com/Breadboard-Jumper-Wire-75pcs-pack/dp/B0040DEI9M |
| Arduino Uno Board  | 1  | $27.60  | https://store-usa.arduino.cc/products/arduino-uno-rev3?selectedStore=us  |
| Arduino IDE  | 1  | $0  | https://www.arduino.cc/en/software/ |


# Schematics

This is a schematic of the adeept circuit board used for my robotic arm. This board connects to my computer which contains the code for the arm. This board also has the potentiometers that control the robotic arm. 
<a href="https://ibb.co/wcMBbYp"><img src="https://i.ibb.co/2ZgWL5c/c14cded516.jpg" alt="c14cded516" border="0"></a>

# Robotic Arm Code 

Below is my code I used to control the original robotic arm. This code is through arduino and it controls both the micro servos in the arm and the potentiometers controlling the micro servos. 

``` 
#include <Servo.h>
Servo servo1;//create servo object to control a servo
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int dataServo1 = 90; 
int dataServo2 = 90; 
int dataServo3 = 90; 
int dataServo4 = 90; 
int dataServo5 = 90; 

float dirServo1Offset = 0;   
float dirServo2Offset = 0;  
float dirServo3Offset = 0;   
float dirServo4Offset = 0;    
float dirServo5Offset = 0;    
int val1;
int val2;
int val3;
int val4;
int val5;
void setup()
{
  servo1.attach(9);
  servo2.attach(6);
  servo3.attach(5);
  servo4.attach(3);
  servo5.attach(11);
  
  servo1.write(dataServo1+dirServo1Offset); 
  servo2.write(dataServo2+dirServo2Offset); 
  servo3.write(dataServo3+dirServo3Offset);
  servo4.write(dataServo4+dirServo4Offset); 
  servo5.write(dataServo5+dirServo5Offset);
}
void loop()
{
  servo1.write(dataServo1+dirServo1Offset);//goes to dataServo1 degrees 
  servo2.write(dataServo2+dirServo2Offset);
  servo3.write(dataServo3+dirServo3Offset);
  servo4.write(dataServo4+dirServo4Offset);
  servo5.write(dataServo5+dirServo5Offset);
  
  val1 = map(analogRead(0), 0, 1023, 0, 180);  
  val2 = map(analogRead(1), 0, 1023, 0, 180);  
  val3 = map(analogRead(2), 0, 1023, 0, 180);  
  val4 = map(analogRead(3), 0, 1023, 0, 180);
  val5 = map(analogRead(6), 0, 1023, 35, 90);  
 
  dataServo1 = val1;
  dataServo2 = val2;
  dataServo3 = val3;
  dataServo4 = val4;
  dataServo5 = val5;
  delay(50);//wait for 0.05second
} 
```

Below is the code that is used to support the graphic user interface which is the modification I made to the robotic arm. For this modification I used a python based program to control the robotic arm instead of the potentiometers. 

```
/*
 * KEY4x4 and MPU6050 cannot be 1 at the same time. Otherwise, insufficient capacity will be indicated.
 * When BUZZER is 1, the infrared sensor cannot be used, but the BUZZER can be used
 * When BUZZER is 0, the infrared sensor can be used, but the BUZZER cannot be used
 *(you can change these values as you apply them)
*/
#define KEY4x4  0
#define MPU6050 0
#define BUZZER   0


#include <ArduinoJson.h>
#if !BUZZER
#include <IRremote.h>
#endif

#include <Servo.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht11.h>
#include <Keypad.h>
LiquidCrystal_I2C lcd(0x27,16,2); 




dht11 DHT11;

#if !BUZZER
char RECV_PIN = 11;//The definition of the infrared receiver pin 11
IRrecv irrecv(RECV_PIN);
decode_results results;
#endif
Servo myservo[5];
char a = 2;
char b = 3;
char c = 4;
char d = 5;
char e = 6;
char f = 7;
char g = 8;
char p = 9;
char dp = 9;
char d4 = 10;
char d3 = 11;
char d2 = 12;
char d1 = 13;
char APin ;                       
char BPin ;                     
int Pin0 = 0;
int Pin1 = 0;
int Pin2 = 0;
int Pin3 = 0;



char line[60] = ""; // Serial data
int ret = 0;
char pingPin,trigPin;



#if MPU6050

unsigned long now, lastTime = 0;
float dt;                                   //derivative time
unsigned short times = 200;             //Sampling frequency
int16_t ax, ay, az, gx, gy, gz;             //Accelerometer gyroscope raw data
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    //Angle variable
long axo = 0, ayo = 0, azo = 0;             //Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             //Gyroscope offset
 
float pi = 3.1415926;
float AcceRatio = 16384.0;                  //Accelerometer scaling factor
float GyroRatio = 131.0;                    //Gyroscope scale factor
 
uint8_t n_sample = 8;                       //The number of samples sampled by accelerometer filtering algorithm
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //Sample the queue on the x and y axes
long aax_sum, aay_sum,aaz_sum;                      //X,y axis sampling queue x,y axis sampling and
 
float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //Accelerometer covariance calculation queue
float Px=1, Rx, Kx, Sx, Vx, Qx;             //The kalman variable on the X-axis
float Py=1, Ry, Ky, Sy, Vy, Qy;             //The kalman variable on the Y-axis
float Pz=1, Rz, Kz, Sz, Vz, Qz;             //The kalman variable on the Z-axis

const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#endif

#if KEY4x4
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the keypad
 
//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
#endif

 int ST = 0; //Pin connected to ST_CP of 74HC595
 int SH = 0;//Pin connected to SH_CP of 74HC595  
 int DS = 0; //Pin connected to DS of 74HC595 

void setup() {
  memset(line, 0, sizeof(line));
Serial.begin(115200); // Open the serial port and set the data transmission rate of 115200

}

void loop() {
  DynamicJsonDocument  jsonBuffer(100);
// Operate when the serial port is available
if (Serial.available() > 0) {
  
//  read incoming data: read up to \n, or up to 500 characters
ret = Serial.readBytesUntil('\n', line, 100);
//Serial.println(line);
  deserializeJson(jsonBuffer, line);
JsonObject root = jsonBuffer.as<JsonObject>();

  my_cmp(root);

memset(line, 0, sizeof(line));
}

}
void my_cmp(JsonObject root)
{
  /* parses the JSon data and executes the corresponding function */
  const char *python_char =  root["start"][0];
  const char *python_one =  root["start"][1];
  long one = root["start"][1],two = root["start"][2];
  long three = root["start"][3],four = root["start"][4];
  //Serial.println(one);Serial.println(two);
  //Serial.println(one);
  if(strcmp(python_char,"setup")==0)
  {
      Serial.println("succes");
      memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"pinmode")==0)
  {
      int q,w;
      q = root["start"][1];
      w = root["start"][2];
      //input 0,output 1
      pinMode(one, two);
      memset(line, 0, sizeof(line));
      
  }
   else if(strcmp(python_char,"pulseIn")==0)
  {
      int q,w;
      q = root["start"][1];
      w = root["start"][2];
      //input 0,output 1
      Serial.println(pulseIn(one, two));
      memset(line, 0, sizeof(line));
      
  }
  else if(strcmp(python_char,"digitalWrite")==0)
  {
      int q,w;
      q = root["start"][1];
      w = root["start"][2];
      //input 0,output 1
      digitalWrite(one, two);
      memset(line, 0, sizeof(line));
      
  }
  else if(strcmp(python_char,"digitalRead")==0)
  {
      int q,w;
      q = root["start"][1];
      
      //input 0,output 1
      w = digitalRead(one);
      Serial.println(w);
      memset(line, 0, sizeof(line));
      
  }
  else if(strcmp(python_char,"analogRead")==0)
  {
      int q,w;
      q = root["start"][1];
      
      //input 0,output 1
      w = analogRead(one);
      //Serial.print("text:");
      Serial.println(w);
      memset(line, 0, sizeof(line));
      
  }
  else if(strcmp(python_char,"analogWrite")==0)
  {
    
      int q,w;
      //q = root["start"][1];
      //w = root["start"][2];
      //Serial.println(q);
      //input 0,output 1
      analogWrite(one, two);
      memset(line, 0, sizeof(line));
      
  }
  #if BUZZER
  else if(strcmp(python_char,"tone")==0)
  {
    
      int q,w;
      //q = root["start"][1];
      //w = root["start"][2];
      //Serial.println(q);
      //input 0,output 1
      tone(one, two);
      memset(line, 0, sizeof(line));
      
  }
  else if(strcmp(python_char,"noTone")==0)
  {
    
      int q,w;
      //q = root["start"][1];
      //w = root["start"][2];
      //Serial.println(q);
      //input 0,output 1
      noTone(one);
      memset(line, 0, sizeof(line));
      
  }
  #endif
  
  else if(strcmp(python_char,"lcd_init")==0)
  {
    //Serial.println("led");
      lcd.init(); 
      lcd.backlight();  
      lcd.clear();
        //lcd.print("asdasdasd");
      memset(line, 0, sizeof(line));
  }
  /*LCD initialization requires 2s delay */
  else if(strcmp(python_char,"lcd_setCursor")==0)
  {
      int q,w;
      q = root["start"][1];
      w = root["start"][2];
      //Serial.println(q);
      lcd.setCursor(one,two); 
      memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"lcd_print")==0)
  {
      //const char *q = root["start"][1]; 
      //LCD_fllag = 1;
       //Serial.println(python_one);
      lcd.print(python_one);  
      memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"lcd_clear")==0)
  {
    
      lcd.clear();  
      memset(line, 0, sizeof(line));
  }
  
   if(strcmp(python_char,"segment_init")==0)
  {
   
    for(int i = 1;i< 9;i++)
    {
      pinMode(i,OUTPUT);
    }
      memset(line, 0, sizeof(line));
  }
  
  else if(strcmp(python_char,"digital")==0)
  {
     int q,w;
     q = root["start"][1];
     digital_all(one);
      memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"fourdigitsegment_init")==0)
  {
    for(int i = 2;i< 14;i++)
      pinMode(i,OUTPUT);
      memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"pickDigit")==0)
  {
    int q,w;
    q = root["start"][1];
    w = root["start"][2];
    pickDigit(one);
    pickNumber(two);
    memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"servo_attach")==0)
  {
   
    myservo[one].attach(two);
    memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"servo_write")==0)
  {
    int q,w;
    

    
    myservo[one].write(two);
    
    memset(line, 0, sizeof(line));
  }
  #if !BUZZER
  else if(strcmp(python_char,"irrecv_init")==0)
  {
    int q,w;
    q = root["start"][1];
    w = root["start"][2];
    irrecv.enableIRIn();
    memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"irrecv_recv")==0)
  {
      if (irrecv.decode(&results)) 
      {
        int irr_data = results.value;
        int irr_recv = switch_irr(irr_data);
        Serial.println(irr_recv);
        //Serial.println(results.value, HEX);//Wrap output in hex receive code
        irrecv.resume(); //Receiving the next value
      }
      else
      {
        Serial.println(-1);  
      }
      memset(line, 0, sizeof(line));
  }
  #endif
  else if(strcmp(python_char,"DHT11_read")==0)
  {
    int q,w,DHT_data;
    
    DHT11.read(one);
    if(two == 1)
      DHT_data = (int)DHT11.temperature;
    else
      DHT_data = (int)DHT11.humidity;
      Serial.println(DHT_data);
    memset(line, 0, sizeof(line));
  }
  else if(strcmp(python_char,"ultra_init")==0)
  {
     pingPin = one;
     trigPin = two;
     pinMode(pingPin, INPUT); //Set the connection pin output mode Echo pin
     pinMode(trigPin, OUTPUT);//Set the connection pin output mode trog pin
    memset(line, 0, sizeof(line));
  }
   if(strcmp(python_char,"ultra_recv")==0)
  {
    
    int a = ping(pingPin);
    Serial.println(a);
  }
  
#if MPU6050
  else if(strcmp(python_char,"MPU6050_init")==0)
  {
      Wire.begin();
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B); // PWR_MGMT_1 register
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      for(int i=0;i<times;i++)
      {
          getMotion6(); // read the original value of six axes
          axo += AcX; ayo += AcY; azo += AcZ;      //sampling interval
          gxo += GyX; gyo += GyY; gzo += GyZ;
      }
      axo /= times; ayo /= times; azo /= times; //Calculate the accelerometer offset
      gxo /= times; gyo /= times; gzo /= times; //Calculate the accelerometer offset

 
  }
  else if(strcmp(python_char,"MPU6050_data")==0)
  {
    int q;
    count6Axle();
    if(one == 1)
      q = agx;
    else if(one == 2)
      q = agy;
    else if(one == 3)
      q = agz;
    Serial.println(q);
  }
#endif  
#if KEY4x4
  else if(strcmp(python_char,"4x4Keyboard")==0)
  {
     while(1)
     {
      char customKey = customKeypad.getKey();//Read Key data  
      if (customKey){
        
        Serial.println(customKey);          //send the key data by serial port (UART)
        break;
      }
     }
  }
#endif
  else if(strcmp(python_char,"DC_init")==0)
  {
      APin = one;
      BPin = two;
      pinMode(APin,OUTPUT);                    //initialize the A pin as output 
      pinMode(BPin,OUTPUT);                    //initialize the B pin as output 
    //int a = ping(pingPin);
    //Serial.println(a);
  }
  else if(strcmp(python_char,"foreward")==0)
  {
      foreward(one);
  }
  else if(strcmp(python_char,"reversal")==0)
  {
      reversal(one);
  }
  
  else if(strcmp(python_char,"stepper")==0)
  {
     Pin0 = one;Pin1 = two;
     Pin2 = three;Pin3 = four;
     //Serial.println(Pin3);
     pinMode(Pin0, OUTPUT);
     pinMode(Pin1, OUTPUT);
     pinMode(Pin2, OUTPUT);
     pinMode(Pin3, OUTPUT);
  }

  else if(strcmp(python_char,"stepper_begin")==0)
  {
   // Serial.println(one);
     Speed(one);//Stepper motor speed = 15 fast (note:speed from 1 to 15)
     Step(two);//Stepper motor forward 512 steps ---- 360 angle 
  }
  else if(strcmp(python_char,"ledMatrix_init")==0)
  {
     ST = one;SH= two;DS= three;
     //Serial.println(ST);
     pinMode(ST, OUTPUT);
     pinMode(SH, OUTPUT);
     pinMode(DS, OUTPUT);
     
  }
  else if(strcmp(python_char,"ledMatrix_data")==0)
  {
    unsigned char  tab[]={0x7f,0xbf,0xdf,0xef,0xf7,0xfb,0xfd,0xfe};  
    char q[8] = "";
    //Serial.println(three);
    char a=one>>8&0xff;char b = one&0xff;
    char c=two>>8&0xff;char d = two&0xff;
    char e=three>>8&0xff;char f = three&0xff;
    char g=four>>8&0xff;char h = four&0xff;
    q[0] = a;q[1]=b;q[2] = c;q[3]=d;
    q[4] = e;q[5]=f;q[6] = g;q[7]=h;
    //Serial.println(q);
    for(int i = 0;i<2000;i++)
    {
     for(int num=0; num < 8; num++)//8 columns of data sent to a dot matrix
      {   
         
        //Serial.println(q[num]);                   
          shiftOut(DS,SH,MSBFIRST,q[num]); //Send column data to a dot matrix
          shiftOut(DS,SH,MSBFIRST,tab[num]);//Send line data to a dot matrix
          //The rising edge of the data shift
          digitalWrite(ST,HIGH); //Output control latch HIGH  
          digitalWrite(ST,LOW);  //Output control latch LOW
      }
     // delay(1);
    }
    for(int num=0; num < 8; num++)//8 columns of data sent to a dot matrix
      {    
        //Serial.println(q[num]);                   
          shiftOut(DS,SH,MSBFIRST,0); //Send column data to a dot matrix
          shiftOut(DS,SH,MSBFIRST,tab[num]);//Send line data to a dot matrix
          //The rising edge of the data shift
          digitalWrite(ST,HIGH); //Output control latch HIGH  
          digitalWrite(ST,LOW);  //Output control latch LOW
      }
     
  }
  
  
  
}
int _step = 512; 
int _speed = 1; 
void Speed(int stepperspeed)//set Stepper speed 
{
    _speed = 15 - stepperspeed;
    if( _speed<1){
     _speed = 1;
    }
    if( _speed>15){
     _speed = 15;
    }
}
void Step(int _step)//Stepper motor rotation
{
  if(_step>=0){  // Stepper motor forward
    for(int i=0;i<_step;i++){   
      setStep(1, 0, 0, 1);
      delay(_speed); 
      setStep(1, 0, 0, 0);
      delay(_speed);
      setStep(1, 1, 0, 0);
      delay(_speed);
      setStep(0, 1, 0, 0);
      delay(_speed);
      setStep(0, 1, 1, 0);
      delay(_speed);
      setStep(0, 0, 1, 0);
      delay(_speed);
      setStep(0, 0, 1, 1);
      delay(_speed); 
      setStep(0, 0, 0, 1);
      delay(_speed);
    }
  }else{ // Stepper motor backward
     for(int i=_step;i<0;i++){  
      setStep(0, 0, 0, 1);
      delay(_speed);
      setStep(0, 0, 1, 1);
      delay(_speed);
      setStep(0, 0, 1, 0);
      delay(_speed);
      setStep(0, 1, 1, 0);
      delay(_speed);
      setStep(0, 1, 0, 0);
      delay(_speed);
      setStep(1, 1, 0, 0);
      delay(_speed);
      setStep(1, 0, 0, 0);
      delay(_speed);
      setStep(1, 0, 0, 1);
      delay(_speed);
    }
   }
}
void setStep(int a, int b, int c, int d)  
{  
    digitalWrite(Pin0, a);     
    digitalWrite(Pin1, b);     
    digitalWrite(Pin2, c);     
    digitalWrite(Pin3, d);     
}  

void foreward(int Speed)                   //the function to drive motor rotate clockwise
{
  analogWrite(APin,Speed);                  //set the speed of motor
  analogWrite(BPin,0);                      //stop the B pin of motor
}
void reversal(int Speed)            //the function to drive motor rotate counterclockwise
{
  analogWrite(APin,0);                      //stop the A pin of motor
  analogWrite(BPin,Speed);                  //set the speed of motor
}

#if MPU6050
void getMotion6()
{
   Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
}


void count6Axle()
{
  unsigned long now = millis();             // current time (ms)
    dt = (now - lastTime) / 1000.0;           // differential time (s)
    lastTime = now;                           // last sampling time (ms)
    getMotion6();
     float accx = AcX / AcceRatio;              //x axis acceleration
    float accy = AcY / AcceRatio;              //y axis acceleration
    float accz = AcZ / AcceRatio;              ///Z axis acceleration
 
    aax = atan(accy / accz) * (-180) / pi;    // the Angle between the Y-axis and the z-axis
    aay = atan(accx / accz) * 180 / pi;       // the Angle between the X-axis and the z-axis
    aaz = atan(accz / accy) * 180 / pi;       // the Angle between the z axis and the y axis
 
    aax_sum = 0;                              // the sliding weighted filtering algorithm for the original accelerometer data
    aay_sum = 0;
    aaz_sum = 0;
     for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
 
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; // Angle am to 0-90°
    aays[n_sample-1] = aay;                        // here the appropriate coefficient is obtained by the experimental method
    aay_sum += aay * n_sample;                     // the coefficient in this example is 9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;
 
    float gyrox = - (GyX-gxo) / GyroRatio * dt; // angular velocity in the X-axis
    float gyroy = - (GyY-gyo) / GyroRatio * dt; // angular velocity in the Y-axis
    float gyroz = - (GyZ-gzo) / GyroRatio * dt; // angular velocity in the Z-axis
    agx += gyrox;                             //Angular velocity integral
    agy += gyroy;                             
    agz += gyroz;

      /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 // average calculation of measured values
        a_x[i-1] = a_x[i];                      // is the average acceleration
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 // average acceleration of the X-axis
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 // average acceleration of the Y-axis
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;
 
    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              // to get the variance
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         
    Kx = Px / (Px + Rx);                     // calculate the kalman gain
    agx = agx + Kx * (aax - agx);             // the gyroscope Angle is superimposed with the accelerometer velocity
    Px = (1 - Kx) * Px;                       // update the p value
 
    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;
      /* kalman end */
 
//    Serial.print(agx);Serial.print(",");
//    Serial.print(agy);Serial.print(",");
//    Serial.print(agz);Serial.println();

}
#endif
int ping(int pingPin) 
{ 
   // establish variables for duration of the ping, 
   // and the distance result in inches and centimeters: 
   long duration, cm; 
   // The PING))) is triggered by a HIGH pulse of 2 or more microseconds. 
   // Give a short LOW pulse beforehand to ensure a clean HIGH pulse: 
   pinMode(trigPin, OUTPUT); 
   digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
   digitalWrite(trigPin, HIGH); 
   delayMicroseconds(5); 
   digitalWrite(trigPin, LOW); 

   pinMode(pingPin, INPUT); 
   duration = pulseIn(pingPin, HIGH); 

   // convert the time into a distance 
   cm = microsecondsToCentimeters(duration); 
   return cm ; 
} 

long microsecondsToCentimeters(long microseconds) 
{ 
   // The speed of sound is 340 m/s or 29 microseconds per centimeter. 
   // The ping travels out and back, so to find the distance of the 
   // object we take half of the distance travelled. 
   return microseconds / 29 / 2; 
} 

int switch_irr(int irr_data)
{
  switch(irr_data)
  {
  case 16750695: return 0;
  case 16753245: return 1;
  case 16736925: return 2;
  case 16769565: return 3;
  case 16720605: return 4;
  case 16712445: return 5;
  case 16761405: return 6;
  case 16769055: return 7;
  case 16754775: return 8;
  case 16748655: return 9;
  case 16738455: return 10;
  case 16756815: return 11;
  case 16718055: return 12;
  case 16730805: return 13;
  case 16716015: return 14;
  case 16734885: return 15;
  case 16726215: return 16;
  }
}
void digital_all(int q)
{
  switch(q)
  {
    case 0: digital_0();break;
    case 1: digital_1();break;
    case 2: digital_2();break;
    case 3: digital_3();break;
    case 4: digital_4();break;
    case 5: digital_5();break;
    case 6: digital_6();break;
    case 7: digital_7();break;
    case 8: digital_8();break;
    case 9: digital_9();break;
  }
}
void digital_0(void) //Segment display digital 0
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,HIGH);
  digitalWrite(f,HIGH);
  digitalWrite(g, LOW);
  digitalWrite(dp,LOW);
}
void digital_1(void) //Segment display digital 1
{
  digitalWrite(a,LOW);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,LOW);
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
  digitalWrite(dp,LOW);
}
void digital_2(void) //Segment display digital 2
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);
  digitalWrite(c,LOW);
  digitalWrite(d,HIGH);
  digitalWrite(e,HIGH);
  digitalWrite(f,LOW);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_3(void) //Segment display digital 3
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_4(void) //Segment display digital 4
{
  digitalWrite(a,LOW);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,LOW);
  digitalWrite(e,LOW);
  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_5(void) //Segment display digital 5
{
  digitalWrite(a,HIGH);
  digitalWrite(b,LOW);
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,LOW);
  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_6(void) //Segment display digital 6
{
  digitalWrite(a,HIGH);
  digitalWrite(b,LOW);  
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,HIGH);
  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_7(void) //Segment display digital 7
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);  
  digitalWrite(c,HIGH);  
  digitalWrite(d,LOW); 
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
  digitalWrite(dp,LOW);
}
void digital_8(void) //Segment display digital 8
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,HIGH);
  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}
void digital_9(void) //Segment display digital 9
{
  digitalWrite(a,HIGH);
  digitalWrite(b,HIGH);
  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH);
  digitalWrite(e,LOW);
  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
  digitalWrite(dp,LOW);
}

void pickDigit(int x)  //Defined pickDigit (x), whose role is to open the port dx
{
  digitalWrite(d1, HIGH);
  digitalWrite(d2, HIGH);
  digitalWrite(d3, HIGH);
  digitalWrite(d4, HIGH);
  switch(x)
  {
    case 1: 
           digitalWrite(d1, LOW); 
           break;
    case 2: 
           digitalWrite(d2, LOW); 
           break;
    case 3: 
           digitalWrite(d3, LOW); 
           break;
    default: 
           digitalWrite(d4, LOW); 
           break;
  }
}

void pickNumber(int x)   //Defined pickNumber (x), whose role is to display digital x
{
  switch(x)
  {
    case 1: 
           one(); 
           break;
    case 2: 
           two(); 
           break;
    case 3: 
           three(); 
           break;
    case 4: 
           four(); 
           break;
    case 5: 
           five(); 
           break;
    case 6: 
           six(); 
           break;
    case 7: 
     seven(); 
     break;
    case 8: 
     eight(); 
     break;
    case 9: 
     nine(); 
     break;
    default: 
           zero(); 
           break;
  }
}
   
void dispDec(int x)  //Decimal point setting Open
{
  digitalWrite(p, HIGH);
}
 
void clearLEDs()  //Clear screen
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
  digitalWrite(p, LOW);
}
 
void zero()  //Define those figures 0 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, LOW);
}
 
void one()  //Define those figures 1 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}
 
void two()  //Define those figures 2 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, LOW);
  digitalWrite(g, HIGH);
}
 
void three()  //Define those figures 3 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, HIGH);
}
 
void four()  //Define those figures 4 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}
 
void five()  //Define those figures 5 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, LOW);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}
 
void six()  //Define those figures 6 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}
 
void seven()  //Define those figures 7 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}
 
void eight()  //Define those figures 8 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}
 
void nine()  //Define those figures 9 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, LOW);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}
```

# What To Know For This Project 


For this project one really important thing to keep in mind before assembling the robotic arm is testing all the micro servos. Sometimes the micro servos don't work or there motors die and it can be really challenging to replace them when the robotic arm is already assembled. In order to check the servo you can use the a simple set up with potentiometers, arduino and micro servo like I built in my first milestone to make sure all the micro servos work. This simple extra task can lead to a smoother building process and a more successful robotic arm.



