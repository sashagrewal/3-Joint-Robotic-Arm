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
| Adeept Robotic Arm Kit  | 1  | $64.99  | adeept.com/adeept-arduino-compatible-diy-5-dof-robotic-arm-kit-for-arduino-uno-r3-steam-robot-arm-kit-with-arduino-and-processing-code_p0118.html  |
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




