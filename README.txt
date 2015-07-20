For documentation visit http://wiki.asctec.de/

Copied right reserved by AscTec;

Customized by Wei Ding. http://weiding.me/

User is able to send desired attitude and euler angular velocity directly to Autopilot. 

State feedback from outside measurement is able to send to Autopilot for more precise control.

Sample control file will be uploaded soon

ctrl_mode 0x04
Desired roll angle 0x0F00
Desired pitch angle 0x0F01
Desired yaw angle 0x0F02
Desired roll velocity 0x0F03
Desired pitch velocity 0x0F04
Desired yaw velocity 0x0F05

Desired Accleration in Z axis(body) 0x0F06
roll angle feedback 0x0F07
pitch angle feedback 0x0F08
yaw angle feedback 0x0F09
