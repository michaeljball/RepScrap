/***************************************************************************************
*  Lin_Enc_02.ino   05-12-2014   unix_guru at hotmail.com   @unix_guru on twitter
*  http://arduino-pi.blogspot.com
*
*  This sketch allows you to run two salvaged printer carriages for X/Y axis using their 
*  linear encoder strips for tracking. 
*  This example uses the Arduino PID Library found at:
*  https://github.com/br3ttb/Arduino-PID-Library/archive/master.zip
*
*  Hardware Interrupt 0 on Digital pin2 is used to determine X-Axis position
*  Hardware Interrupt 1 on Digital pin3 is used to determine Y-Axis position
*  PinchangeInterrupt is used to identify the Zero Endstop for X and Y axis

*****************************************************************************************/
