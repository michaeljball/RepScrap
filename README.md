/**********************************************************************************************/
* 3Dprint-teensy-Demo1.ino 02-12-2015 unix_guru at hotmail.com @unix_guru on twitter
* http://arduino-pi.blogspot.com
*
* This sketch allows you to run two salvaged printer carriages for X/Y axis using their
* linear encoder strips for tracking on a Teensy 3.1 uController module based on Freescale's
* Kinetis K20DX256 ARM Coretex-M4 processor with two hardware based Quadrature Decoder channels
*
* The purpose of this sketch is to test and tune the PID loops for two AXIS simultaneously
*
* This example uses the Arduino PID Library found at:
* https://github.com/br3ttb/Arduino-PID-Library/archive/master.zip
*
*
* The most important part of this entire project however came from Trudy Benjamin with
* her FTM based Quadrature Decoder library.
* https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x
*
* Thank you Trudy.
*
*
/************************************************************************************************/
