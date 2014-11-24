/*----------------------------------------------------------------------------------------------
 
 Control stick compass translation for Phantom Vision 2+
 
 This adds a new and more intuitive flying mode for the Phantom 
 that is relative to the pilot, not the quadcopter. 
 
 I added a small microcontroller (a "Teensy" from PJRC,$20) to the remote control box, 
 which intercepts and reads the signals from the right joystick. It then generates new 
 signals for a simulated joystick that is fed to the DJI controller board. A combination 
 3-axis magnetometer plus accelerometer (LSM303D from Pololu, $10) acting as a compass
 tells my microcontroller which way the RC box is pointing, and the software applies 
 a rotational transformation of the X-Y deflection to the simulated joystick.
 
 It's a totally different and easy flying experience for a novice like me. As long as 
 I keep facing the phantom, the joystick works intuitively: "forward" makes it go away 
 from me,"back" make it come towards me, "right" sends it to my right, and "left" to my 
 left. It doesn't matter what the takeoff orientation was, where it currently is, or 
 how it is rotated.  
 
 Now I can spin it to have the camera point whereever I want, and it doesn't affect 
 how it flies. I don't have to remember where it was pointing when it took off and 
 adjust for that in my head. I just have to keep the remote control box pointed to it.
 
 -- Len
 ---------------------------------------------------------------------------------------------*/
/*   (C) Copyright 2014, Len Shustek
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of version 3 of the GNU General Public License as
 *   published by the Free Software Foundation at http://www.gnu.org/licenses,
 *   with Additional Permissions under term 7(b) that the original copyright
 *   notice and author attibution must be preserved and under term 7(c) that
 *   modified versions be marked as different from the original.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 ---------------------------------------------------------------------------------------------*/
// 9 Nov 2013, L. Shustek, version 1.0: try with Honeywell HMC5883L magnetometer
//16 Nov 2014, L. Shustek, version 2.0: change to ST LM303D magnetometer/accelerometer

// This is built by the "Teensyduino" Arduino-like integrated development environment.
// See https://www.pjrc.com/teensy/teensyduino.html.

#define DEBUG 1      // debugging output?
#define CALIBRATE 0  // also include calibration phase?

#include <Wire.h>
#include <LSM303.h>  // from https://github.com/pololu/lsm303-arduino

// our hardware

#define LED    17  // diagnostic LED; active high
#define XL_IN  A10 // analog X left joystick input
#define YL_IN  A11 // analog Y left joystick input
#define X_IN   14  // analog X right joystick input
#define Y_IN   15  // analog Y right joystick input
#define X_OUT  23  // PWM X simulated right joystick output
#define Y_OUT  22  // PWM Y simulated right joystick output
#define SW1    20  // toggle switch pole 1 (up)
#define SW2    21  // toggle switch pole 2 (down)

#if DEBUG
#define THRESHOLD_DEG 3  // how many compass degrees (of 360) is a significant change to display
#define THRESHOLD_POS 8  // how much joystick deflection (of 1024) is a significant change to display
#endif

enum {  // mode based on our switch position 
  MODE_OFF, MODE_TRANSLATE, MODE_CALIBRATE} // bottom, middle, top
current_mode;
int last_sw1=3, last_sw2=3;  // last value of SPDT switch pole inputs


void check_mode_change(void) { // ***** check for mode switch position change
  int new_sw1 = digitalRead(SW1);
  int new_sw2 = digitalRead(SW2);
  if (last_sw1!=new_sw1 || last_sw2!=new_sw2) { // switch change?
    delay(50); // debounce delay
    new_sw1 = digitalRead(SW1); // reread stable values
    new_sw2 = digitalRead(SW2);
    if (new_sw1==LOW) current_mode = MODE_CALIBRATE;
    else if (new_sw2==LOW) current_mode = MODE_OFF;
    else current_mode = MODE_TRANSLATE;
#if DEBUG
    Serial.print("New mode: ");
    Serial.println(current_mode);
#endif
    last_sw1 = new_sw1;
    last_sw2 = new_sw2;
  }
}

//  compass (magnetometer + accelerometer) routines
//  we use the polulu.com library; see http://www.pololu.com/product/2127 
//  and https://github.com/pololu/lsm303-arduino

LSM303 compass;

#if DEBUG
void mag_calibrate(void) {  // display calibration data until switch is changed to "OFF"
  LSM303::vector<int16_t> running_min = {
    32767, 32767, 32767
  }
  , running_max = {
    -32768, -32768, -32768
  };
  do {
    char report[80];
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}   max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z, running_max.x, running_max.y, running_max.z);
    Serial.println(report);
    delay(250);
    check_mode_change();
  } 
  while (current_mode != MODE_OFF);
}
#endif

void mag_initialize(void) {  // start up the magnetometer/accelerometer
  char msg[60];
#if DEBUG
  do { // wait for switch to TRANSLATE so we have a chance to start the serial monitor before any output
    check_mode_change();
  }  
  while (current_mode != MODE_TRANSLATE);
#endif
  Wire.begin();
  if (! compass.init()) { 
#if DEBUG
    Serial.println("*** compass init failed ***");
#endif
    while(1) ;
  }
  compass.enableDefault();
  compass.writeReg(LSM303::CTRL5, 0x68); // change magnetometer from 6.25 Hz rate to 12.5 Hz rate

  // calibration values 
  compass.m_min = (LSM303::vector<int16_t>){ // -32767, -32767, -32767 };
    -3110,  -2199,  -4467
  };
  compass.m_max = (LSM303::vector<int16_t>){ // +32767, +32767, +32767 };
    +2466,  +3912,  +1675 
  };
  //min: { -3067,  -2199,  -4419}   max: { +1697,  +3912,  +1669}
  //min: { -3110,  -1984,  -4316}   max: { +2410,  +3880,  +1675}

#if DEBUG 
  sprintf(msg, "device type %d, device ID %02X \n", 
  compass.getDeviceType(), compass.readReg(LSM303::WHO_AM_I));
  Serial.print(msg);
  if (CALIBRATE) mag_calibrate();
#endif
}

float mag_read(void) {      // read current magnetometer setting
  compass.read();
  return (PI/180.) * compass.heading();  // tilt-compensated direction in radians
}

void setup(void) { //**********  SETUP  *****************

#if DEBUG
  Serial.begin(57600);
  Serial.println("Starting Phantom joystick remapper");
#endif

  pinMode(LED, OUTPUT);        // our LED
  pinMode(X_IN, INPUT);        // analog inputs from joysticks
  pinMode(Y_IN, INPUT);
  pinMode(XL_IN, INPUT);
  pinMode(YL_IN, INPUT);
  pinMode(X_OUT, OUTPUT);      // our PWM analog simulated right joystick outputs
  pinMode(Y_OUT, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);  // mode switch "up" pole
  pinMode(SW2, INPUT_PULLUP);  // mode switch "down" pole

  analogWriteFrequency(X_OUT, 10000);  // 10Khz PWM (timer 0, so also changes Y-OUT)
  analogWriteResolution(10);           // 10-bit A-to-D resolution
  // The potentiometer rotation covers only 1.4K out of 8.3K, so we get a 3.3(1.4/8.3) = 0.5v change, from 1.4v to 1.9v.
  // With the 10-bit A-to-D, that's about 0-160 for full range of joystick deflection, or about 0.6% resolution
  // about 0.003 volts/division.

  mag_initialize();            // start magnetometer
}

void print_joystick(char *title, int x, int y) {  // print joystick position in raw volts, and maybe degrees if not centered
#if DEBUG
  char msg[80];
  int xmid=512, ymid=512; // joystick neutral positions
  int xdef, ydef;
  float angle;
  sprintf(msg,"%s x=%.2fv, y=%.2fv", title, (float)x*3.3/1024., (float)y*3.3/1024.);
  Serial.print(msg);
  xdef = (1024-x)-xmid;
  ydef = (1024-y)-ymid;
  // threshold for avoiding a false center position angle: 0.05 volts, or 15 out of 1024
  if (abs(xdef)>15 || abs(ydef)>15) { // if joystick is not within about 10% of the center
    angle = atan2(ydef,xdef);
    if (angle<0) angle += 2.*PI;
    sprintf(msg,", %d deg", (int) (angle*(180/PI)));

    Serial.print(msg);
  }
#endif
}

void loop(void) {  

  int x, oldx, newx, center_x, leftx;
  int y, oldy, newy, center_y, lefty;
  float RC_heading, takeoff_heading=0, rotation_angle;
#if DEBUG
  int deg, olddeg=0;
  boolean joystick_changed;
  char msg[60];
#endif

  oldx=0; 
  oldy=0;
  center_x = analogRead(X_IN);  // remember joystick neutral position
  center_y = analogRead(Y_IN);

  while (1) { //***************** MAIN LOOP ********************

    check_mode_change();     // check for mode switch change

      // Read the compass

    RC_heading =  mag_read(); // read the heading
    RC_heading += 0.2388;   // Add the magnetic declination from http://www.magnetic-declination.com/
    // In Portola Valley CA it is 13* 41' E, which is 0.2388 radians
    // (It actually doesn't matter for this application, except for debugging messages.)
    if (RC_heading < 0)   // Correct for when signs are reversed.
      RC_heading += 2*PI;
    if (RC_heading > 2*PI) // Check for wrap due to addition of declination.
      RC_heading -= 2*PI;
#if DEBUG
    deg = RC_heading * 180./PI;  // Convert radians to approximate degrees, and print if significantly changed
    int diff = abs(deg-olddeg);
    if (diff>180) diff=360-diff; // circular difference
    if (diff>=THRESHOLD_DEG) {
      sprintf(msg, "RC heading: %3d\n", deg);
      Serial.print(msg);
      olddeg = deg;
    }
#endif

    // Read the current right joystick position 

    x = analogRead(X_IN); 
    y = analogRead(Y_IN);
#if DEBUG    
    joystick_changed = false;
    if (abs(x-oldx)>=THRESHOLD_POS || abs(y-oldy)>=THRESHOLD_POS) { // print if changed
      char msg[40];
      print_joystick("stick", x, y);
      joystick_changed = true;
      oldx=x;
      oldy=y;
    }
#endif 

    // Output a  simulated joystick

    newx=x; 
    newy=y; // assume no rotational translation

    if (current_mode == MODE_CALIBRATE) { // mode switch is "up"
      takeoff_heading = RC_heading; // memorize current remote control rotation
      // We expect the phantom to be in NAZA "course lock" mode, and that this
      // calibration is done with the remote pointing in the takeoff direction it locks to.
      center_x = analogRead(X_IN);  // also recalibrate joystick neutral position
      center_y = analogRead(Y_IN);  // (just in case it drifts)
      for (byte i=0; i<3; ++i) { // flash the light to show we did it
        digitalWrite(LED, HIGH); 
        delay(100);
        digitalWrite(LED,LOW);  
        delay(100);
      }
    }
    else if (current_mode == MODE_TRANSLATE) { // mode switch is "middle": do a rotational transform
      // First: check if the user is doing a "Combination Stick Command" (CSC) to start or stop the motors,
      // which is pushing both sticks to their bottom center positions.  If so, we should just pass that 
      // through to the Phantom's controller without rotating the right joystick. Hopefully the craft is 
      // on the ground for this maneuver!
      leftx = analogRead(XL_IN); 
      lefty = analogRead(YL_IN);
      int csc_min = (int)(1.42/3.3*1024.); // need some way to calibrate these voltages dynamically
      int csc_max = (int)(1.85/3.3*1024.); // because they may vary from RC to RC
      // Note that the two sticks are opposite polarity both for X and Y
      if (leftx < csc_max || lefty > csc_min || x < csc_max || y < csc_max) { // not CSC, so rotate
        digitalWrite(LED, HIGH); // on
        rotation_angle = takeoff_heading - RC_heading;
        // do a Euclidean coordinate rotation relative to the neutral joystick position
        // x' = x cos(t) - y sin(t)
        // y' = x sin(t) + y cos(t)
        newx = center_x + (int)((x-center_x)*cos(rotation_angle) - (y-center_y)*sin(rotation_angle));
        newy = center_y + (int)((x-center_x)*sin(rotation_angle) + (y-center_y)*cos(rotation_angle));
#if DEBUG
        if (joystick_changed) { // print if changed
          Serial.print(", rotation "); 
          Serial.print((int)(rotation_angle * 180./PI));
          print_joystick(", new stick", newx, newy);
        }
#endif
      }
    }
    // else current_mode == MODE_OFF, so leave joystick unchanged

#if DEBUG
    if (joystick_changed) Serial.println();
#endif
    analogWrite(X_OUT, newx); // output simulated joystick position
    analogWrite(Y_OUT, newy);
    digitalWrite(LED, LOW);  // off
    delay (90);  // magnetometer rate is 12.5 Hz, or 80 msec
  }
}

