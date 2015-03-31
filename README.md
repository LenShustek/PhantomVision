PhantomVision
=============

DJI Phantom Vision+ Quadcopter remote controller firmware

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

The images in this directory are of the schematic, and the interior of the controller.
For a discussion on the DJI forum, see http://forum.dji.com/forum.php?mod=viewthread&tid=2287.
