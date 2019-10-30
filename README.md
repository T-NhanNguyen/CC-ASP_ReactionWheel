# CC-ASP_ReactionWheel
###### Clark Aerospace club project developed for ESRA IREC Spaceport America Cup 2018

**Purpose:**
This project was intended to practice our methodology and prototyping skills by researching various approaches, fundamental understanding of force and inertias. Along with designing and implementing systems of 3D printed models, and analog signal devices.
There were two approaches involving the reaction wheel that we prototyped with. The first version was to have self-balancing systems much like the other engineering projects done by graduate students, however, we've come to realize that the endeavour towards that goal was beyond our skills at the time. With most of the designing and development did for the system, we've realized that couldn't just end it there, rather it would be in our best interest to diverge the project goal towards another practical application. Our second version of this project is the current self reorienting solar tracking project that we use photo sensors to locate a light source with the greatest illuminance. Using the encoded motors, it would theoretically create enough torque to rotate the system towards that light source for maximum coverage.

**Contents**
In this repo is the significant works done throughout the project, specifically, our thought process and a visual diagram for the components.
This is all optional, and you may ignore everything and focus on the wiring diagram and code, or revise and refine our approach by looking through our visuals and misc.
**some notes:**
There is a library for the motors and motor driver that we used to get the encoder motors to work properly. It is reccommend you use the provided version as we don't know the consequences of using other versions.

### How to get started
This is an Arduino file, with majority of the code being commented to help catch new and adept users up developing this project.
**1:**
Import the Makeblock-libraries-master.zip into your Arduino IDE
**2:**
Compile the Arduino with the Arduino IDE 
**3:**
Wire the circuit accordingly to the wiring diagram. However, if you decide to use your prefer pins on the Arduino, I've made the variables as intuitive as possible so you can manipulate it to your preference.
