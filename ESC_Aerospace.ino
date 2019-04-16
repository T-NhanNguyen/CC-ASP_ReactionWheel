 #include <Servo.h>

Servo ESC;    //Uses servo syntax to send signal to ESC. ESC is the name
int pot = A0; //Potentiometer pin is A0 
int button = A1; //button for reverse 

void setup() 
{
  ESC.attach(2);        //ESC white wire is connected to D2 on board. red 5V, balck GND.
  Serial.begin(9600);   //For debugging and monitoring purposes
}
void loop() 
{
  int outputvelocity = speedfunc(analogRead(pot));  //setting the value of the pot value into a variable
  ESC.write(outputvelocity);      //Send the value out to the ESC.
  Serial.println(outputvelocity); //To monitor the values of the pot
  

  while (button == HIGH)
  {
    Serial.println("ON");
  }
}
int speedfunc(int setspeed) //capping out the potentiometer values.
{
  int velocity = map(setspeed, 0, 1023, 0, 179); //map(var, lower range, upper range, lower cap, upper cap)
  // This minimum value to start the motor is 28% upper capped value.  max rpm will be at 66%
  return velocity;
}


