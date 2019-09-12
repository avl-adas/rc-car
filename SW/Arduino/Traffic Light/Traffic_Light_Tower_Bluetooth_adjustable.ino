
#include <SoftwareSerial.h>


const int greenpin = 6;
const int yellowpin = 3;
const int redpin = 5;
const int switch1pin = 8;
const int switch2pin = 12;


int redtime = 1000;
int yellowtime = 100;
int greentime = 600;

const int brightness = 255;

int timer = 0;
int colorstate = 0; //0=red 1=yellow 2=green

int switch1state = 0;
int switch2state = 0;

int switcher = 0;
int switched = 0;

int autostate = 0;

int bluetooth = 0;


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);


pinMode(greenpin,OUTPUT);//green
pinMode(yellowpin,OUTPUT);//yellow
pinMode(redpin,OUTPUT);//red

pinMode(switch1pin,INPUT);//switch 1
pinMode(switch2pin,INPUT);//switch 2

//Serial.begin(38400);
}// end setup

void loop() {
  // put your main code here, to run repeatedly:

//switch1state = digitalRead(switch1pin);
//switch2state = digitalRead(switch2pin);


if(Serial.available() > 0){
  bluetooth = Serial.read();//inputs the signal from bluetooth
  //Serial.println(autostate);

}

//bluetooth signals 0-2 indicate LED color, 4 indicates automatic mode
if (bluetooth < 3 ) {
  colorstate = bluetooth;
  autostate = 0;
}
if (bluetooth == 4 ){
  autostate = 1;
}
if (bluetooth >= 50 and bluetooth < 100)
{
  redtime = ((bluetooth - 50)*100);
}
if (bluetooth >= 100 and bluetooth < 150)
{
  yellowtime = ((bluetooth - 100)*100);
}
if (bluetooth >= 150 and bluetooth < 200)
{
  greentime = ((bluetooth - 150)*100);
}
/*if (switch2state == HIGH)
{
 
  if (switcher == 2)
  {
    switched = 1;
  }
  else if (switcher == 1)
  {
    switched = 0;
  }
}

if(switch2state == LOW)
{
 
 if (switcher == 1)
 {
  switched = 1;
 }
 else if (switcher == 2)
 {
  switched = 0;
 }
  
}
*/



if (autostate == 1){//automatic loop

  if (colorstate == 0){
    if (timer < redtime){

      timer++;
    }
      else {
      colorstate = 2;
      timer = 0;
      }
    
    }
    else if (colorstate == 2){
      if (timer < greentime){
        timer++;
      }
      else {
        colorstate = 1;
        timer = 0;
      }
    }
    else if (colorstate == 1){
      if(timer < yellowtime){
        timer++;
      }
      else {
        colorstate = 0;
        timer = 0;
      }
    }
  
  
  }// end automatic loop


/*if (switch1state == LOW){//switch mode

    if (switched == 1)
    {
      if (colorstate == 0)
      {
        colorstate = 2;
        Serial.println("red to green");
      }
       else if (colorstate == 1)
      {
        colorstate = 0;
        Serial.println("yellow to red");
      }
      else if (colorstate ==2)
      {
        colorstate = 1;
        Serial.println("green to yellow");
      }
    }

  
}//end switch mode
*/




if (colorstate == 0){//red
  if (redtime > 0 or autostate == 0){
  analogWrite(greenpin, 0);
  analogWrite(yellowpin, 0);
  analogWrite(redpin,brightness);
  }
}
if (colorstate == 1){//yellow
  if (yellowtime > 0 or autostate == 0){
  analogWrite(greenpin, 0);
  analogWrite(yellowpin, brightness);
  analogWrite(redpin,1);
  }
  }
if (colorstate == 2){//green
  if (greentime > 0 or autostate == 0){
  analogWrite(greenpin, brightness);
  analogWrite(yellowpin, 0);
  analogWrite(redpin,0);
  }
}





if (switch2state == HIGH)
{
  switcher = 1;
}

if(switch2state == LOW)
{
 switcher = 2;
  
}





//Serial.println(colorstate);
Serial.println(bluetooth);


delay(10);
}
