#include <AFMotor.h>
#include <IRremote.h>  

AF_Stepper motor(200, 1);

IRrecv irrecv(A0);
decode_results results;

bool roating = false;
bool opening = false;
bool closing = false;

bool enabled = false;

unsigned long previousMillis = 0;        

unsigned long interval = 10000;

int redpin = A3;
int bluepin =A4;
int greenpin = A5;

void setup(){
  // button input
  pinMode(A1, INPUT);

  // stop button input
  pinMode(A2, INPUT);

  // input of clock
  pinMode(9, INPUT);
    
  // LEDs
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);

  // buzzer
  pinMode(2, OUTPUT);

  irrecv.enableIRIn();   // enable ir receiver module

  motor.setSpeed(15);
  motor.release();

  analogWrite(greenpin, 255);

  Serial.begin(9600);
  delay(1000);
}

void loop(){

  // get current timer since start of arduino
  unsigned long currentMillis = millis();

  // check if start button pressed
  if(digitalRead(A1) == 1 && roating == false){
      Serial.print("check if start button pressed");
    digitalWrite(2, HIGH);
    delay(300);
    digitalWrite(2, LOW);
    roating = true;
    opening = true;
    previousMillis = currentMillis;
    analogWrite(redpin, 255);
    analogWrite(greenpin, 142);
  }
  // check if time passed
  if(currentMillis - previousMillis >= interval && enabled == true){
          Serial.print("check if time passed");
    closing = true;
  }

//   check if clock input is high
          Serial.print("PIN 13 ");
          Serial.print(digitalRead(9));

  if(digitalRead(9) == 1 && roating == false){
          Serial.print("check if clock input is high");

    roating = true;
    opening = true;
    previousMillis = currentMillis;
    analogWrite(redpin, 255);
    analogWrite(greenpin, 142);
  }
  
  // check if stop button was pressed
  if(digitalRead(A2) == 1 && roating == true){
              Serial.print("check if stop button was pressed");
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);

    roating = false;
    closing = true;
    analogWrite(redpin, 255);
    analogWrite(greenpin, 0);
  }

  // start opening
  if(opening == true && enabled == false){
              Serial.print("start opening");

    enabled = true;
    motor.step(100, FORWARD, DOUBLE);
  }

  // start closing
  if(closing == true && enabled == true){
              Serial.print("start closing");
    opening = false;
    roating = false;
    enabled = false;
    closing = false;
    motor.step(100, BACKWARD, DOUBLE);
  }

  // read remote
  if (irrecv.decode(&results)){  
    Serial.print("irCode: ");    //print "irCode: "        
    Serial.print(results.value, HEX); //print the value in hexdecimal 
    Serial.print(",  bits: ");  //print" , bits: "         
    Serial.println(results.bits); //print the bits
    irrecv.resume();    // Receive the next value 
  }  
  Serial.print("PIN");
  Serial.print(digitalRead(A1));
  delay(600); //delay 600ms
  
}
