// this code is a test code for testing ir array sensors
//no pin setup required, as analog pins a getting used
// make sure values will 1024 for black line


void setup() 
{ Serial.begin(9600);   //Necessary to set up Serial port 
}  
void loop() 
{ Serial.print(analogRead(0)); 
Serial.print(' '); 
Serial.print(analogRead(1)); 
Serial.print(' '); 
Serial.print(analogRead(2)); 
Serial.print(' '); 
Serial.print(analogRead(3)); 
Serial.print(' '); 
Serial.print(analogRead(4)); 
Serial.print(' '); 
Serial.print(analogRead(5)); 
Serial.print(' '); 
Serial.print(analogRead(6)); 
Serial.println(' '); 
delay(800);              //Set the number to change frequency of readings.        
  }
