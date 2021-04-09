int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(7, OUTPUT);
  pinMode(5, INPUT);
  
  
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);
    if(incomingByte ==48){ //TASTE 0
      Serial.println("alles AUS");

      dvrTrigger();
      delay(10000);
      digitalWrite(7, LOW);
      
      }
      if(incomingByte ==49){//TASTE 1
      Serial.println("alles EIN");
      digitalWrite(7, HIGH);
      delay(10000);
      dvrTrigger();     
      }

      if(incomingByte ==52){ //TASTE 4
      Serial.println("DVR EIN");
      digitalWrite(7, HIGH);    
      }

      if(incomingByte ==53){ //TASTE 5
      Serial.println("DVR AUS");
      digitalWrite(7, LOW);    
      }

      if(incomingByte ==54){ //TASTE 6
      Serial.println("DVR Trigger");
      dvrTrigger();     
      }
    
  }
}

void dvrTrigger(){
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(1000);
  pinMode(5, INPUT);
  
  }
