int incomedate = 0;
char buf[2] = "A";
unsigned int ix = 0;
void setup() {

  Serial.begin(57600);
  delay(5000);
  Serial.write(buf, 1);
}

void loop() {
#if 1
  if (Serial.available() > 0)
  {
    incomedate = Serial.read();
    if (incomedate == 'H')
    {
      buf[0] += 1;
     // Serial.println("Good Job!");
      Serial.write(buf, 1);

      if(buf[0]>'z')
      {
        buf[0]='A';
      }

     // delay(1000);
    }
  }
  #else
    delay(1000);
    Serial.write(buf, 1);
  #endif
}

