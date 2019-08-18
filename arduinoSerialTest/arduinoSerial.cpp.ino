
int incomedate = 0;
char buf[2] = "A";
unsigned int ix = 0;
void setup() {

  Serial.begin(57600); //设置串口波特率9600
  delay(5000);
  Serial.write(buf, 1);
}

void loop() {
#if 1
  if (Serial.available() > 0)//串口接收到数据
  {
    incomedate = Serial.read();//获取串口接收到的数据
    if (incomedate == 'H')
    {
      buf[0] += ix;
     // Serial.println("Good Job!");
      Serial.write(buf, 1);

      if(buf[0]<'z')
      {
        ix++;
      }
      else
      {
        ix=0;
      }

      delay(1000);
    }
  }
  #else
    delay(1000);
    Serial.write(buf, 1);
  #endif

  //delay(5);


//  Serial.println("Good Job!");
}