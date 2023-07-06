// void setup() {
//   // put your setup code here, to run once:
// Serial.begin(115200);
// //pinMode(13,OUTPUT);
// }
// float f;

// void loop() {
//  if(Serial.available())
//       {f=Serial.parseFloat();
//         //Serial.(f);
//         Serial.println(f+1.00);   
//       }
//     //delay(1000); 
        
  
//   // Serial.println("a");
//   // delay(100);
// }
// //count=count+1;
//       //delay(1);
// //}

float kp,ki,kd;
int flag_new_value = 0;
float temp;
void setup() {
 Serial.begin(9600);
  Serial.setTimeout(1);

 while( temp != 111.1){
   temp = Serial.readString().toFloat();
 };
 Serial.print(222.2);
  delay(2000);
}

void loop() {
 while (!Serial.available());
  if (flag_new_value == 0){ 
    // kp = Serial.readString().toFloat();
    // ki = Serial.readString().toFloat();
    kd = Serial.readString().toFloat();
    float sum = kd+1;
    Serial.print(sum);
  }
}


