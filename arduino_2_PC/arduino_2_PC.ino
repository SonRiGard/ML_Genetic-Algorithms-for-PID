#define Interval_Time_GA_Process 1 //minute
float x=0,y=0,z=0;
float kp,ki,kd;
int T_pre_start_GA = 0;
uint32_t start_GA = 0 ;
uint32_t time_cur_process = 0 ;
uint32_t start_process = 0;

uint8_t flag_end_process = 0;
//example usage for genetic algorithm
#define POPULATION_SIZE  100
#define K  5 // tournament selection size
#define NUM_GENERATIONS  50

void receiver_Kpid();
void process_simulat ();
void end_process();
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  pinMode(25, OUTPUT);
}

void loop() {
  // while (!Serial.available());
  // x = Serial.readString().toFloat();
  // Serial.println(x);
  // while (!Serial.available());
  // y = Serial.readString().toFloat();
  // Serial.println(y);
  // while (!Serial.available());
  // z = Serial.readString().toFloat();
  // Serial.println(x + y + z);
  
  // start_GA = millis();
  // if (millis())
  //Serial.print("OK");
  receiver_Kpid();
  process_simulat();
  end_process();
  // uint32_t T_srt_cur_gen = millis();

  //   Serial.println(x);
}


void receiver_Kpid(){
  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  kp = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  ki = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  
  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  kd = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  delay(1000);
  Serial.print(millis());
  Serial.print(",");
  Serial.println("111");
}

void process_simulat (){
  start_process = millis();
  float err = 10.1;  
  while (millis()-start_process<20000){
    Serial.print(kp);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(flag_end_process);
    Serial.println("");
    delay(10);
  }
  delay(100);
  Serial.println("0,0,222");
  //delay(50);
}

void end_process(){
  //Serial.println("0,0,222");
  //flag_end_process= 0;
  delay(1000);
}