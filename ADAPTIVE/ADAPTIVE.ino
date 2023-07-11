#include "I2Cdev.h"
#include "Wire.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "adaptive.h"
#include "control.h"

// // --- Индексы  для углов ----
// # define YAW         0
// # define PITCH       1
// # define ROLL        2

// --- Для прерывания ---
#define INTERRUPT_PIN 2         // пин 2 для прерывания
volatile bool mpuFlag = false;  // флаг прерывания готовности

// Объявления IMU
MPU6050 mpu;

/***** ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ *******/// --> лучше делать некторых из них локальными??????
// Характеристики пропеллера (коэффициенты сил и моментов)
float kTM=51.0; 
// Плечо сил тяги (для вычисления момента силы тяги)
float Lq=0.18; //18см  - большой коптер
//float Lq=0.05; //18см  - большой коптер

// Силы тяги (4 пропеллера)
float T1=0.0,T2=0.0,T3=0.0,T4=0.0;

//-------- ПИД регулятры -------------
// заданное значение периода работы алгоритма (в мс)
int TPID;        
// период работы алгоритма (в с)
float TAU;
// сигналы управления по 4 каналам - высота, тангаж, крен, рыскание 
float uh=0.0,uY[3]={0.0},uP[3]={0.0},uR[3]={0.0};
// значение, формируемое ПИД регуляторами (моменты по 3 угловым каналам)
float Myaw, Mpitch, Mroll;
float Mroll_filter=0.0;
// коэффициенты цифрового ПИД регулятора по угловым каналам [yaw, pitch, roll]
float qP[5]={0.0},qR[5]={0.0},qY[5]={00.0};
// вектор оценок углов [yaw, pitch, roll]
float theta[3]={0.0}, omg[3]={0.0},omg_prev[3]={0.0};
float gyrox[2]={0.0}, gyroy[2]={0.0}, gyroz[2]={0.0};
// заданное значение углов Эйлера [yaw, pitch, roll]
float thetaDesired[3]={0.0}, omgDesired[3]= {0.0};
// Текущая ошибка управления по углам [yaw, pitch, roll]
float thetaErrs[3],omgErrs[3];
// Вектор ошибок по углам 
float yawErr[3]={0.0}, pitchErr[3]={0.0},rollErr[3]={0.0};
float yawRateErr[3]={0.0}, pitchRateErr[3]={0.0},rollRateErr[3]={0.0};

//--------- для работы с DMP --------------
// Для тайминга
uint32_t startMillis;
// таймер
uint32_t tmr,tmr_motion;
// адрес обращения
uint8_t IMUAddress = 0x68;
// буффер для считывания данных из DMP
uint8_t fifoBuffer[64];
// кватернион поворота
Quaternion q;
// вектор гравитационного ускорения
VectorFloat gravity;
// вектор текущих показаний ГИРО и АССЕЛ
int16_t GYR[3];
VectorInt16 ACC,ACCb,ACCi;
float DV[3], V[3], R[3];
// ----- для управления моторами ----------------
// ШИМ сигналы управления моторами
int PWM1, PWM2, PWM3, PWM4;
// параметры кривой PWM-T
//float pMotor[3]={4.513E-6,3.332E-3,-63.35E-3};
float pMotor[3]={4.03E-6,2.69E-3,-45.30E-3};

// мин макс тяги
int THROTTLE_MINIMUM = 1000;                        
int THROTTLE_MAXIMUM = 2000; 
// программная тяга (начальная)
int THROLLTE0 = 1100;  
// мотора
Servo motor_1; // Motor front right
Servo motor_2; // Motor front left
Servo motor_3; // Motor back left
Servo motor_4; // Motor back right
// флаг, для выключения двигателей 
int flag=0;
//для управления вводом с клавиатуры
int comportin=0;
//------------------GA---------------
float GA_pitch_pid[3];
uint32_t start_process;

//----------------END-------------
//ADAPTIVECONTROL
float unn=0.0;
float vyr[5]={0.0},vwr[5]={0.0},vur[5]={0.0},cgl=0.0,an[5]={0.0}, bn[5]={0.0},pr[5]={0.0},qr[5]={0.0},pm[25]={0.0};
int mp, mq;
float alpha[3]={0.0};
float omgAD[3];
float MrollAD;
float dMroll=0.0,dMpitch=0.0,dMyaw=0.0;
/********************************************************************
********************** ПОДГОТОВКА К ЗАПУСКУ *************************
********************************************************************/
void setup() {

  // стандартная 
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  Serial.begin(115200);
  Serial.setTimeout(10);

  init_mpu6050();
  Initmotors();

  // ------ Инициализация параметров ПИД регуляторов ------------------
  // Период вызова алгоритма управления 
  TPID=8000;                  // в мкс (8мс)
  TAU= 0.01;                  // в с  (10мс)
    // расчет альфа для АУ
  SetAlpha(1.0,0.71, TAU,alpha);  

  //инициализация состояния ЛА (Для БИНС)
//  V[0]=0.0;V[1]=0.0;V[2]=0.0;
//  R[0]=0.0;R[1]=0.0;R[2]=0.0;
  initcntrl(vur, vyr, vwr,  an, bn, pr, qr,cgl,pm, mp, mq);
  //финиш

  // для начала считывания времени
  startMillis = millis();   
  tmr=micros();  
  tmr_motion=millis();
}

// прерывание готовности. Поднимаем флаг
void dmpReady() {
  mpuFlag = true;
}

/********************************************************************
********************** ГЛАВНАЯ ПРОГРАММА ****************************
********************************************************************/
void loop() { 
    receiver_Kpid();
    process_simulat();
    end_process();
}// end loop

/************ ВСМОПОГАТЕЛЬНЫЕ ФУНКЦИИ ****************************/
void init_mpu6050(void){
  
  // ---- MPU ------
  // Инициализация mpu
  Serial.println("Initialize MPU!");
  mpu.initialize();
  // There MUST be some delay between mpu.initialize() and mpu.dmpInitialize(); 
  // Typically, the delay should be longer that 5ms. Otherwise you will get a garbage sensor.
  delay(10);   
  // Инициализация DMP
  Serial.println("Initialize DMP!");
  mpu.dmpInitialize();  // в этой функции включается фильтр, задается gyro range ....
  
  // Автоматическая калибровка АССЕЛ и ГИРО

  // Serial.println("Calibrate Accels!");
  // mpu.CalibrateAccel(6); 
  // Serial.println("Calibrate Gyros!");
  // mpu.CalibrateGyro(6);
  // mpu.PrintActiveOffsets(); 
    
  // Ручная калибровка (ставим оффсеты из памяти)
  mpu.setXAccelOffset(-1408);
  mpu.setYAccelOffset(569);
  mpu.setZAccelOffset(2098);
  mpu.setXGyroOffset(52);
  mpu.setYGyroOffset(31);
  mpu.setZGyroOffset(24);


  // включить DMP
  mpu.setDMPEnabled(true);
  // инициализация прерывания
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpReady, RISING);
}
//------------------------------------------------------------------
//--------------Init motors-------------------------------------
void Initmotors(void){
  motor_1.attach(12);                                
  motor_2.attach(11);    //9 
  motor_3.attach(10);   //10
  motor_4.attach(9);  //11

  
  //Калибровка моторов ???
  //calibrateMotors();
  armMotors();
  pinMode(13, OUTPUT);
  Serial.println("SETUP: Motors calibrated");
  
}
//-----------------------------END------------------------------
//--------------Init PID----------------------
void Initpid(void){
    // вычисление коэффициентов цифровых ПИД регуляторов
  DIGPIDCOEFF1(qP,GA_pitch_pid[0],GA_pitch_pid[1],GA_pitch_pid[2],TAU); // Pitch  
  DIGPIDCOEFF1(qR,0.58,0.1,0.04,TAU);        //Roll
  DIGPIDCOEFF1(qY,1.42,0.001,0.001,TAU);       //Yaw
}
//--------------END---------------------------
//--------------------GA FUNCTIONS_---------
void receiver_Kpid(void){
  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  GA_pitch_pid[0] = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  GA_pitch_pid[1] = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  
  while (!Serial.available()){
    digitalWrite(25, HIGH);
  };
  GA_pitch_pid[2] = Serial.readString().toFloat();
   digitalWrite(25, LOW);

  delay(1000);
  Serial.print(millis());
  Serial.print(",");
  Serial.println("111");
}

void process_simulat (){
  start_process = millis();
  while(millis()-start_process<10000){
  // Время, считанное с последнего момента получения данных с DMP

    static uint32_t DLT, TM, DTL_MOTION,flagYAW0=0;
    float DT;
    uint8_t zerotime;
    DLT=micros()-tmr;   
    TM=millis()-startMillis;
    DTL_MOTION=millis()-tmr_motion;

    // При готовности DMP и превышении интервала ждания
    if (DLT>=TPID && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
        // сброс таймера  
        DT=(micros()-tmr)*1.E-6; 
        tmr = micros();           
        // получение данных с DMP (кватернион)   
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // вычисление гравитационного ускорения
        mpu.dmpGetGravity(&gravity, &q);
        // вычисление углов
        mpu.dmpGetYawPitchRoll(theta, &q, &gravity);        
        // получение показаний ГИРО
        mpu.dmpGetGyro(GYR,fifoBuffer);
        // расчет и фильтрация угловой скорости 
        ANGLERATEFILTER(GYR, omg,DT,40.0);           
    
        // задающие сигналы (при стабилизации они равны 0)
/*        if (flagYAW0==0)
        {
            thetaDesired[YAW]=theta[YAW];
            flagYAW0=1;
        }
*/
        thetaDesired[YAW]=0.0;
        thetaDesired[PITCH]=0.0;
        thetaDesired[ROLL]=0.0*M_PI/180;
        omgDesired[YAW]=0.0;omgDesired[PITCH]=0.0;omgDesired[ROLL]=0.0;

        thetaDesired[PITCH]=PROGCONTROL(millis()-start_process); 
        // закон изменения управляемого парамета 
        
        // ПИД по угловым скоростям
        GetErrs(thetaDesired, theta, thetaErrs);
        //SignalLMT(thetaErrs,30.0);
        //SignalDEADZONE(thetaErrs,1.0);
        FindAngleRateDesired(thetaErrs, omgDesired);

//        if (TM>=10000)
//          omgDesired[ROLL]=PROGCONTROL(TM);

        GetErrs(omgDesired, omg, omgErrs);
        //SignalDEADZONE(omgErrs,5.0);
      
        vectorstore(yawRateErr,omgErrs[YAW]); 
        vectorstore(pitchRateErr,omgErrs[PITCH]); 
        vectorstore(rollRateErr,omgErrs[ROLL]); 

        // Выполнение алгоритмов ПИД регулвторов
        PID_DIGITAL(qY,yawRateErr,uY);    
        PID_DIGITAL(qP,pitchRateErr,uP);
        PID_DIGITAL(qR,rollRateErr,uR);

        // Пересчет требуемого значения моментов (с учетом физическими ограничениями)
        Myaw   = SATURATION(uY[0], 2.5, -2.5);        //рыскание
        Mpitch = SATURATION(uP[0], 2.5, -2.5);        //тангаж
        Mroll  = SATURATION(uR[0], 2.5, -2.5);        //крен

        // Для тестирования выключаем некоторые каналы
        uh=0.0; 
        Myaw=0.0;
        Mroll=0.0;
  
        dMyaw=0.0; dMpitch=-0.018; dMroll=0.0;

        // Расчет требуемого значения сил тяги для всех моторов
        T1=0.25*uh+0.25*(Mpitch+dMpitch)/Lq+0.25*(Mroll+dMroll)/Lq+0.25*Myaw*kTM;
        T2=0.25*uh+0.25*(Mpitch+dMpitch)/Lq-0.25*(Mroll+dMroll)/Lq-0.25*Myaw*kTM;
        T3=0.25*uh-0.25*(Mpitch+dMpitch)/Lq-0.25*(Mroll+dMroll)/Lq+0.25*Myaw*kTM;
        T4=0.25*uh-0.25*(Mpitch+dMpitch)/Lq+0.25*(Mroll+dMroll)/Lq-0.25*Myaw*kTM;

        THROLLTE0=1300;

        // Расчет ШИМ сигналов для моторов (по кривой PWM-T)
        PWM1=T2PWM(T1,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM2=T2PWM(T2,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM3=T2PWM(T3,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM4=T2PWM(T4,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;

        // Печать  
        if (flag==0)  {
          Serial.print(thetaErrs[0]*180/M_PI);
          Serial.print(",");
          Serial.print(millis());
          Serial.print(",");
          Serial.print("0");
          Serial.println("");
          delay(10);      
        }    
        //PRINTRES();
    }

    //Контроль работы моторов (выключаем все после несколько секунд)
    if (abs(theta[PITCH])>=45*M_PI/180 || abs(theta[ROLL])>=45*M_PI/180 || abs(theta[YAW])>=90*M_PI/180)
    {
        flag = 1 ;
    }

    if (flag==1)
    {
      StopDrone();
    }

    // Передаем ШИМ сигналы в мотора
    motor_1.writeMicroseconds(PWM1);
    motor_2.writeMicroseconds(PWM2);
    motor_3.writeMicroseconds(PWM3);
    motor_4.writeMicroseconds(PWM4);
  }

  delay(100);
  Serial.println("0,0,222");//send stop ACk To PC
  //delay(50);

}

void end_process(){
  //Serial.println("0,0,222");
  //flag_end_process= 0;
  delay(1000);
}
//-------------------------END GA FUNCTION -----
void PRINTRES()
{
  
  int INDX=PITCH;
 
  Serial.print(Mpitch); Serial.print(",");
  Serial.print(thetaDesired[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(theta[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(omgDesired[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(omg[INDX] * 180.0 / M_PI);//Serial.print(",");
  Serial.println();
}

void SignalLMT(float x[], float threshold)
{
    x[0] = LMT(x[0],threshold*M_PI/180);    
    x[1] = LMT(x[1],threshold*M_PI/180);
    x[2] = LMT(x[2],threshold*M_PI/180); 
}

void SignalDEADZONE(float x[], float threshold)
{
    x[0] = DEADZONE(x[0],threshold*M_PI/180);    
    x[1] = DEADZONE(x[1],threshold*M_PI/180);
    x[2] = DEADZONE(x[2],threshold*M_PI/180); 
}

// Вычисление ошибок управления 
void GetErrs(float xd[], float x[], float dx[]) {
    dx[0]  = xd[0]   - x[0];//*180/M_PI;
    dx[1]  = xd[1]   - x[1];//*180/M_PI;
    dx[2]  = xd[2]   - x[2];//*180/M_PI;
}


// Калибровка моторов
void calibrateMotors() {
//   Serial.println("Calib motors!");
//   motor_1.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_2.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_3.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_4.writeMicroseconds(THROTTLE_MAXIMUM);
// delay(7000);
    motor_1.writeMicroseconds(THROTTLE_MINIMUM);
    motor_2.writeMicroseconds(THROTTLE_MINIMUM);
    motor_3.writeMicroseconds(THROTTLE_MINIMUM);
    motor_4.writeMicroseconds(THROTTLE_MINIMUM);
    delay(5000);
}

void armMotors() {
    motor_1.writeMicroseconds(THROTTLE_MINIMUM);
    motor_2.writeMicroseconds(THROTTLE_MINIMUM);
    motor_3.writeMicroseconds(THROTTLE_MINIMUM);
    motor_4.writeMicroseconds(THROTTLE_MINIMUM);
    delay(5000);
}

void StopDrone()
{
  PWM1=1000;
  PWM2=1000;
  PWM3=1000;
  PWM4=1000;
}

void ANGLERATEFILTER(int16_t GYR[], float omg[],float DT, float w0)
{
    static float gyrox[2]={0.0}, gyroy[2]={0.0}, gyroz[2]={0.0};
    float omgx, omgy, omgz;

    // расчет угловой скорости 
    gyrox[0] = (float)GYR[0]/32768 * 2000/180*M_PI;   //в рад/с   //roll
    gyroy[0] =-(float)GYR[1]/32768 * 2000/180*M_PI;               //pitcj
    gyroz[0] = -(float)GYR[2]/32768 * 2000/180*M_PI;              //yaw
    
    // фильтрация показаний гироскопов
    omgx=DigLowPassFil(omg[0], gyrox[0], gyrox[1],DT,w0);
    omgy=DigLowPassFil(omg[1], gyroy[0], gyroy[1],DT,w0);
    omgz=DigLowPassFil(omg[2], gyroz[0], gyroz[1],DT,w0);

    // запомнить
    gyrox[1]=gyrox[0];gyroy[1]=gyroy[0];gyroz[1]=gyroz[0];

    // вывод
    omg[ROLL]=omgx; omg[PITCH]=omgy; omg[YAW]=omgz;       
}

void ACCELERATIONFILTER(VectorInt16 *ACC, float a[],float DT, float w0)
{
    static float accx[2]={0.0}, accy[2]={0.0}, accz[2]={0.0};
    float ax, ay, az;
    

    accx[0]=ACC->x/16384.0f;
    accy[0]=ACC->y/16384.0f;
    accz[0]=ACC->z/16384.0f;

    // фильтрация показаний гироскопов
    ax=DigLowPassFil(a[0], accx[0], accx[1],DT,w0);
    ay=DigLowPassFil(a[1], accy[0], accy[1],DT,w0);
    az=DigLowPassFil(a[2], accz[0], accz[1],DT,w0);

    // запомнить
    accx[1]=accx[0];accy[1]=accy[0];accz[1]=accz[0];

    // вывод
    a[0]=ax; a[1]=ay; a[2]=az;       
      // a[0]=DEADZONE(ax, 0.05);
      // a[1]=DEADZONE(ay, 0.05);
      // a[2]=DEADZONE(az, 0.05);  
}

// Программое управление
float PROGCONTROL1(uint32_t tmr)
{  
	float thetaDesired=0.0;
  uint32_t tmr1=tmr%400;
  if (tmr1<200)
  {
      thetaDesired=-0.5;  //roll=0.1, pitch=0.05  
  }
  else  
  {
      thetaDesired=0.5;  
  }

	return thetaDesired;
}

