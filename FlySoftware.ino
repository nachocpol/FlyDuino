#include <bmi160.h>
#include <bmi160_defs.h>
#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>

#define PI 3.1415926536f

// ------------------------------ //
//            IMU                 //
// ------------------------------ // 

//#define IMU_DEBUG
#define IMU_ACC_RANGE 2      // 2, 4, 8                   (+/-g)
#define IMU_GYRO_RANGE 2000  // 2000, 1000, 500, 250, 125 (º/s)
#define STD_G 9.80665f  

void IMUDelay(uint32_t ms)
{
  delay(ms);
}

int8_t IMUWrite(uint8_t addr,uint8_t reg,uint8_t* data, uint16_t dataSize)
{
#if defined(IMU_DEBUG)
  Serial.print("Write: ");
  Serial.print(addr);
  Serial.print(" ");
  Serial.print(reg);
  Serial.print(" ");
  Serial.println(dataSize);
#endif
  
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data, dataSize);
  Wire.endTransmission(); 
}

int8_t IMURead(uint8_t addr,uint8_t reg,uint8_t* data, uint16_t dataSize)
{  
#if defined(IMU_DEBUG)
  Serial.print("Read: ");
  Serial.print(addr);
  Serial.print(" ");
  Serial.print(reg);
  Serial.print(" ");
  Serial.println(dataSize);
#endif

  // Request read from [reg, reg+dataSize]: 
  Wire.beginTransmission(addr);
  for(uint8_t r = 0; r != dataSize; ++r)
  {
    Wire.write(reg + r);  
  }
  Wire.endTransmission();

  // Retrieve data
  uint16_t sizeReceived = 0;
  Wire.requestFrom(addr, dataSize);
  while(Wire.available())
  {
    data[sizeReceived++] = Wire.read();
  }
  
  // Perform some error checking:
  if(sizeReceived != dataSize)
  {
#if defined(IMU_DEBUG)
    Serial.print("There was a problem retrieving data from the IMU! Requested:"); 
    Serial.print(dataSize);
    Serial.print(", Received:");
    Serial.println(sizeReceived);
#endif
    return BMI160_E_COM_FAIL;
  }
  return BMI160_OK;
}

struct IMU
{
  bool Setup()
  {
    // BMI config!
    Sensor.id = BMI160_I2C_ADDR;
    Sensor.interface = BMI160_I2C_INTF;
    Sensor.read = IMURead;
    Sensor.write = IMUWrite;
    Sensor.delay_ms = IMUDelay;  

    int8_t res = bmi160_init(&Sensor);
    if(res != BMI160_OK)
    {
      Serial.println(res);
      Serial.println("Error while initializing the IMU");
      return false;
    }

    // Select the Output data rate, range of accelerometer sensor
    Sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
#if IMU_ACC_RANGE == 2
    Sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
#elif IMU_ACC_RANGE == 4
    Sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
#elif IMU_ACC_RANGE == 8
    Sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
#else
    #error Invalid IMU_ACC_RANGE!
#endif
    Sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    // Select the power mode of accelerometer sensor
    Sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Select the Output data rate, range of Gyroscope sensor
    Sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    Sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    // Select the power mode of Gyroscope sensor
    Sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    // Set the sensor configuration
    res = bmi160_set_sens_conf(&Sensor);
    if(res != BMI160_OK)
    {
      Serial.println(res);  
    }
        
    return true;
  }

  // Returns current acceleration values in m/s^2
  void GetAcceleration(float& x, float& y,float& z)
  {
    // Normalize [-32,768, +32,767], then scale by the range.
    struct bmi160_sensor_data accel;
    bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &Sensor);

    // Values used to tune the sensor!
    // x offset: 606
    // y offset: -677
    // z offset: 133
    accel.x -= 606;
    accel.y += 677;
    accel.z -= 133;
    
    // This could be reduced to 1 mult: rcp(32767) * range * g!
    x = ((float)accel.x / 32767.0f) * IMU_ACC_RANGE * STD_G;
    y = ((float)accel.y / 32767.0f) * IMU_ACC_RANGE * STD_G;
    z = ((float)accel.z / 32767.0f) * IMU_ACC_RANGE * STD_G;
  }

  void GetAngularRate(float& x, float& y, float& z)
  {
    struct bmi160_sensor_data gyro;
    bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &Sensor);

    x = ((float)gyro.x / 32767.0f) * IMU_GYRO_RANGE;
    y = ((float)gyro.y / 32767.0f) * IMU_GYRO_RANGE;
    z = ((float)gyro.z / 32767.0f) * IMU_GYRO_RANGE;
  }

  bmi160_dev Sensor;
}imu;

// ------------------------------ //
//        RECEIVER                //
// ------------------------------ // 
#define RC_NUM_CH 4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2   // Throttle
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

void RCCallback(uint8_t channel, uint8_t input_pin);

void RCCallbackCH1();
void RCCallbackCH2();
void RCCallbackCH3();
void RCCallbackCH4();

struct Receiver
{
  void Setup()
  {
    // Pin modes
    pinMode(RC_CH1_INPUT, INPUT);
    pinMode(RC_CH2_INPUT, INPUT);
    pinMode(RC_CH3_INPUT, INPUT);
    pinMode(RC_CH4_INPUT, INPUT);
    
    // Connect interrupt callback
    enableInterrupt(RC_CH1_INPUT, RCCallbackCH1, CHANGE);
    enableInterrupt(RC_CH2_INPUT, RCCallbackCH2, CHANGE);
    enableInterrupt(RC_CH3_INPUT, RCCallbackCH3, CHANGE);
    enableInterrupt(RC_CH4_INPUT, RCCallbackCH4, CHANGE);
  }

  // Copies the internal data to the user accesible data.
  void RetrieveValues()
  {
    noInterrupts();
    memcpy(DurationInternal, (const void *)Duration, sizeof(DurationInternal));
    interrupts();
  }
  
  uint16_t Duration[RC_NUM_CH];
  uint32_t Start[RC_NUM_CH];
  volatile uint16_t DurationInternal[RC_NUM_CH];
}receiver;

// ------------------------------ //
//        MOTORS                  //
// ------------------------------ //
struct Motors
{
  void Setup()
  {
    // Connected to digital pins
    ESC1.attach(7,1000,2000);
    ESC1.attach(8,1000,2000);
    ESC1.attach(9,1000,2000);
    ESC1.attach(10,1000,2000);
  }
  
  Servo ESC1;
  Servo ESC2;
  Servo ESC3;
  Servo ESC4;
}motors;


// ------------------------------ //
//        CODE IMPL.              //
// ------------------------------ //

// Callback used to retrieve the data from the receiver
void RCCallback(uint8_t channel, uint8_t input_pin) 
{
  if (digitalRead(input_pin) == HIGH) 
  {
    receiver.Start[channel] = micros();
  } 
  else 
  {
    receiver.DurationInternal[channel] = (uint16_t)(micros() - receiver.Start[channel]);
  }
}

// Redirects the callback with pin information.
void RCCallbackCH1() { RCCallback(RC_CH1, RC_CH1_INPUT); }
void RCCallbackCH2() { RCCallback(RC_CH2, RC_CH2_INPUT); }
void RCCallbackCH3() { RCCallback(RC_CH3, RC_CH3_INPUT); }
void RCCallbackCH4() { RCCallback(RC_CH4, RC_CH4_INPUT); }

void setup() 
{
  // Generic setup
  Serial.begin(9600);
  while (!Serial) {}
  Wire.begin();

  // RC receiver, first we set the pins as inputs, and then we setup the interrup callbacks
  receiver.Setup();

  // Setup the ESC
  motors.Setup();

  // Setup the BMI160
  imu.Setup();
}

void loop()
{
  // Get up to date values from the receiver:
  receiver.RetrieveValues();

  // Throttle value 0-100(%)
  int curThrottle = map(receiver.Duration[RC_CH1],1000,2000,0,100);
  // Serial.print("Throttle: "); Serial.println(curThrottle);

  motors.ESC1.write(curThrottle * 180);

  float ax,ay,az;
  imu.GetAcceleration(ax,ay,az);
  //Serial.println(ay);

  float pitch = -(atan2(ay, az) * 180.0f / PI);
  float roll = atan2(ax, az) * 180.0f / PI;
  //Serial.println(roll);
  
  //float wx,wy,wz;
  //imu.GetAngularRate(wx, wy, wz);
  //Serial.println(wx);

  Serial.print(roll);
  Serial.print("/");
  Serial.println(pitch);

}
