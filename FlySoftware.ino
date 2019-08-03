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
#define IMU_ACC_RANGE 4      // 2, 4, 8                   (+/-g)
#define IMU_GYRO_RANGE 1000  // 2000, 1000, 500, 250, 125 (º/s)
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
    Wire.write(reg + r); // This is not needed!  
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
#if IMU_GYRO_RANGE == 2000
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
#elif IMU_GYRO_RANGE == 1000
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
#elif IMU_GYRO_RANGE == 500
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
#elif IMU_GYRO_RANGE == 250
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
#elif IMU_GYRO_RANGE == 125
    Sensor.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
#else
    #error Invalid IMU_GYRO_RANGE!
#endif
    Sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE; ///mmmmm meh

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

  void GetRawAcceleration(int& x, int& y, int& z)
  {
    struct bmi160_sensor_data accel;
    bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &Sensor);
    x = accel.x;
    y = accel.y;
    z = accel.z;
  }

  // Returns current acceleration values in m/s^2
  void GetAcceleration(float& x, float& y,float& z)
  {
    int rx, ry, rz;
    GetRawAcceleration(rx, ry, rz);

    // Values used to tune the sensor!
    rx += m_calAccX;
    ry += m_calAccY;
    rz += m_calAccZ;
    
    // This could be reduced to 1 mult: rcp(32767) * range * g!
    x = ((float)rx / 32767.0f) * IMU_ACC_RANGE * STD_G;
    y = ((float)ry / 32767.0f) * IMU_ACC_RANGE * STD_G;
    z = ((float)rz / 32767.0f) * IMU_ACC_RANGE * STD_G;
  }

  void CalibrateAccelerometer()
  {
    // Serial.println("Calibrating accelerometer...");
    const int calibrationSteps = 1000;
    int acumX = 0;
    int acumY = 0;
    int acumZ = 0;
    int rawX, rawY, rawZ;
    for(int i=0; i<calibrationSteps; ++i)
    {
      GetRawAcceleration(rawX,rawY,rawZ);
      acumX += rawX;
      acumY += rawY;
      acumZ += rawZ;
      delay(1);
    }
    m_calAccX = acumX / calibrationSteps;
    m_calAccY = acumY / calibrationSteps;
    m_calAccZ = acumZ / calibrationSteps;
    // Serial.print("Done!: "); Serial.print(m_calAccX);Serial.print(",");Serial.print(m_calAccY);Serial.print(",");Serial.println(m_calAccZ);
  }
  
  void GetRawAngularRate(int& x,int& y, int& z)
  {
    struct bmi160_sensor_data gyro;
    bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &Sensor);
    
    x = gyro.x;
    y = gyro.y;
    z = gyro.z;
  }
  
  void GetAngularRate(float& x, float& y, float& z)
  {
    int rx,ry,rz;
    GetRawAngularRate(rx, ry, rz);

    // Apply calibration values:
    rx += m_calGyroX;
    ry += m_calGyroY;
    rz += m_calGyroZ;
    
    x = ((float)rx / 32767.0f) * IMU_GYRO_RANGE;
    y = ((float)ry / 32767.0f) * IMU_GYRO_RANGE;
    z = ((float)rz / 32767.0f) * IMU_GYRO_RANGE;
  }

  void CalibrateGyroscope()
  {
    // Serial.println("Calibrating gyro...");
    const int calibrationSteps = 1000;
    int acumX = 0;
    int acumY = 0;
    int acumZ = 0;
    int rawX, rawY, rawZ;
    for(int i=0; i<calibrationSteps; ++i)
    {
      GetRawAngularRate(rawX,rawY,rawZ);
      acumX += rawX;
      acumY += rawY;
      acumZ += rawZ;
      delay(1);
    }
    m_calGyroX = acumX / calibrationSteps;
    m_calGyroY = acumY / calibrationSteps;
    m_calGyroZ = acumZ / calibrationSteps;
    // Serial.print("Done!: "); Serial.print(m_calGyroX);Serial.print(",");Serial.print(m_calGyroY);Serial.print(",");Serial.println(m_calGyroZ);
  }

  void GetCurrentOrientation(float deltaSeconds, float& pitch, float& yaw, float& roll)
  {
    // Get current acceleration values:
    float ax, ay, az;
    GetAcceleration(ax,ay,az);

    // Initial noisy pitch and roll computed from acceleration, this is bad, as it suffers from gimbal lock :(
    float accMagnitude = sqrt((ax*ax) + (ay*ay) + (az*az));
    float noisyPitch = -asin(ay / accMagnitude) * 57.296;
    float noisyRoll = asin(ax / accMagnitude) * 57.296;
 
    // Get gyroscope data:
    float wx, wy, wz;
    GetAngularRate(wx, wy, wz);

    // Angular rate to orientation:
    m_acumGyroX -= wx * deltaSeconds;
    m_acumGyroY -= wy * deltaSeconds;

    // Transfer angle as we have yawed.. TO-DO: understand this better
    m_acumGyroX += m_acumGyroY * sin((wz * deltaSeconds) * PI / 180.0f);
    m_acumGyroY -= m_acumGyroX * sin((wz * deltaSeconds) * PI / 180.0f);

    // Compute the final orientation values (combine gyro and acc data):
    m_acumGyroX = m_acumGyroX * 0.98f + noisyPitch * 0.02f;
    m_acumGyroY = m_acumGyroY * 0.98f + noisyRoll * 0.02f;

    // Copy out the values!
    pitch = m_acumGyroX;
    yaw -= wz * deltaSeconds;
    roll = m_acumGyroY;
  }

  bmi160_dev Sensor;
  
private:
  int m_calGyroX;
  int m_calGyroY;
  int m_calGyroZ;
  
  int m_calAccX;
  int m_calAccY;
  int m_calAccZ;

  float m_acumGyroX;
  float m_acumGyroY;
}imu;

// ------------------------------ //
//        RECEIVER                //
// ------------------------------ // 
#define RC_NUM_CH 4

#define RC_CH1  0   // Roll
#define RC_CH2  1   // Pitch
#define RC_CH3  2   // Throttle
#define RC_CH4  3   // Yaw

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
    memcpy(Duration,DurationInternal, sizeof(DurationInternal));
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
    ESC1.attach(6,1000,2000);
    ESC2.attach(9,1000,2000);
    ESC3.attach(10,1000,2000);
    ESC4.attach(11,1000,2000);
  }
  
  Servo ESC1;
  Servo ESC2;
  Servo ESC3;
  Servo ESC4;
}motors;

// ------------------------------ //
//        QUAD INFO               //
// ------------------------------ //
struct Quad
{
  float Pitch;
  float Yaw;
  float Roll;
}quad;


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
  //while (!Serial) {}
  
  Wire.begin();

  // RC receiver, first we set the pins as inputs, and then we setup the interrup callbacks
  receiver.Setup();

  // Setup the ESC
  motors.Setup();

  // Setup the BMI160
  imu.Setup();
  imu.CalibrateGyroscope();
  imu.CalibrateAccelerometer();
}

// Timing.
float acumTimeSeconds = 0.0f;
unsigned long lastMicro = 0;

void loop()
{
  // Update delta time and acumulated time:
  unsigned long curMicro = micros(); // This will reset after 70 minutes...
  float deltaSeconds = (float)(curMicro - lastMicro) / 1000000.0f;
  lastMicro = curMicro;
  acumTimeSeconds += deltaSeconds;

  // Start by processing the current orientation:
  imu.GetCurrentOrientation(deltaSeconds, quad.Pitch, quad.Yaw, quad.Roll);
  quad.Pitch = -quad.Pitch; // Test stand is upside down.
  
  // Get up to date values from the receiver:
  receiver.RetrieveValues();

  // Throttle value 0-1
  float curThrottle = (float)receiver.Duration[RC_CH3]; // [1000-2000]
  curThrottle = curThrottle - 1000.0f;                  // [0 - 1000]
  curThrottle /= 1000.0f;                               // [0 - 1]
  curThrottle = constrain(curThrottle, 0.0f, 1.0f);

  float throttleESC1 = curThrottle;
  float throttleESC2 = curThrottle;
  
  // PID!
  if(curThrottle > 0.1f) // safe!
  {
    float targetPitch = 0.0f; // This is the value we want
    float pitchError = quad.Pitch - targetPitch; // Current error.

    // [P]
    float P = pitchError * 0.0001f;
    
    // [I]
    float I = 0.0f;
    
    // [D]
    float D = 0.0f;

    throttleESC2 -= P + I + D; // D9
    throttleESC1 += P + I + D; // D6

    // Clamp the values!
    throttleESC1 = constrain(throttleESC1, 0.0f, 1.0f);
    throttleESC2 = constrain(throttleESC2, 0.0f, 1.0f);
    Serial.println(throttleESC1);
  }
  
  // Send the throttle to the ESC.
  motors.ESC1.write(throttleESC1 * 180.0f);
  motors.ESC2.write(throttleESC2 * 180.0f);
  
#if 0
  Serial.println(quad.Pitch );
  Serial.print(quad.Yaw); Serial.print("/");
  Serial.println(quad.Roll);
#endif
}
