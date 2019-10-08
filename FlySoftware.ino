#include <Servo.h>
#include <MPU9250.h>
#include <Wire.h>

#define STD_G   9.80665f  
#define RC_PIN  2

#define RC_CH1  0   // Roll
#define RC_CH2  1   // Pitch
#define RC_CH3  2   // Throttle
#define RC_CH4  3   // Yaw

// ------------------------------ //
//            IMU                 //
// ------------------------------ // 

MPU9250 IMUSensor(Wire,104);

struct IMU
{
  bool Setup()
  {
    m_acumGyroPitch = 0.0f;
    m_acumYaw= 0.0f;
    m_acumGyroRoll= 0.0f;
    
    IMUSensor.begin();
    IMUSensor.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    IMUSensor.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        
    return true;
  }

  // Returns current acceleration values in m/s^2
  void GetAcceleration(float& x, float& y, float& z)
  {
    x = IMUSensor.getAccelX_mss();
    y = IMUSensor.getAccelY_mss();
    z = IMUSensor.getAccelZ_mss();
  }

  // Returns angular rage in degrees per second:
  void GetAngularRate(float& x,float& y, float& z)
  {
    x = IMUSensor.getGyroX_rads() * RAD_TO_DEG;
    y = IMUSensor.getGyroY_rads() * RAD_TO_DEG;
    z = IMUSensor.getGyroZ_rads() * RAD_TO_DEG;
  }

  // Returns the magnetic field in micro teslas
  void GetMagneticField(float& x, float& y, float& z)
  {
    x = IMUSensor.getMagX_uT();
    y = IMUSensor.getMagY_uT();
    z = IMUSensor.getMagZ_uT();
  }
  
  void GetCurrentOrientation(float deltaSeconds, float& pitch, float& yaw, float& roll)
  {
    static bool firstIteration = true;
    
    // Retrieve the values!
    IMUSensor.readSensor();
    
    // Get current acceleration values:
    float ax, ay, az;
    GetAcceleration(ax,ay,az);

    // Initial noisy pitch and roll computed from acceleration, this is bad, as it suffers from gimbal lock :(
    // ...it is a right handed coordinate system with the z-axis positive down, common in aircraft dynamics.
    float accMagnitude = sqrt((ax*ax) + (ay*ay) + (az*az));
    float noisyPitch =   (atan2((ax / accMagnitude) , (-az / accMagnitude)) * RAD_TO_DEG);
    float noisyRoll =  (atan2((-ay / accMagnitude) , (-az / accMagnitude)) * RAD_TO_DEG);

    // Now that I look at this, I guess we could compute Yaw as well... LOL
    noisyPitch = constrain(noisyPitch, -80.0f, 80.0f);
    noisyRoll = constrain(noisyRoll, -80.0f, 80.0f);
     
    // Get gyroscope data:
    float wx, wy, wz;
    GetAngularRate(wx, wy, wz);

    // Get magnetometer data:
    float mx, my, mz;
    GetMagneticField(mx, my, mz);
    float noisyYaw = atan2(my, mx) * RAD_TO_DEG;
    
    // Angular rate to orientation:
    if(firstIteration)
    {
       m_acumGyroPitch = noisyPitch;
       m_acumGyroRoll = noisyRoll;
       firstIteration = false;
    }
    else
    {
      m_acumGyroPitch += wy * deltaSeconds;
      m_acumYaw += wz * deltaSeconds;
      m_acumGyroRoll += wx * deltaSeconds;
    }
    
    // Transfer angle as we have yawed:
    m_acumGyroPitch -= m_acumGyroRoll * sin((wz * deltaSeconds) * PI / 180.0f);
    m_acumGyroRoll += m_acumGyroPitch * sin((wz * deltaSeconds) * PI / 180.0f);

    // Compute the final orientation values (combine gyro and acc data):
    m_acumGyroPitch = m_acumGyroPitch * 0.996f + noisyPitch * 0.004f;
    m_acumGyroRoll = m_acumGyroRoll * 0.996f + noisyRoll * 0.004f;

    // Copy out the values!
    pitch = m_acumGyroPitch;
    yaw = m_acumYaw;
    roll = m_acumGyroRoll;

#if 0
    Serial.print(pitch);Serial.print("/");
    Serial.print(yaw); Serial.print("/");
    Serial.println(roll);
#endif

#if 0
    Serial.print(noisyPitch);Serial.print(" / ");
    Serial.print(noisyYaw); Serial.print(" / ");
    Serial.println(noisyRoll);
#endif
  }

private:
  float m_acumGyroPitch;
  float m_acumYaw;
  float m_acumGyroRoll;
}imu;

// ------------------------------ //
//        RECEIVER                //
// ------------------------------ // 

static void RCCallback();

struct Receiver
{
  Receiver():
    mCurChannel(0),
    mPulseStart(0)
  {
    memset(mChannelsInternal, 0, sizeof(mChannelsInternal));
    memset(mChannels, 0, sizeof(mChannels));
  }

  void CallbackImpl()
  { 
    if(digitalRead(RC_PIN) == HIGH)
    {
      mPulseStart = micros();
    }
    else
    {
      unsigned long elapsed = micros() - mPulseStart;
      if(elapsed > 7000) // Just finished padding pulse, get ready for channels.
      {
        mCurChannel = 0;
      }
      else
      {
        mChannelsInternal[mCurChannel++] = (float)elapsed / 1000.0f;    
      }
    }
  }

  void Setup()
  {
    // Pin modes
    pinMode(RC_PIN, INPUT);

    // Connect interrupt callback
    attachInterrupt(
      digitalPinToInterrupt(RC_PIN), 
      RCCallback, CHANGE
    );
  }

  // Copies the internal data to the user accesible data.
  void RetrieveValues()
  {
    noInterrupts();
    memcpy(mChannels, mChannelsInternal, sizeof(mChannelsInternal));
    interrupts();
  }

  float GetChannel(uint8_t chnum)
  {
    if(chnum > 5) 
    {
      return 0.0f;
    }
    float curMs = (mChannels[chnum] - 0.6f) * 0.625f; // [0,1]
    curMs = curMs * 2.0f - 1.0f; // [-1,1]
    return constrain(curMs, -1.0f, 1.0f);
  }
  
private:
  uint8_t mCurChannel;              // Channel being processed
  volatile float mChannelsInternal[6]; // Internal data, pulse in ms for each channel
  unsigned long mPulseStart;        // Start of current pulse
  float mChannels[6];               // This is the data that the user reads (-1 to 1)

}receiver;

static void RCCallback()
{
  receiver.CallbackImpl();
}

// ------------------------------ //
//        QUAD & PID              //
// ------------------------------ //
struct PIDController
{
  PIDController(float pGain, float iGain, float dGain):
    mPitchIntegral(0.0f),
    mPreviousError(0.0f)
  {
    SetGains(pGain, iGain, dGain);
  }

  void SetGains(float pGain, float iGain, float dGain)
  {
    mPGain = pGain;
    mIGain = iGain;
    mDGain = dGain;
  }
  
  float Update(float error, float deltaSeconds)
  {
    float p = error * mPGain;
    mPitchIntegral += error * deltaSeconds;
    float i = mPitchIntegral * mIGain;
    float d = ((error - mPreviousError) / deltaSeconds) * mDGain;
    mPreviousError = error;
    return p + i + d;
  }
  
 private:
  float mPGain;
  float mIGain;
  float mDGain;

  float mPitchIntegral;
  float mPreviousError;
};


#define P_GAIN 0.0f     // 0.45f
#define I_GAIN 0.0f    // 0.065f
#define D_GAIN 0.08f    // 0.075f

struct Quad
{
  Quad():
    PitchPID(P_GAIN, I_GAIN, D_GAIN),
    YawPID(0.45f, 0.065f, 0.0f),
    RollPID(P_GAIN, I_GAIN, D_GAIN)
  {
    Pitch = 0.0f;
    Yaw = 0.0f;
    Roll = 0.0f;
  }
  void Setup()
  {
    // Connected to digital pins
    ESC1.attach(6,1000,2000);
    ESC2.attach(9,1000,2000);
    ESC3.attach(10,1000,2000);
    ESC4.attach(11,1000,2000);    

    SetThrottle(0.0f,0.0f,0.0f,0.0f);
  }

  // Input values in the 0-1000 range:
  void SetThrottle(float tESC1, float tESC2, float tESC3, float tESC4)
  {
    tESC1 = constrain(tESC1, 0.0f, 1000.0f);
    tESC2 = constrain(tESC2, 0.0f, 1000.0f);
    tESC3 = constrain(tESC3, 0.0f, 1000.0f);
    tESC4 = constrain(tESC4, 0.0f, 1000.0f);
  
    ESC1.write((tESC1 / 1000.0f) * 180.0f);
    ESC2.write((tESC2 / 1000.0f) * 180.0f);
    ESC3.write((tESC3 / 1000.0f) * 180.0f);
    ESC4.write((tESC4 / 1000.0f) * 180.0f);
  }
  
  float Pitch;
  float Yaw;
  float Roll;

  PIDController PitchPID;
  PIDController YawPID;
  PIDController RollPID;
  
  Servo ESC1;
  Servo ESC2;
  Servo ESC3;
  Servo ESC4;
}quad;


// ------------------------------ //
//        CODE IMPL.              //
// ------------------------------ //

void setup() 
{
  // Generic setup
  Serial.begin(9600);
  
  Wire.begin();
  Wire.setClock(400000); // 400 khz test this!
  
  // RC receiver, first we set the pins as inputs, and then we setup the interrup callbacks
  receiver.Setup();

  // Setup the quad (motors)
  quad.Setup();

  // Setup the MPU9250
  imu.Setup();
}

// Timing.
float acumTimeSeconds = 0.0f;
unsigned long lastMicro = 0;
float targetYaw = 0.0f;

void loop()
{
  // Update delta time and acumulated time:
  unsigned long curMicro = micros(); // This will reset after 70 minutes...
  float deltaSeconds = (float)(curMicro - lastMicro) / 1000000.0f;
  lastMicro = curMicro;
  acumTimeSeconds += deltaSeconds;

  // Start by processing the current orientation:
  imu.GetCurrentOrientation(deltaSeconds, quad.Pitch, quad.Yaw, quad.Roll);

  // Get up to date values from the receiver:
  receiver.RetrieveValues();

  float rxRoll = receiver.GetChannel(0);
  float rxPitch = receiver.GetChannel(1);
  float rxYaw = receiver.GetChannel(3);
  // Throttle from receiver, we clamp it to 960 to give the PID some headroom:
  float rxThrottle = (receiver.GetChannel(2) * 0.5f + 0.5f) * 960.0f;
  
#if 0
  Serial.print("Throttle:");Serial.print(rxThrottle);
  Serial.print(" Pitch:");Serial.print(rxPitch);
  Serial.print(" Yaw:");Serial.print(rxYaw);
  Serial.print(" Roll:");Serial.println(rxRoll);
#endif

  float throttleESC1 = rxThrottle;
  float throttleESC2 = rxThrottle;
  float throttleESC3 = rxThrottle;
  float throttleESC4 = rxThrottle;
  
  if(rxThrottle > 30) // Add some dead band.
  {    
    float targetPitch = 20.0f * -rxPitch; 
    float pitchError = quad.Pitch - targetPitch; 
    float pidPitch = quad.PitchPID.Update(pitchError, deltaSeconds);

    if(!(rxYaw > -0.15f && rxYaw < 0.15f)) // Some dead band so we dont yaw all the time.
    {
      targetYaw += rxYaw * (25.0f * deltaSeconds); // º/second
    }
    float yawError = quad.Yaw - targetYaw;
    float pidYaw = quad.YawPID.Update(yawError, deltaSeconds);
    pidYaw = 0.0f; // Disable it for now
    
    float targetRoll = 20.0f * rxRoll;
    float rollError = quad.Roll - targetRoll;
    float pidRoll = quad.RollPID.Update(rollError, deltaSeconds);
    
    throttleESC1 += -pidPitch - pidRoll - pidYaw;
    throttleESC2 += -pidPitch + pidRoll + pidYaw;   
    throttleESC3 += pidPitch + pidRoll - pidYaw;    
    throttleESC4 += pidPitch - pidRoll + pidYaw;    

#if 0
  Serial.print(throttleESC1); Serial.print(", "); Serial.print(throttleESC2); Serial.print(", "); Serial.print(throttleESC3); Serial.print(", "); Serial.println(throttleESC4);
  //Serial.print(targetPitch); Serial.print(", "); Serial.print(targetYaw);Serial.print(", "); Serial.println(targetRoll);
#endif
  }
  else
  {
    targetYaw = quad.Yaw; // Just reset it... 
    
    throttleESC1 = 0.0f;
    throttleESC2 = 0.0f;
    throttleESC3 = 0.0f;
    throttleESC4 = 0.0f;
  }

  // Send the throttle to the ESC.
  quad.SetThrottle(throttleESC1, throttleESC2, throttleESC3, throttleESC4);  
}
