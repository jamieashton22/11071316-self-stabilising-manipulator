
#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
using namespace BLA;

// =============== MACROS and GLOBALS ===============================================================================================================================

// Servo A - 180 degree SER0064
#define SERVO_A_MIN 100 // pulse count for 0 deg
#define SERVO_A_MAX 550 // pulse count for 180 deg
// Servo B - 270 degree SER0046
#define SERVO_B_MIN 100 // pulse count for 0 deg
#define SERVO_B_MAX 600 // pulse count for 270 deg

// starting position joint angles
#define J0_START M_PI/2
#define J1_START -3*M_PI/4
#define J2_START M_PI/3
#define J3_START 0
#define J4_START 0

// starting ee position coords
#define START_X -6    
#define START_Y 0
#define START_Z 36

// link lengths
#define L1 12.5 //cm
#define L2 14.5
#define L3 14.5
#define L4 1.0 
#define L5 7.0

#define KP_POS 7.5f // position gain
#define KP_ORI_PITCH 3.0f
#define KP_ORI_ROLL 3.0f 
#define LAMBDA 0.05// for DLS pseudoinverse
#define DT 0.012 // loop timing

// comp filter alpha 
#define ALPHA_CF 0.96f

const float DELTA = 1e-4f; //step for jacobian computation

#define CONV_THRESHOLD 1.0f // convergence threshold (cm)
#define ORI_THRESHOLD (DEG_TO_RAD * 10) // level threshold (rad)  
#define MAX_ITER 200  // max iterations to converge before returning to start

// =============== IMU CLASS ===============================================================================================================================

// responsible for acquiring accel and gyro readings and carrying out sensor fusion
class IMU {
  public:
    Adafruit_MPU6050 mpu;
    float roll  = 0.0f;
    float pitch = 0.0f;

    bool begin() {  // method to check IMU working
      if (!mpu.begin()) return false;
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      return true;
    }

    // method to fuse accel and gyro readings via complementary filter
    void update(float dt) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float ax = a.acceleration.x;
      float ay = a.acceleration.y;
      float az = a.acceleration.z;
      float accel_roll  =  atan2f(ay, az);
      float accel_pitch =  atan2f(-ax, sqrtf(ay*ay + az*az));
      roll  = ALPHA_CF * (roll  + g.gyro.x * dt) + (1.0f - ALPHA_CF) * accel_roll;
      pitch = ALPHA_CF * (pitch + g.gyro.y * dt) + (1.0f - ALPHA_CF) * accel_pitch;
    }
};

// ============== ROBOT ARM CLASS =============================================================================================================
// responsible for all key methods relating to the manipulator including forward kinematics computation,
// Jacobian computation, DLS pseudoinverse, control step etc. 

class RobotArm {
  
  public:

    static const int DOF = 5;
    float q[DOF];

    // set true to output logging data
    bool logging = false;
    float lambda = LAMBDA;
    float dt = DT;

    // to hold IMU measurements from IMUHandler update method
    float imu_roll  = 0.0f;
    float imu_pitch = 0.0f;
    float desired_roll  = 0.0f;
    float desired_pitch = 0.0f;

    //DH angle in rad that corresponds to servo midpoints - offsets between DH model and physical
    const float q_home_rad[DOF] = {PI/2.0, -PI/2.0, 0.0f, 0.0f, 0.0f}; 
    const float servo_dir[DOF] = {1.0f,1.0f,1.0f,1.0f,1.0f}; // tweak if directions inverted

    bool target_reached = false;  // set true when both position and orientation converged
    bool pos_converged = false; // set true when just position converged
    const float pos_threshold = CONV_THRESHOLD;
    const float ori_threshold = ORI_THRESHOLD;

    Adafruit_PWMServoDriver ServoDriver;

    // starting target position
    float pd[3] = {START_X, START_Y, START_Z};

    // set initial joint values - for starting position
    RobotArm(Adafruit_PWMServoDriver _ServoDriver) {
        q[0] = J0_START;
        q[1] = J1_START;
        q[2] = J2_START;
        q[3] = J3_START;
        q[4] = J4_START;
        ServoDriver = _ServoDriver;
    }

    // member functions

    //start servo driver
    void init_servo_driver();
    // convert degrees to pulse count
    int angleToPulse(int angle, bool isB);
    // set single servo angle 
    void setServoAngle(uint8_t channel, int angle, bool isB = false);
    // convert dh angle in radians to servo degree command
    int dhRadtoDeg(float dh_rad, int joint);
    // function to write all servos
    void writeServos();
    // get Dh transform
    void dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H);
    // get current end effector position
    void forwardKinematics(float* q_in, float& x, float& y, float& z, float R[3][3]);
    // compute current jacobian position rows
    void computeJacobianPos(BLA::Matrix<3,3> & Jp);
    // compute current Jacobian orientation rows 
    void computeJacobianOri(BLA::Matrix<2,2>&Jo);
    // compute DLS inverse
    Matrix<2,2> DLSinvOri(BLA::Matrix<2,2>& J); 
    Matrix<3,3> DLSinvPos(BLA::Matrix<3,3>& J);
    // carry out control step 
    void controlStep();
    // forward kinematics using only first three joints - for decoupled position and orientation control
    void forwardKinematicsPos(float& x, float& y, float& z);
    // for serial input
    void handleSerial();
    // print end effector position and target
    void printStatus();

  private:

    bool isReachable(float x, float y, float z);  // set false if desired position not reachable 

      // DH parameters 
    const float a_dh[DOF]     = { 0.0f,  L2,          L3,         L4,         L5   };
    const float d_dh[DOF]     = { L1,    3.0f,         0.0f,       3.0f,       0.0f };
    const float alpha_dh[DOF] = { -M_PI/2.0f, 0.0f,   M_PI,   M_PI/2.0f,  0.0f };

      // joint limits
    const float Q_MIN[DOF] = {  0.0f,    -M_PI,    -M_PI/2.0f,  -3*M_PI/4.0f,  -3*M_PI/4.0f }; //ADD
    const float Q_MAX[DOF] = {  M_PI,     0.0f,     M_PI/2.0f,   3*M_PI/4.0f,   3*M_PI/4.0f }; //ADD

};

// ============== MEMBER FUNCTIONS ================================================================

void RobotArm::init_servo_driver() {
    ServoDriver.begin();
    ServoDriver.setPWMFreq(50);
}

void RobotArm::setServoAngle(uint8_t channel, int angle, bool is270) {
    ServoDriver.setPWM(channel, 0, angleToPulse(angle, is270));
}
// map pulse to angle
int RobotArm::angleToPulse(int angle, bool isB) {

    if (isB) {
        return map(angle, 0, 270, SERVO_B_MIN, SERVO_B_MAX);
    }
    return map(angle, 0, 180, SERVO_A_MIN, SERVO_A_MAX);
}

// Convert DH angle in radians to servo joint command 
int RobotArm::dhRadtoDeg(float dh_rad, int joint) {
    float delta_rad = dh_rad - q_home_rad[joint];
    float delta_deg = RAD_TO_DEG * delta_rad * servo_dir[joint];

    if (joint >= 3) {
        // servo B
        int servo_deg = (int)(135.0f + delta_deg);
        return constrain(servo_deg, 0, 270);
    } else {
        // servo A
        int servo_deg = (int)(90.0f + delta_deg);
        return constrain(servo_deg, 0, 180);
    }
}

void RobotArm::writeServos() {
    for (int i = 0; i < DOF; i++) {
        int deg    = dhRadtoDeg(q[i], i);
        bool isB = (i >= 3);
        setServoAngle(i, deg, isB);
    }
}

void RobotArm::dhTransform(float theta, float a, float d, float alpha,
                            BLA::Matrix<4,4>& H) {
    float ct = cos(theta), st = sin(theta);
    float ca = cos(alpha), sa = sin(alpha);
    H = { ct, -st*ca,  st*sa,  a*ct,
          st,  ct*ca, -ct*sa,  a*st,
         0.0f,    sa,     ca,     d,
         0.0f,  0.0f,   0.0f,  1.0f };
}

void RobotArm::forwardKinematics(float* q_in,
                                  float& x, float& y, float& z,
                                  float R[3][3]) {
    Matrix<4,4> H = { 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1 };
    Matrix<4,4> Hi;

    for (int i = 0; i < DOF; i++) {
        dhTransform(q_in[i], a_dh[i], d_dh[i], alpha_dh[i], Hi);
        H = H * Hi;
    }

    // extract positions
    x = H(0,3);
    y = H(1,3);
    z = H(2,3);

    // extract rotation matrix
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R[r][c] = H(r,c);
}

// Jacobians computed  numerically with forward difference, perturb each joint by small step delta then 
// recompute forward kinematics and divide change in pose over delta to approximate partial derivative 
// columns, essentially approximate of analytical jacobian

void RobotArm::computeJacobianPos(BLA::Matrix<3,3>& Jp) {
    // only position rows
    float x0, y0, z0;
    float R0[3][3];
    forwardKinematics(q, x0, y0, z0, R0);

    // only first 3 joints

    for (int i = 0; i < 3; i++) {  
        float q_pert[DOF];
        for (int k = 0; k < DOF; k++) q_pert[k] = q[k];
        q_pert[i] += DELTA;

        float xp, yp, zp;
        float Rp[3][3];
        forwardKinematics(q_pert, xp, yp, zp, Rp);

        Jp(0,i) = (xp - x0) / DELTA;  // dx/dqi
        Jp(1,i) = (yp - y0) / DELTA;  // dy/dqi
        Jp(2,i) = (zp - z0) / DELTA;  // dz/dqi
    }
}

void RobotArm::computeJacobianOri(BLA::Matrix<2,2>& Jo) {
    // only orientation rows
    float x0, y0, z0;
    float R0[3][3];
    forwardKinematics(q, x0, y0, z0, R0);

    for (int i = 0; i < 2; i++) {  // joints 3 and 4
        float q_pert[DOF];
        for (int k = 0; k < DOF; k++) q_pert[k] = q[k];
        q_pert[i + 3] += DELTA;  // offset to joints 3,4

        float xp, yp, zp;
        float Rp[3][3];
        forwardKinematics(q_pert, xp, yp, zp, Rp);

        float M[3][3] = {};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 3; k++)
                    M[r][c] += ((Rp[r][k] - R0[r][k]) / DELTA) * R0[c][k];

        Jo(0,i) = 0.5f * (M[2][1] - M[1][2]);  // roll
        Jo(1,i) = 0.5f * (M[0][2] - M[2][0]);  // pitch
    }
}

// forward kinematics using only first three joints 

void RobotArm::forwardKinematicsPos(float& x, float& y, float& z) {
    Matrix<4,4> H = { 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1 };
    Matrix<4,4> Hi;

    for (int i = 0; i < 3; i++) {  // first 3 joints only
        dhTransform(q[i], a_dh[i], d_dh[i], alpha_dh[i], Hi);
        H = H * Hi;
    }
    x = H(0,3);
    y = H(1,3);
    z = H(2,3);
}


Matrix<3,3> RobotArm::DLSinvPos(BLA::Matrix<3,3>& J) {
    Matrix<3,3> Jt  = ~J;
    Matrix<3,3> JJt = J * Jt;
    for (int i = 0; i < 3; i++) JJt(i,i) += lambda * lambda;
    return Jt * Inverse(JJt);
}

Matrix<2,2> RobotArm::DLSinvOri(BLA::Matrix<2,2>& J) {

    Matrix<2,2> Jt  = ~J;
    Matrix<2,2> JJt = J * Jt;

    for (int i = 0; i < 2; i++) JJt(i,i) += lambda * lambda;
    return Jt * Inverse(JJt);
}

bool RobotArm::isReachable(float x, float y, float z) {

    const float max_reach = L2 + L3 + L4 + L5;  // 37cm
    float z_relative = z - L1;
    float dist = sqrt(x*x + y*y + z_relative*z_relative);

    if (dist > max_reach) { // out of maximum reach
        return false;
    }
    if (z < 0.0f) { // below base
        return false;
    }
    return true;
}

void RobotArm::controlStep() {

    // counter for number of iterations
    static int iter = 0;

    // FK for position error using first 3 joints only
    float px3, py3, pz3;
    forwardKinematicsPos(px3, py3, pz3);

    // calculate position error
    float ex = pd[0] - px3;
    float ey = pd[1] - py3;
    float ez = pd[2] - pz3;
    float pos_err = sqrtf(ex*ex + ey*ey + ez*ez);

    // Full FK used for Jacobian
    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    // check if position converged
    if (pos_err < pos_threshold) {
        if (!pos_converged) {
            Serial.println("Position converged");
            pos_converged = true;
        }
    }

    // calculate orientation error 
    float eo_x = desired_roll  - imu_roll;
    float eo_y = desired_pitch - imu_pitch;
    float ori_err = sqrtf(eo_x*eo_x + eo_y*eo_y);

    // if logging is eabled print time, orientation error, roll, pitch, target position, actual position
    if (logging) {
        Serial.print(millis());                      Serial.print(",");
        Serial.print(pos_err, 3);                    Serial.print(",");
        Serial.print(RAD_TO_DEG * ori_err, 3);       Serial.print(",");
        Serial.print(RAD_TO_DEG * imu_roll,  2);     Serial.print(",");
        Serial.print(RAD_TO_DEG * imu_pitch, 2);     Serial.print(",");
        Serial.print(pd[0], 2); Serial.print(",");
        Serial.print(pd[1], 2); Serial.print(",");
        Serial.print(pd[2], 2); Serial.print(",");
        Serial.print(px3, 2);    Serial.print(",");
        Serial.print(py3, 2);    Serial.print(",");
        Serial.println(pz3, 2);
    }

    // Position update step if position not converged
    if (!pos_converged) {

      // calculate DLS pseudoinverse
        Matrix<3,3> Jp;
        computeJacobianPos(Jp);
        Matrix<3,3> Jpinv = DLSinvPos(Jp);

        // proportional control 
        Matrix<3,1> pos_error_vec;
        pos_error_vec(0) = KP_POS * ex;
        pos_error_vec(1) = KP_POS * ey;
        pos_error_vec(2) = KP_POS * ez;

        // calculate change in joint angle for position
        Matrix<3,1> qdot_pos = Jpinv * pos_error_vec;

        // integrate angle change, update new joint angle, constrain to joint limits
        for (int i = 0; i < 3; i++)
            q[i] = constrain(q[i] + qdot_pos(i) * dt, Q_MIN[i], Q_MAX[i]);
    }

    // Orientation update always runs, no seperate check for orientation convergence

    // compute DLS pseudoinv for orientation 
    Matrix<2,2> Jo;
    computeJacobianOri(Jo);
    Matrix<2,2> Joinv = DLSinvOri(Jo);

    // orientation error vector
    Matrix<2,1> ori_error_vec;
    // apply proportional control, axis of IMU dont align with world axis so flip 
    ori_error_vec(0) = -KP_ORI_PITCH * eo_y;
    ori_error_vec(1) =  KP_ORI_ROLL  * eo_x;

    // change in joint angle for orientation correction
    Matrix<2,1> qdot_ori = Joinv * ori_error_vec;
    // integrate, update new joint angle for orientation correction, constrain to joint limits
    for (int i = 0; i < 2; i++)
        q[i+3] = constrain(q[i+3] + qdot_ori(i) * dt, Q_MIN[i+3], Q_MAX[i+3]);

    // position convergence and level orientation check
    if (pos_converged && ori_err < ori_threshold) {
        if (!target_reached) {
            Serial.println("Target reached");
            target_reached = true;
            iter = 0;
        }
        return;
    }

    // after attempting for MAX_ITER give up, return to start 
    if (iter > MAX_ITER) {
        Serial.println("Failed to reach target - returning to start");
        target_reached = true;
        pos_converged = false;  // reset for next target
        iter = 0;
        q[0] = J0_START; q[1] = J1_START; q[2] = J2_START;
        q[3] = J3_START; q[4] = J4_START;
        writeServos();
        return;
    }

    iter++;
}

void RobotArm::handleSerial() {

    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // set target position to home 
    if (line == "H" || line == "h") {
        pd[0] = START_X; pd[1] = START_Y; pd[2] = START_Z;
        target_reached = false;
        Serial.println("Returning to start position");
        return;
    }

    // directly write joint angles to safe starting pose 
    if (line == "S" || line == "s") {
      q[0] = J0_START; q[1] = J1_START; q[2] = J2_START;
      q[3] = J3_START; q[4] = J4_START;
      target_reached = true;
      Serial.println("Safe pose");
      return;
  }

  // print status
    if (line == "P" || line == "p") {
        printStatus();
        return;
    }

    // enable/disable logging
    if (line == "L" || line == "l") {
        logging = !logging;
        return;
    }

    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);
    if (s1 == -1 || s2 == -1) { Serial.println("Invalid"); return; }

    float nx = line.substring(0, s1).toFloat();
    float ny = line.substring(s1 + 1, s2).toFloat();
    float nz = line.substring(s2 + 1).toFloat();

    Serial.print("Target: ["); Serial.print(nx,2); Serial.print(", ");
    Serial.print(ny,2); Serial.print(", "); Serial.print(nz,2); Serial.println("]");

    if (isReachable(nx, ny, nz)) {
        pd[0] = nx; pd[1] = ny; pd[2] = nz;
        target_reached = false;
        pos_converged = false;
        Serial.println("Target accepted");
    } else {
        pd[0] = nx; pd[1] = ny; pd[2] = nz;
        Serial.println("Target rejected");
        target_reached = true;
    }
}

void RobotArm::printStatus() {
    float px3, py3, pz3;
    forwardKinematicsPos(px3, py3, pz3);

    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    Serial.print("Actual position: ");
    Serial.print(px3,2);
     Serial.print(", ");
    Serial.print(py3,2);
     Serial.print(", ");
    Serial.print(pz3,2);

    Serial.print("Target position: ");
    Serial.print(pd[0],2); 
    Serial.print(", ");
    Serial.print(pd[1],2);
     Serial.print(", ");
    Serial.print(pd[2],2);

    // calculate current pos error
    float pos_err = sqrtf(pow(pd[0]-px3,2)+pow(pd[1]-py3,2)+pow(pd[2]-pz3,2));
    Serial.print("Pos err: "); Serial.println(pos_err, 3);

    Serial.print("IMU roll:  "); Serial.print(RAD_TO_DEG * imu_roll,  2);
    Serial.print("IMU pitch: "); Serial.print(RAD_TO_DEG * imu_pitch, 2);
}

// ============== OBJECTS ================================================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm  arm(ServoDriver);
IMU imu;

// ============== SETUP ==================================================================

void setup() {

    Serial.begin(115200);
    arm.init_servo_driver();

    if (!imu.begin()) {
    Serial.println("ERROR: MPU6050 not found");
    while (1) delay(10);
    } 
    Serial.println("MPU6050 ready");

    Serial.println("===== Beginning Arm Test =====");
    Serial.println("Going to start position...");
    delay(1000);
    arm.writeServos();

    // // UNCOMMENT FOR ASSEMBLY
    // // Joints 0-2: 180° servos, midpoint = 90°
    // arm.setServoAngle(0, 90, false);
    // arm.setServoAngle(1, 90, false);
    // arm.setServoAngle(2, 90, false);
    // arm.setServoAngle(3, 135, true);
    // arm.setServoAngle(4, 135, true);

    // while(true);
  
}

// ============== LOOP ===================================================================

void loop() {

    // get serial input
    arm.handleSerial();

    // get time 
    static unsigned long last_t = 0;
    unsigned long now = millis();
    last_t = now;

    // get pitch and rol from imu
    imu.update(DT);
    arm.imu_roll  = imu.roll; // copy for use in arm class
    arm.imu_pitch = imu.pitch;

    // if target not reached carry out control step and write the servos 
    if (!arm.target_reached) {
        arm.controlStep();
        arm.writeServos();
    }

}