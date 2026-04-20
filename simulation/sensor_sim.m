
%% IMU and sensor-fusion sim

%% Parameters =============================================================

dt = 0.01; % sample period
T = 5000;
t = (0:T)' * dt;

%% Ground truth orientation sinusoidal motion===============================================

roll_true  = deg2rad(60) * sin(0.3 * t);
pitch_true = deg2rad(45) * sin(0.5 * t + pi/3);
yaw_true   = zeros(T, 1);

roll_rate  = deg2rad(60) * 0.3 * cos(0.3 * t);
pitch_rate = deg2rad(45) * 0.5 * cos(0.5 * t + pi/3);
yaw_rate   = zeros(T, 1);

%% MPU6050 sensor parameters from datasheet ================================

gyro_noise_density = deg2rad(0.5);
acc_noise_density = 800e-6 * 9.81;

gyro_constant_bias  = deg2rad([0.5; -0.4; 0.2]);  
acc_constant_bias   = [0.025; -0.02; 0.05];         

% random walk from Rudyk 
gyro_bias_rw_sigma  = deg2rad(0.002449);

sample_rate = 1 / dt;

% convert noise densities
gyro_noise  = gyro_noise_density * sqrt(sample_rate);
acc_noise   = acc_noise_density  * sqrt(sample_rate);

g_vec = [0; 0; 9.81]; % grav. vector

roll_gyro_pure = 0;
pitch_gyro_pure = 0;

%% Complementary Filter ========================================================================

comp_alpha = 0.97;

roll_cf     = 0;
pitch_cf    = 0;


%% Kalman (imufilter) Setup ========================================================================
Kalman = imufilter('SampleRate', sample_rate);
Kalman.AccelerometerNoise  = acc_noise_density^2  * sample_rate;  
Kalman.GyroscopeNoise      = gyro_noise_density^2 * sample_rate;  
Kalman.GyroscopeDriftNoise = gyro_bias_rw_sigma^2;                


%% Madgwick filter ========================================================================

madgw_beta = 4;
madgwick  = Madgwick('UpdateFrequency', sample_rate, 'Beta', madgw_beta,'Zeta', 0.02);


%% Data Storage ========================================================================

data = struct();

data.time       = t;
data.true_roll  = roll_true;
data.true_pitch = pitch_true;

data.cf_roll    = zeros(T,1);
data.cf_pitch   = zeros(T,1);

data.kf_roll    = zeros(T,1);
data.kf_pitch   = zeros(T,1);

data.mf_roll    = zeros(T,1);
data.mf_pitch   = zeros(T,1);

data.gyro_raw_roll  = zeros(T,1);
data.gyro_raw_pitch = zeros(T,1);
data.acc_raw_roll   = zeros(T,1);
data.acc_raw_pitch  = zeros(T,1);

% Execution time per step [s]
data.time_cf = zeros(T,1);
data.time_kf = zeros(T,1);
data.time_mf = zeros(T,1);

%% MAIN LOOP ========================================================================

for k = 1:T

    % simulate IMU measurements 
    phi   = roll_true(k);   % roll
    theta = pitch_true(k);  % pitch
    psi   = yaw_true(k);    % yaw

    % rotation matrix in IMU frame
    Re = eul2rotm([psi, theta, phi], 'ZYX');
    W = [1,    0,              -sin(theta);
         0,    cos(phi),   sin(phi)*cos(theta);
         0,   -sin(phi),   cos(phi)*cos(theta)];

    % gyro reading in body frame
    omega_body = W * [roll_rate(k); pitch_rate(k); yaw_rate(k)];

    % accel reading, grav. vector in body frame 
    acc_true = Re' * g_vec;

    % simulate noisy gyro measurements
    gyro_bias = gyro_constant_bias + gyro_bias_rw_sigma * sqrt(dt) * randn(3,1);
    gyro_meas = (omega_body + gyro_bias + gyro_noise * randn(3,1))';  

    % simulate noisy acc measurements
    acc_meas = (acc_true + acc_constant_bias + acc_noise * randn(3,1))';  % row vector

    % raw accelerometer orientation
    roll_acc  = atan2(acc_meas(2), acc_meas(3));
    pitch_acc = atan2(-acc_meas(1), sqrt(acc_meas(2)^2 + acc_meas(3)^2));
    data.acc_raw_roll(k)  = roll_acc;
    data.acc_raw_pitch(k) = pitch_acc;

    % raw gyro orientation
    euler_rates_gyro = inv(W)*gyro_meas';  
    roll_gyro_pure   = roll_gyro_pure  + euler_rates_gyro(1) * dt;
    pitch_gyro_pure  = pitch_gyro_pure + euler_rates_gyro(2) * dt;
    data.gyro_raw_roll(k)  = roll_gyro_pure;
    data.gyro_raw_pitch(k) = pitch_gyro_pure;

    % complementary filter 
    tic;
    roll_gyro_cf  = roll_cf  + gyro_meas(1) * dt;
    pitch_gyro_cf = pitch_cf + gyro_meas(2) * dt;

    roll_cf   = comp_alpha * roll_gyro_cf  + (1 - comp_alpha) * roll_acc;
    pitch_cf  = comp_alpha * pitch_gyro_cf + (1 - comp_alpha) * pitch_acc;

    data.cf_roll(k)  = roll_cf;
    data.cf_pitch(k) = pitch_cf;
    data.time_cf(k)  = toc;

    % Kalman filter (imufilter)
    tic;
    orientation_q = Kalman(acc_meas, gyro_meas);
    orientation_e = quat2eul(orientation_q, 'ZYX');
    data.kf_roll(k)  = orientation_e(3);
    data.kf_pitch(k) = orientation_e(2);
    data.time_kf(k)  = toc;

    % Madgwick filter no magnetometer
    tic;
    madgwick.update(gyro_meas, acc_meas,NaN);   % accel+gyro only
    mw_eul = quat2eul(madgwick.qEst, 'ZYX');
    data.mf_roll(k)  = mw_eul(3);
    data.mf_pitch(k) = mw_eul(2);
    data.time_mf(k)  = toc;

end

% save data

save("sensor_results.mat", 'data');
