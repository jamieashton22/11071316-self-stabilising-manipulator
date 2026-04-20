

%% CONTROLLER SIMULATION - SETPOINT TRACKING

%% ROBOT ==================================================================

a = [0 14.5 14.5 1 7];
d = [12.5 3 0 3 0];
alpha =[-pi/2 0 pi pi/2 0];

robot = rigidBodyTree("DataFormat","column");
 
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
setFixedTransform(jnt1, [a(1) alpha(1) d(1) 0], 'dh');
body1.Joint = jnt1;

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
setFixedTransform(jnt2, [a(2) alpha(2) d(2) 0], 'dh');
body2.Joint = jnt2; 

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
setFixedTransform(jnt3, [a(3) alpha(3) d(3) 0], 'dh');
body3.Joint = jnt3;

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4', 'revolute');
setFixedTransform(jnt4, [a(4) alpha(4) d(4) 0], 'dh');
body4.Joint = jnt4;

body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5', 'revolute');
setFixedTransform(jnt5, [a(5) alpha(5) d(5) 0], 'dh');
body5.Joint = jnt5;

addBody(robot, body1, 'base');
addBody(robot, body2, 'body1');
addBody(robot, body3, 'body2');
addBody(robot, body4, 'body3');
addBody(robot, body5, 'body4');

%% Initialisation =========================================================

dt = 0.1; % fixed loop timing
T = 500;

q = [-1;-1;-1;-1;-1]; % initial joint config
qdot = [0;0;0;0;0]; % initial joint veloc.

% setpoint
pd = [15;15;15];  % desired position  
Rd = eul2rotm([0, 0, 0]); % desired orientation (level)

% Gains
Kp_p = 1; % position error gain
Kp_o = 4; % orientation error gain 
Kp_po = diag([Kp_p Kp_p Kp_p Kp_o Kp_o]); % gain matrix

lambda = 0.05;    % weighting for dls

figure(1);
ax = axes;


%% DATA STORAGE ==================================================================

data = struct();

data.true_roll  = zeros(T,1);
data.true_pitch = zeros(T,1);

data.x = zeros(T,1);
data.y = zeros(T,1);
data.z = zeros(T,1);

data.yoshikawa = zeros(T,1);
data.singular  = zeros(T,1);
data.det       = zeros(T,1);

data.error_orient_norm = zeros(T,1);
data.error_pos_norm = zeros(T,1);

time = (1:T)' * dt;

%% MAIN LOOP  =========================================================

for k = 1:T

    % get current end-effector pose
    H  = getTransform(robot, q, "body5");
    p  = H(1:3, 4);
    Re = H(1:3, 1:3);

    rollpitch = rotm2eul(Re);

    % Jacobian
    J     = geometricJacobian(robot, q, "body5");
    Jswap = J([4 5 6 1 2 3], :);

    % orientation error
    eo = 0.5 * (cross(Re(:,1), Rd(:,1)) + ...
                cross(Re(:,2), Rd(:,2)) + ...
                cross(Re(:,3), Rd(:,3)));
   
    rot_err_matrix = Re' * Rd;  % relative rotation from current to desired
    angle_err_rad  = acos((trace(rot_err_matrix) - 1) / 2); % rotation angle in radians
    data.error_orient_norm(k) = rad2deg(angle_err_rad);  % store in degrees

    % pose error (for setpoint pd dot = 0)
    e = [(pd - p); eo(1); eo(2)];

    % drop jacobian yaw row
    Je = Jswap([1 2 3 4 5], :);

    % DLS pseudoinv
    qdot = Je' * ((Je*Je' + lambda^2 * eye(size(Je,1))) \ (Kp_po * e));

    % calculate new joint angle 
    q    = q + qdot * dt;

    % store data
    data.x(k) = p(1);
    data.y(k) = p(2);
    data.z(k) = p(3);
    data.true_roll(k)  = rollpitch(3);
    data.true_pitch(k) = rollpitch(2);
    data.error_pos_norm(k) = norm(p - pd);

     % manipulability
    Jswap_svd      = svd(Jswap);
    data.yoshikawa(k) = prod(Jswap_svd);
    data.singular(k)  = min(Jswap_svd);
    data.det(k)       = det(Je);

    % animation
    show(robot,q,"Parent", ax, "PreservePlot", false);
    title("Task-space Setpoint Control");
    drawnow;

end 

% save data

save("ctrl_setpoint_results.mat", 'data', 'time');