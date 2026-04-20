
%% CONTROLLER SIMULATION - TRAJECTORY TRACKING 

%% ROBOT ==================================================================

% Define DH params

a = [0 14.5 14.5 1 7];
d = [12.5 3 0 3 0];
alpha =[-pi/2 0 pi pi/2 0];

robot = rigidBodyTree("DataFormat","column");
 
% first body

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');

setFixedTransform(jnt1, [a(1) alpha(1) d(1) 0], 'dh');
body1.Joint = jnt1;


% second body 

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');

setFixedTransform(jnt2, [a(2) alpha(2) d(2) 0], 'dh');
body2.Joint = jnt2; 


% third body

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');

setFixedTransform(jnt3, [a(3) alpha(3) d(3) 0], 'dh');
body3.Joint = jnt3;

% fourth body

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4', 'revolute');

setFixedTransform(jnt4, [a(4) alpha(4) d(4) 0], 'dh');
body4.Joint = jnt4;

%fifth body

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

dt = 0.1; % time passing each iteration 
T = 500;

q = [-1;-1;-1;-1;-1]; % initial joint vals

qdot = [0;0;0;0;0]; % inital joint veloc.

% for trajectories
radius = 15;
omega = 0.6;

%seperating position and orientation gains
Kp_p = 1;
Kp_o = 4;
Kp_po = diag([Kp_p Kp_p Kp_p Kp_o Kp_o]);

% DLS lambda
lambda = 0.05;

% figure(1);
% ax = axes;

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

    t = k*dt;

    Rd = eul2rotm([0, 0, 0]);

    %% TRAJECTORIES 
    % 
    % % circle trajectory position and desired rotation matrix
    % pd = [10 + radius*cos(omega*t); 10 + radius*sin(omega*t);20];
    % 
    % % circle trajectory velocity (derivative)
    % pd_dot =[-radius*omega*sin(omega*t); radius*omega*cos(omega*t);0;0;0];

    % 
    % % helix trajectory 
    % z_rate = 0.15;
    % % 
    % pd = [10 + radius*cos(omega*t);
    %       10 + radius*sin(omega*t);
    %       15 + z_rate*t];
    % 
    % pd_dot = [-radius*omega*sin(omega*t);
    %            radius*omega*cos(omega*t);
    %            z_rate;
    %            0;0];

     % % Square
    % L = 10;
    % Tseg = 10;
    % 
    % tau = mod(t, 4*Tseg);
    % 
    % if tau < Tseg
    %     pd = [10 + L*(tau/Tseg); 10; 20];
    %     pd_dot = [L/Tseg; 0; 0; 0;0];
    % elseif tau < 2*Tseg
    %     pd = [10 + L; 10 + L*((tau-Tseg)/Tseg); 20];
    %     pd_dot = [0; L/Tseg; 0; 0;0];
    % elseif tau < 3*Tseg
    %     pd = [10 + L*(1 - (tau-2*Tseg)/Tseg); 10 + L; 20];
    %     pd_dot = [-L/Tseg; 0; 0; 0;0];
    % else
    %     pd = [10; 10 + L*(1 - (tau-3*Tseg)/Tseg); 20];
    %     pd_dot = [0; -L/Tseg; 0; 0;0];
    % end
    

    % figure eight
    pd = [10 + radius*sin(omega*t);
      10 + radius*sin(2*omega*t);
      20];

    pd_dot = [radius*omega*cos(omega*t);
              2*radius*omega*cos(2*omega*t);
              0;
              0;0];

    % store desired traj. to plot
    data.x_des(k) = pd(1);
    data.y_des(k) = pd(2);
    data.z_des(k) = pd(3);

    % get current end-effector position
    H = getTransform(robot,q,"body5");
    p = H(1:3,4);
    Re = H(1:3,1:3);

    rollpitch = rotm2eul(Re); % ground truth orientation

    % Calculate Jacobian
    J = geometricJacobian(robot,q,"body5");
    Jswap = J([4 5 6 1 2 3], :); % swap because matlab different order

    % orientation error
    eo = 0.5 * (cross(Re(:,1),Rd(:,1)) + cross(Re(:,2),Rd(:,2)) + cross(Re(:,3),Rd(:,3)));

    % pose error
    e = [(pd-p);eo(1);eo(2)];

    % drop jacobian yaw row
    Je = Jswap([1 2 3 4 5], :);

    % calculate angle change using DLS

    qdot = Je' * ((Je*Je' + lambda^2 * eye(size(Je,1))) \ (pd_dot + Kp_po*e));
    q = q + qdot*dt;

    % manipulability data
    Jswap_svd = svd(Jswap); % take svd
    data.yoshikawa(k) = prod(Jswap_svd);
    data.singular(k) = min(Jswap_svd);
    data.det(k) = det(Je);
    
    % store ground truth
    data.x(k) = p(1);
    data.y(k) = p(2);
    data.z(k) = p(3);
    data.true_roll(k) = rollpitch(3);
    data.true_pitch(k) = rollpitch(2); 

    data.error_pos_norm(k) = norm(p - pd);

    % % animation
    % show(robot,q,"Parent", ax, "PreservePlot", false);
    % title("Task-space Trajectory Control");
    % drawnow;

end

% save data

save("ctrl_traj_results.mat", 'data', 'time');
