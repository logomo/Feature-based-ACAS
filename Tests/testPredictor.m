%% Predictor test script
clear;
%initial position
x_0 = 0;
y_0 = 0;
z_0 = 0;
%initial orientation
alpha_0 = 0;
beta_0 =  0;
gamma_0 = 0;

%initial time 
t_0=0;
state = [x_0;y_0;z_0;alpha_0;beta_0;gamma_0;t_0];
lm=LinearizedModel; % build initial model for predictor

% Create sample Movement buffer
testBuffer = [MovementType.Straight, MovementType.Down, MovementType.Straight,...
              MovementType.Up,MovementType.Straight,MovementType.Up,MovementType.Straight,...
              MovementType.Down,MovementType.Straight];

% predict state
outState = lm.predictBuffer(state,testBuffer);

%compare with real wehicle (just fast check)
omega_alpha_0 = 0;
omega_beta_0 =  0;
omega_gamma_0 = 0;
x_0 = 0;
y_0 = 0;
z_0 = 0;

%input vector
velocity = 1;
omega_alpha = 0;
omega_beta =  0;
omega_gamma = 0;
%movement matrix
t_sim = 1;

s = State(omega_alpha_0,omega_beta_0,omega_gamma_0,x_0,y_0,z_0, velocity);
v = Vehicle(s);
v.flyBuffer(testBuffer); 

% test plot
figure(1)
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
grid on
title('Trajectory comparison based on prediction [m]')
hold on
    plot3(outState(1,:),outState(2,:),outState(3,:),'r');
    v.plotTrajectory;
hold off 