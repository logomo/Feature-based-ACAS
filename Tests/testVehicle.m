%% Test vehicle - unsuprisingly tests vehicle constructor
clear;
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
v.fly(MovementType.Straight);
v.fly(MovementType.Down);
v.fly(MovementType.Straight);
v.fly(MovementType.Up);
v.fly(MovementType.Straight);
v.fly(MovementType.Up);
v.fly(MovementType.Straight);
v.fly(MovementType.Down);
v.fly(MovementType.Straight);

figure(1)
grid on
hold on
v.plotTrajectory;
hold off

