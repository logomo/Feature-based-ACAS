%Add master path
addpath 'C:\Users\H172461\Documents\MATLAB\ProbabilisticEngine\'
%initial state
omega_alpha_0 = 0;
omega_beta_0 =  0;
omega_gamma_0 = 0;
x_0 = 0;
y_0 = 0;
z_0 = 0;

%input vector
%velocity = 1;
%omega_alpha = 0;
%omega_beta =  0;
%omega_gamma = 0;
%movement matrix
%t_sim = 1;


linearizedStates=[];
figure (1)
hold on 
grid on
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
title('Movement prototypes')
for k=0:8
    omega_alpha_0 = 0;
    omega_beta_0 =  0;
    omega_gamma_0 = 0;
    x_0 = 0;
    y_0 = 0;
    z_0 = 0;
    movement = Cmnf.getMovement(k);
    velocity = movement(4);
    omega_alpha = movement(1);
    omega_beta =  movement(2);
    omega_gamma = movement(3);
    t_sim = movement(5);
    sim('plane');
    [m,n]=size(inputState);
    linearizedStates=[linearizedStates,inputState(m,:)'];
    plot3(inputState(:,5),inputState(:,6),inputState(:,7))
end
plot3(linearizedStates(5,:),linearizedStates(6,:),linearizedStates(7,:),'*r');
hold off
save ('model','linearizedStates');