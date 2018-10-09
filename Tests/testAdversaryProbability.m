%% Adversary probability distribution function test example
clear
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);                        % Initialize standard grid
adv=AdversaryVehicle([5;5;5],[1;0;0;0],pi/4,pi/6);                          % Create adversary at global position [5,5,5], with velocity 1 and uncertainities horizontal pi/4, verticalpi/6
adv.debug = 1;                                                              % Set adversarial behaviour to debug
%for k=1:20
%   r=adv.calculateElipse(k,1);
%end

r=adv.calculateElipse(50,1);                                                %Create figures
close(4)
close(1)
figure(2)
title('Probability p(x,y) distribution in direct cut')
xlabel('x [m]')
ylabel('y [m]')
zlabel('Probability of passing p(x,y)')
grid on;
figure(3)
title('Expected reach time')
xlabel('x [m]')
ylabel('y [m]')
zlabel('Expected reach time [s]')
grid on;