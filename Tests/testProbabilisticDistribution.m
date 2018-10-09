%% Basic framework functionality for adrersary test
% Displays probabilistic distribution for adversary
%
%
clear;
%gc=GridCell(0,1,0,pi/4,0,pi/6);
%layer = GridLayer(5,6,-pi/4,pi/4,-pi/6,pi/6,6,4);
%layer.getCellIndexes(0,0);
%ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);
%ag.getCells(1:10,1:6,4);
%tc=ag.getCells(5,4,2);
%figure(1);
%ag.plotBaseSlice(1:10,1:6,3,0,0,1)
%figure(2);
%ag.plotBaseSlice(1:10,1,1:4,1,0,1)
%av =AvoidanceGrid(0,50,-pi/4,pi/4,-pi/6,pi/6,50,24,16);
%figure(3);
%av.plotBaseSlice(1:50,1:24,3,0,1,1)
%av.plotBaseSlice(1:50,1:24,3,0,1,2)
%av.plotBaseSlice(1:50,1:24,3,0,1,3)
%av.plotBaseSlice(1:50,1:24,3,0,1,4)
%av.plotBaseSlice(1:50,1:24,3,0,1,5)
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);
adv=AdversaryVehicle([5;5;5],[1;0;0;0],pi/4,pi/6);
adv.debug = 1;
%for k=1:20
%   r=adv.calculateElipse(k,1);
%end

r=adv.calculateElipse(50,1);
close(4)
close(1)
figure(2)
title('Probability p(x,y) distribution in direct cut')
xlabel('x-distance from trajectory [m]')
ylabel('y-distance from trajectory [m]')
zlabel('Probability of passing p(x,y)')
grid on;
figure(3)
title('Expected reach time')
xlabel('x-distance from trajectory [m]')
ylabel('y-distance from trajectory [m]')
zlabel('Expected reach time [s]')
grid on;