%% TEST CASE nice plot of vehicle
state=State(0,0,0,0,0,0, 1);
vehicle=Vehicle(state);
figure(1)
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
grid on
hold on 
for k=1:50
handles=plotPlaneModel(vehicle,0.6,'c');
vehicle.fly(MovementType.UpRight);
end
hold off