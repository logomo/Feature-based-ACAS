%% Misiso0n control - scenario - static obstacles
clear
%test waypoints
waypoints = [Waypoint(0,0,0),Waypoint(20,0,0),Waypoint(20,20,0),Waypoint(0,20,0),Waypoint(0,0,10)];

%test vehicle properties
orientation= [0;0;0];
position = [0;0;0];

%Avoidance grid 
gridDistance=10;
gridHorizontalRange=pi/4;
gridHorizontalCount=7;
gridVerticalRange=pi/6;
gridVerticalCount=5;



obj=MissionControl;
%Start mission control creation
Cmnf.logc(obj,'Creating mission control object');

% Add waypoints
obj.waypoints = waypoints;
for k=1:length(obj.waypoints)
    str = ['Adding waypoint ',mat2str(k),'. with value ',mat2str(obj.waypoints(k).position)];
    Cmnf.logc(obj,str);
end

%Create vehicle
omega_alpha_0=orientation(1);
omega_beta_0=orientation(2);
omega_gamma_0=orientation(3);
x_0=position(1);
y_0=position(2);
z_0=position(3);
velocity=Cmnf.vehicleSpeed;

Cmnf.logc(obj,'Creating state object')
s = State(omega_alpha_0,omega_beta_0,omega_gamma_0,x_0,y_0,z_0, velocity);

Cmnf.logc(obj,'Creating vehicle object')
v = Vehicle(s);
obj.vehicle=v;

%Create avoidance grid
ds=0;
de=gridDistance;
ts=-gridHorizontalRange;
te=gridHorizontalRange;
ps=-gridVerticalRange;
pe=gridVerticalRange;
lc=gridDistance/(Cmnf.vehicleSpeed*Cmnf.simStep);
hc=gridHorizontalCount;
vc=gridVerticalCount;
Cmnf.logc(obj,'Creating avoidance grid')
avoidanceGrid=AvoidanceGrid(ds,de,ts,te,ps,pe,lc,hc,vc);
obj.avoidanceGrid=avoidanceGrid;
Cmnf.logc(obj,'Starting reach set calculation')
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
avoidanceGrid.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
Cmnf.logc(obj,'Finished reach set calculation');
Cmnf.logc(obj,'Set initial values');
avoidanceGrid.resetGrid;