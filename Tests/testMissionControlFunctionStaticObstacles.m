%% Mission control initialization - static obstacles - various method performance
clear
%test waypoints
waypoints = [Waypoint(0,0,0),Waypoint(20,0,0),Waypoint(20,20,0),Waypoint(0,20,0),Waypoint(0,0,10)];
%waypoints = [Waypoint(20,0,10)];

%test vehicle properties
orientation= [0;0;0];
position = [0;0;0];

%Avoidance/Navigation grid 
gridDistance=10;
gridHorizontalRange=pi/4;
gridHorizontalCount=7;
gridVerticalRange=pi/6;
gridVerticalCount=5;


%Create mission control object
obj=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount);

obstacles=[ObstacleSphere([10;1;0],2),ObstacleSphere([20;10;0],2),ObstacleSphere([10;20;0],2)];
obj.putObstacles(obstacles);
obj.plotMissionStaticContent;
%Just for showing off purpose
axis([-10,30,-10,30,-5,15])