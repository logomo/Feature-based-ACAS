%% Misiso0n control - scenario - side hit 
clear
%test waypoints
waypoints = [Waypoint(0,0,0),Waypoint(25,0,0)];


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

i1=Intruder([6;8;-0.5],[0;-1;0],5);
intruders=[i1];
obj.addIntruders(intruders);
obj.plotMissionStaticContent;
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
obj.plotRasterRange
axis([0,25,-8,8,-8,8])