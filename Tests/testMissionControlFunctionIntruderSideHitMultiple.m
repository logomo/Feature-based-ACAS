%% Misiso0n control - scenario - side hit multiple
clear
%test waypoints
waypoints = [Waypoint(0,0,0),Waypoint(55,0,0)];


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
i2=Intruder([6;8;-0.5],[0;-1;0],10);
i3=Intruder([6;8;-0.5],[0;-1;0],15);
i4=Intruder([6;8;-0.5],[0;-1;0],20);
i5=Intruder([6;8;0.5],[0;-1;0],25);
i6=Intruder([6;8;0.5],[0;-1;0],30);
i7=Intruder([6;8;0.5],[0;-1;0],35);
i8=Intruder([6;8;0.5],[0;-1;0],40);
intruders=[i1,i2,i3,i4,i5,i6,i7,i8];
obj.addIntruders(intruders);
obj.plotMissionStaticContent;
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
testMisisonControlRunOnce
%obj.plotRasterRange
axis([0,55,-8,8,-8,8])