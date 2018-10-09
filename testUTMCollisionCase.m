clear

%First vehicle
    %test waypoints
    waypoints = [Waypoint(0,20,0),Waypoint(40,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [0;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount);
    missionControl1.vehicleName='SRotor-2000';
    

%Second vehicle
    %test waypoints
    waypoints = [Waypoint(20,0,0),Waypoint(20,40,0)];


    %test vehicle properties
    orientation= [0;0;pi/2];
    position = [20;0;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl2=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount);
    %Plot parameters
    missionControl2.trajectoryColor='c';
    missionControl2.plannedColor='m';
    missionControl2.vehicleName='TurboAcademic-3000';
    

  
% MissionControl
    utmControl = UTMControl;
    utmControl.registerMission(missionControl1,false);
    utmControl.registerMission(missionControl2,false);
    %f = @() utmControl.createCollisionCase(1,2)
    %timeit(f)
    
    %g=@()utmControl.runSimulations;
    %timeit(g)

    %for k=1:14
    %    utmControl.createCollisionCase(1,2)
    %    utmControl.runSimulations;
    %end
    
