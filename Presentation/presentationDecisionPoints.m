clear

navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal,MovementGroup.Vertical];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal,MovementGroup.Vertical];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(0,0,0),Waypoint(20,0,0),Waypoint(20,20,0),Waypoint(0,20,0),Waypoint(0,0,10)];


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
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    
    % Static obstacles - this is reactive part, we need to add constraints
    % obstacles=[ObstacleSphere([50;0;0],5,ObstacleType.Map),ObstacleSphere([100;50;0],5,ObstacleType.Map),ObstacleSphere([50;100;0],5,ObstacleType.Map)];
    % missionControl1.putObstacles(obstacles);
    
    % Static constraints
    %constraints=[BarrelConstraint([50;0;0],20),BarrelConstraint([100;50;0],20),BarrelConstraint([50;100;0],20)];
    c1=ExamplePolyConstraint(ExamplePolygonType.Poly5,[50;0;0],20,pi/4);
    c2=ExamplePolyConstraint(ExamplePolygonType.Hospital,[100;50;0],20,pi/4);
    c3=ExamplePolyConstraint(ExamplePolygonType.Unusual,[50;100;0],20,pi/4);
    c4=ExamplePolyConstraint(ExamplePolygonType.Square,[0;50;0],20,pi/4);
    c1.forceColor('c');
    c2.forceColor('c');
    c3.forceColor('c');
    c4.forceColor('c');
    c2.safetyMargin=7;
    c3.safetyMargin=8;
    c4.safetyMargin=4;
    %missionControl1.putConstraints([c1,c2,c3,c4]);
    % plot mission control
    missionControl1.plotMissionStaticContent;
    missionControl1.vehicleId = 1;
    missionControl1.vehicleName = 'L';
% run mission
k=0;
handles = 0;
while ~missionControl1.finalWaypointReachedFlag
    %missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    if mod(k,  5) == 0
        %prev_handles = handles;
        handles = missionControl1.plotRasterRange;
        %delete(prev_handles);
        figure(1)
        daspect([1,1,1]);
        view(-235.6000,19.4000)
        axis([-12,32,-12,32,-6,17])

        title(['']); 
        drawnow

        Cmnf.exportFigureForced('DecisionPoints',k);
    end
    
    k=k+1;
end

