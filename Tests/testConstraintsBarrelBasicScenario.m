%% Scenario - basic barel constraint avoidance
clear

navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(100,0,0),Waypoint(100,100,0),Waypoint(0,100,0),Waypoint(0,0,0)];


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
    missionControl1.plotMissionStaticContent;
    missionControl1.vehicleId=1;
    missionControl1.vehicleName='Srotor 2000';
    
    % Static obstacles - this is reactive part, we need to add constraints
    % obstacles=[ObstacleSphere([50;0;0],5,ObstacleType.Map),ObstacleSphere([100;50;0],5,ObstacleType.Map),ObstacleSphere([50;100;0],5,ObstacleType.Map)];
    % missionControl1.putObstacles(obstacles);
    
    % Static constraints
    constraints=[BarrelConstraint([50;0;0],20),BarrelConstraint([100;50;0],20),BarrelConstraint([50;100;0],20)];
    missionControl1.putConstraints(constraints);
    % plot mission control
    missionControl1.plotMissionStaticContent;
    
% run mission
k=0;
while ~missionControl1.finalWaypointReachedFlag
    %missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    
    figure(1)
    daspect([1,1,1]);
    %axis([-5,45,-5,45,-10,10])
    k=k+1;
    title(['Decision frame ',mat2str(k),'Color',[255/255,165/255,0]],'Lin'); 
    drawnow
end

