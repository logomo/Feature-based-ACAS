clear

navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal];

%Vehicle
    %Maze definition
    mazeMap=[1,2,3,4,1;
             2,6,0,0,4;
             1,2,3,0,1;
             1,0,0,0,1;
             1,0,4,1,1;
             3,0,0,0,4;
             3,1,1,0,3;
             2,5,0,0,4;
             1,2,4,2,2;];
    % mazeMap=[1,1,1,1,1,1,1,1,1; % this one needs an additional waypoint
    % guidance
    %         1,0,0,0,0,0,1,6,1;
    %         1,0,3,4,1,0,1,0,1;
    %         2,0,0,0,4,0,1,0,1;
    %         1,2,3,0,1,0,1,0,1;
    %         1,0,0,0,1,0,1,0,1;
    %         1,0,4,1,1,0,1,0,1;
    %         3,0,0,0,4,0,1,0,1;
    %         3,1,1,0,3,0,1,0,1;
    %         2,5,0,0,4,0,0,0,1;
    %         1,2,4,2,2,3,3,1,1;];     
    maze= MazeMatrix(mazeMap);
    obstacles=maze.generateMazeObstacles();
    
    %test waypoints
    waypoints = [Waypoint(maze.finalWaypoint)];


    %test vehicle properties
    orientation= [0;0;0];
    position = maze.startPosition;

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
    constraints=obstacles;
    missionControl1.putConstraints(constraints);
    % plot mission control
    missionControl1.plotMissionStaticContent;
    
% run mission
k=0;
while ~missionControl1.finalWaypointReachedFlag
    %missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    
    figure(1);
    daspect([1,1,1]);
    %axis([-5,45,-5,45,-10,10])
    k=k+1;
    title(['Decision frame ',mat2str(k),'.']);
    f.DataAspectRatioMode='manual';
    f.DataAspectRatio=[1,1,1];
    drawnow
end

