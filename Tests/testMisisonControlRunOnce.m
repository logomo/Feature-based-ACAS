%% Misiso0n control - scenario - run once - run after initialzation of other mission control scenarios
%%Waypoint check logic
[obj.finalWaypointReachedFlag,obj.waypointReachedFlag]=obj.checkAndSetObjective;
obj.obstacleFlag=0;
obstacleHits=0;
intruderHits=0;

if obj.finalWaypointReachedFlag 
    return
end

%%Path planning logic
if obj.waypointReachedFlag
    obj.movementBuffer=[];
end

obj.checkForcedPathReplaning;
%% intruder logic
detectedIntruders=obj.getDetectedIntruders;
intruderHits= obj.intersectIntrudersWithGrid(obj.avoidanceGrid,detectedIntruders);
if intruderHits > 0
    obj.intruderFlag=1;
else
    obj.intruderFlag=0;
end


%% static obstacles
[intersections,collisions]=getIntersectionCollisionCandidates(obj);
if ~isempty(intersections)
    obstacleHits=obj.intersectObstaclesWithGrid(obj.avoidanceGrid,intersections);
    if obstacleHits>0
        obj.staticObstacleFlag=1;
    else
        obj.staticObstacleFlag=0;
    end
else
    obj.staticObstacleFlag=0;
end

if obj.staticObstacleFlag || obj.intruderFlag
    obj.obstacleFlag = 1;
    obj.avoidanceGrid.recalculate;
    %obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Reachability,2)
    %obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Obstacle,3)
    %obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Visibility,4)
else
    obj.obstacleFlag = 0;
end

if isempty(obj.movementBuffer) || obj.forcerReplaningFlag || ~obj.staticObstacleFlag || obj.intruderFlag
    if obj.obstacleFlag
        [bestTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.avoidanceGrid);
    else
        [bestTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.navigationGrid);
    end
    if obj.obstacleFlag
        obj.avoidanceGrid.resetGrid
    end
        
    obj.movementBuffer=bestTrajectory.collectMovements;
    if obj.waypointUnreachableFlag
        a=1
    end
    if obj.collisionFlag
        b=1
    end
end
%% Movement execution
%Log begin
    log=FlightLog;
    log.simulationTime=obj.simulationTime;
    log.vehiclePosOrBefore=obj.vehicle.getActualPositionOrientation;
    log.executedMovement=obj.movementBuffer(1);
    log.movementBuffer=obj.movementBuffer;
    log.waypointID=obj.waypointID;
    log.waypoint=obj.goal;
    log.intersections=intersections;
    log.obstacleHits=obstacleHits;
    log.staticObstacleFlag=obj.staticObstacleFlag;
    log.detectedIntruders=detectedIntruders;
    log.intruderHits=intruderHits;
    log.intruderFlag=obj.intruderFlag;
    log.waypointReachedFlag=obj.waypointReachedFlag;
    log.waypointUnreachableFlag=obj.waypointUnreachableFlag;
    log.finalWaypointReachedFlag=obj.finalWaypointReachedFlag;
    log.collisionFlag=obj.collisionFlag;
    log.obstacleFlag=obj.obstacleFlag;
    log.forcerReplaningFlag=obj.forcerReplaningFlag;
% log interupt
% fly movement
obj.vehicle.fly(obj.movementBuffer(1));
% log continue
    log.vehiclePosOrAfter=obj.vehicle.getActualPositionOrientation;
    obj.missionLog=[obj.missionLog,log];
    obj.simulationTime=obj.simulationTime+Cmnf.simStep;
    log.showMissionStatus;
% log end
if length(obj.movementBuffer)~=1
    obj.movementBuffer=obj.movementBuffer(2:length(obj.movementBuffer));
else
    obj.movementBuffer=[];
end
%% Plot 
figure(1)
obj.plotMissionTrajectory;
obj.plotDetectedIntruders;

