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
    
    % Static obstacles - this is reactive part, we need to add constraints
    % obstacles=[ObstacleSphere([50;0;0],5,ObstacleType.Map),ObstacleSphere([100;50;0],5,ObstacleType.Map),ObstacleSphere([50;100;0],5,ObstacleType.Map)];
    % missionControl1.putObstacles(obstacles);
    
    % Static constraints
    constraints=obstacles;
    missionControl1.putConstraints(constraints);
    % plot mission control
    missionControl1.plotMissionStaticContent;
    missionControl1.vehicleId = 1;
    missionControl1.vehicleName = 'L';
% run mission
k=0;
while ~missionControl1.finalWaypointReachedFlag
    %missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    
    figure(1);
    daspect([1,1,1]);
    view(44.4800,38.8000);
    %axis([-5,45,-5,45,-10,10])
    k=k+1;
    title(['Decision frame ',mat2str(k),'.']);
    f.DataAspectRatioMode='manual';
    f.DataAspectRatio=[1,1,1];
    drawnow
    Cmnf.exportFigure('ConstraintsPolynomialMaze',k);
end

%% Draw Clash distance performance
    % gather Data
    cCenters=[];
    cRadius=[];
    cSafetyMargin=[];
    cID=[];
    cc=missionControl1.constraints;
    for cn=cc
        cCenters=[cCenters,cn.center];
        cRadius=[cRadius,cn.radius];
        cSafetyMargin=[cSafetyMargin,cn.safetyMargin];
        cID=[cID,cn.id];
    end
    
    % calculate closest obstacle
    cld=[];
    clid=[];
    clrad=[];
    clsm=[];
    trajectory=missionControl1.vehicle.getPositionalData;
    for k=1:length(trajectory)
        pos = trajectory(:,k);
        [val,ind]=min(sqrt([1,1,1]*((cCenters-pos).^2)));
        cld=[cld,val];
        clid=[clid,cID(ind)];
        clrad=[clrad,cRadius(ind)];
        clsm=[clsm,cSafetyMargin(ind)];
    end
    
    % time to plot
    noise=rand(1,length(clrad))-0.5;
    pDistance=[0,cld+noise,0];
    pBody = [0,clrad+noise,0];
    pSafetyMargin = [0,clsm+clrad + noise,0];
    pTime=missionControl1.vehicle.state.time;
    pTime=[pTime(1),pTime,pTime(length(pTime))];
    figure(2)
    grid on 
    hold on
    %plot(pTime,pSafetyMargin,'Color',[255/255,99/255,71/255],'Linestyle','-.')
    %plot(pTime,pBody,'Color',[255/255,0/255,0/255],'Linestyle','--')
    %plot(pTime,pDistance,'Color','b','Linestyle','--')
    a=fill(pTime,pDistance,'b');
    a.FaceAlpha=0.2;
    a.EdgeColor='b';
    a=fill(pTime,pSafetyMargin,'y');
    a.FaceAlpha=0.2;
    a.EdgeColor='y';
    a.LineStyle='-.';
    a=fill(pTime,pBody,'r')
    a.FaceAlpha=0.2;
    a.EdgeColor='r';
    a.LineStyle='-.';
    
    %Now set axes
    cVec=[cld+noise,clrad+noise,clsm+clrad + noise];
    ymax=floor(max(cVec)+1);
    ymin=floor(min(cVec));
    xmin=min(pTime);
    xmax=max(pTime);
    axis([xmin,xmax,ymin,ymax])
    
    %Labeling
    title('Maze avoidance performance')
    xlabel('UTM time [s]')
    ylabel('Performance distance [m]')
    legend('UAS [m]','Safety margin [m]','Body [m]') 
    hold off
    Cmnf.exportFigure('ConstraintsPolynomialMazePerformance');
 %% Trajectory tracking performance
    figure(3)
    title('Path following performance')
    missionControl1.plotRealvsPlanTrajectoryStatistics()
    Cmnf.exportFigure('ConstraintsPolynomialMazePathFollowing');

    

    

