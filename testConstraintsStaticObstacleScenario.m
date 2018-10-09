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
    missionControl1.putConstraints([c1,c2,c3,c4]);
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
    
    figure(1)
    daspect([1,1,1]);
    view(180,45)
    axis([-40,140,-20,140,-5,5])
    k=k+1;
    title(['Decision frame ',mat2str(k),'.']); 
    drawnow
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
    title('Building avoidance performance')
    xlabel('UTM time [s]')
    ylabel('Performance distance [m]')
    legend('UAS [m]','Safety margin [m]','Body [m]') 
    hold off

