%% Scenario - storm avoidance early version
clear

navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(0,60,0)];


    %test vehicle properties
    orientation= [0;0;pi/2];
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
    %type=ExamplePolygonType.Poly5;
    %scale= 10.0;
    %rotation = 0;
    %center = [0;50;0];
    %base=ExamplePolygonType.getPolygonData(type);

    %scaled = base.*scale;
    %rotated = Cmnf.rot2D(rotation,scaled);
    %shifted = rotated + center(1:2);
    %plot(shifted(1,:),shifted(2,:))

    constraints= ExamplePolyConstraint(ExamplePolygonType.Poly5,[0;50;0],10,-pi/4);
    constraints.dynamize([0;50;0],[0;-0.5;0]);
    constraints.forceColor('m');
    missionControl1.putConstraints(constraints);
    % plot mission control
    missionControl1.plotMissionStaticContent;
    missionControl1.vehicleId = 1;
    missionControl1.vehicleName = 'L';
    
% run mission
k=0;
while ~missionControl1.finalWaypointReachedFlag
    figure(1)
    daspect([1,1,1]);
    axis([-10,20,-1,60,-5,5]) %TODO
    view(90,41)
    k=k+1;
    title(['Decision frame ',mat2str(k),'.']);
    missionControl1.runOnceWithPlot
    drawnow
end

%% Plot procedure
    time=missionControl1.vehicle.state.time;
    mcs=[];
    for cc = missionControl1.constraints
        if missionControl1.constraints.type == CostraintType.Moving
            mcs=[mcs,cc];
        end
    end
    cld=[];
    clid=[];
    clrad=[];
    clsm=[];
    trajectory=missionControl1.vehicle.getPositionalData;
    for k = 1:length(time)
        ktime=time(k);
        kpos=trajectory(:,k);
        cstc=[]; % constrainds timed centers
        for l=1:length(mcs)
           cstc=[cstc,mcs(l).estimateCenter(ktime)];
        end
        [val,ind]=min(sqrt([1 1 1]*((cstc-kpos).^2)));
        cld=[cld,val];
        clid=[clid,ind];
        clrad=[clrad,mcs(ind).radius];
        clsm=[clsm,mcs(ind).safetyMargin];
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
    a=fill(pTime,pBody,'r');
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
    title('Storm avoidance performance')
    xlabel('UTM time [s]')
    ylabel('Performance distance [m]')
    legend('UAS [m]','Safety margin [m]','Body [m]') 
    hold off
