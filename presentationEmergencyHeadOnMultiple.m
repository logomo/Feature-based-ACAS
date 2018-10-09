clear



%First vehicle
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];
    %test waypoints
    waypoints = [Waypoint(45,20,0)];


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
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleId=1;
    missionControl1.vehicleName='Shrotor 2000';
    missionControl1.plotMissionStaticContent;

%Second vehicle
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Vertical];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Vertical];
    %test waypoints
    waypoints = [Waypoint(-5,20,0)];


    %test vehicle properties
    orientation= [0;0;pi];
    position = [40;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl2=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
    missionControl2.vehicleId=2;
    missionControl2.vehicleName='Snuser 2000';
    missionControl2.trajectoryColor='c';
    missionControl2.plannedColor='m';
    missionControl2.plotMissionStaticContent;
   k=0; 
   
%Third vehicle
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Slash];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Slash];
    %test waypoints
    waypoints = [Waypoint(20,45,0)];


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
    missionControl3=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
        missionControl3.vehicleId=3;
        missionControl3.vehicleName='Bazmek 3000';
        missionControl3.trajectoryColor='g';
        missionControl3.plannedColor='y';
        missionControl3.plotMissionStaticContent;

%Fourth vehicle
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];
    %test waypoints
    waypoints = [Waypoint(20,-5,0)];


    %test vehicle properties
    orientation= [0;0;-pi/2];
    position = [20;40;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl4=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
        missionControl4.vehicleId=4;
        missionControl4.vehicleName='Zeby stvorka?';
        missionControl4.trajectoryColor='k';
        missionControl4.plannedColor='g';
        missionControl4.plotMissionStaticContent;
% run mission
k=0;
while missionControl1.isActive || missionControl2.isActive || missionControl3.isActive || missionControl4.isActive
    k=k+1;
    % First vehlicle notification
    missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    missionControl1.notifyIntruder(3,missionControl3.vehicle.getActualPositionOrientation,1);
    missionControl1.notifyIntruder(4,missionControl4.vehicle.getActualPositionOrientation,1);
    % Second vehicle notification
    missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl2.notifyIntruder(3,missionControl3.vehicle.getActualPositionOrientation,1);
    missionControl2.notifyIntruder(4,missionControl4.vehicle.getActualPositionOrientation,1);
    % Third vehicle notification
    missionControl3.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl3.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    missionControl3.notifyIntruder(4,missionControl4.vehicle.getActualPositionOrientation,1);
    % Fourth vehicle notificaiton
    missionControl4.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl4.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    missionControl4.notifyIntruder(3,missionControl3.vehicle.getActualPositionOrientation,1);
    
    missionControl1.runOnceWithPlot
    missionControl2.runOnceWithPlot
    missionControl3.runOnceWithPlot
    missionControl4.runOnceWithPlot
    figure(1)
    axis([-6,46,-6,46,-5,5])
    daspect([1,1,1]);
    title(['Decision frame ',mat2str(k),'.']);
    drawnow
    Cmnf.exportFigure('UtmEmergencyHeadOnMultiple',k);
end

figure(2)
% plot sequence 1-2
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl2.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,1)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')
% plot sequence 1-3
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl3.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,2)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV1 to UAV3 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)') 

% plot sequence 1-4
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl4.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,3)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV1 to UAV4 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')        

% plot sequence 2-3
    v1position=missionControl2.vehicle.getPositionalData;
    v2position=missionControl3.vehicle.getPositionalData;
    missionTime=missionControl2.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,4)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV2 to UAV3 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')  
        
% plot sequence 2-4
    v1position=missionControl2.vehicle.getPositionalData;
    v2position=missionControl4.vehicle.getPositionalData;
    missionTime=missionControl2.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,5)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV2 to UAV4 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')

% plot sequence 3-4
    v1position=missionControl3.vehicle.getPositionalData;
    v2position=missionControl4.vehicle.getPositionalData;
    missionTime=missionControl2.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    subplot(2,3,6)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[1.2,1.2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV3 to UAV4 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')         
    Cmnf.exportFigure('UtmEmergencyHeadOnMultiplePerformance');
    
%% Trajectory tracking performance
    figure(3)
    missionControl1.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance  UAV1')
    Cmnf.exportFigure('UtmEmergencyHeadOnMultipleUAV1PathFollowing');
    
    figure(4)
    missionControl2.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance  UAV2')
    Cmnf.exportFigure('UtmEmergencyHeadOnMultipleUAV2PathFollowing');
    
    figure(5)
    missionControl3.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance  UAV3')
    Cmnf.exportFigure('UtmEmergencyHeadOnMultipleUAV3PathFollowing');
    
    figure(6)
    missionControl4.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance  UAV4')
    Cmnf.exportFigure('UtmEmergencyHeadOnMultipleUAV4PathFollowing');    