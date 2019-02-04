%% Scenario 7.3.5 Emergency Converging
clear

navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(40,20,0)];


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
    missionControl1.plotMissionStaticContent;
    missionControl1.vehicleId=1;
    missionControl1.vehicleName='Srotor 2000';
    missionControl1.plotMissionStaticContent;
%Second vehicle
    %test waypoints
    waypoints = [Waypoint(20,40,0)];


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
    missionControl2=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
        missionControl2.trajectoryColor='c';
        missionControl2.plannedColor='m';
        missionControl2.vehicleId=2;
        missionControl2.vehicleName='Turbo academic 2000';
        missionControl2.plotMissionStaticContent;
   k=0; 
% run mission
while missionControl1.isActive || missionControl2.isActive
    k=k+1;
    missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    missionControl2.runOnceWithPlot
    figure(1)
    daspect([1,1,1]);
    view(-35,49)
    axis([-2,42,-2,42,-5,5])
    title(['Decision frame ',mat2str(k),'.']);
    drawnow
    Cmnf.exportFigure('UtmEmergencyConverging',k);
end

% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl2.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        %%plot(missionTime(1:nCount),crashDistance,'b')
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
        Cmnf.exportFigure('UtmEmergencyConvergingPerformance');

%% Trajectory tracking performance    
    figure(3)
    missionControl1.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance UAV1')
    Cmnf.exportFigure('UtmEmergencyConvergingUAV1PathFollowing');
    
    figure(4)
    missionControl2.plotRealvsPlanTrajectoryStatistics()
    subplot(3,1,1)
    title('Path following performance UAV2')
    Cmnf.exportFigure('UtmEmergencyConvergingUAV2PathFollowing');
    
    
%% computaiton Load

    figure(5)
    aat=[];
    
    subplot(2,1,1)
    aat= [aat,missionControl1.plotAndCalculateComputationTime];
    daspect('auto')
    title('UAS 1')
    
    subplot(2,1,2)
    aat= [aat,missionControl2.plotAndCalculateComputationTime];
    daspect('auto')
    title('UAS 2')
    
    
    mean(aat')*100
    Cmnf.exportFigure('EmergencyConvergingComputationTime');