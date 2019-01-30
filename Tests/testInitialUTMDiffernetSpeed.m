clear

%First vehicle
    %test waypoints
    waypoints = [Waypoint(10,20,0),Waypoint(40,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [10;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount);
    missionControl1.plotMissionStaticContent;

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
    missionControl2.plotMissionStaticContent;
    
% run mission
for k=1:45
    missionControl1.notifyIntruder(2,missionControl2.vehicle.getActualPositionOrientation,1);
    %missionControl2.notifyIntruder(1,missionControl1.vehicle.getActualPositionOrientation,1);
    missionControl1.runOnceWithPlot
    missionControl2.runOnceWithPlot
    missionControl2.runOnceWithPlot
    missionControl2.runOnceWithPlot
    missionControl2.runOnceWithPlot
    figure(1)
    title(['Decision frame ',mat2str(k),'.']);
    drawnow
end

% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl2.vehicle.getPositionalData;
    v2position=v2position(1:3,1:4:length(v2position));
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[1.2,1.2],'r.-');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')