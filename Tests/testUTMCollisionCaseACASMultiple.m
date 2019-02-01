%% Scenario ACAS Multiple collisions
clear

%Avoidance setup
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];
%First vehicle
    %test waypoints
    waypoints = [Waypoint(0,20,0),Waypoint(40,20,0)];


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
    missionControl1.vehicleName='SRotor-2000';

%Second vehicle
    %test waypoints
    waypoints = [Waypoint(40,20,0),Waypoint(0,20,0)];


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
    missionControl2.trajectoryColor='c';
    missionControl2.plannedColor='m';
    missionControl2.vehicleName='TurboAcademic-3000';

% Third vehicle
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
    missionControl3=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
    missionControl3.trajectoryColor='g';
    missionControl3.plannedColor='y';
    missionControl3.vehicleName='FlyingSaucer-4000';

% Foutrh vehicle
%test waypoints
    waypoints = [Waypoint(20,40,0),Waypoint(20,0,0)];


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
    missionControl4.trajectoryColor='k';
    missionControl4.plannedColor='g';
    missionControl4.vehicleName='DestroyerOfDreams-5000';
    
% MissionControl
    utmControl = UTMControl;
    utmControl.registerMission(missionControl1,false);
    utmControl.registerMission(missionControl2,false);
    utmControl.registerMission(missionControl3,false);
    utmControl.registerMission(missionControl4,false);
    %f = @() utmControl.createCollisionCase(1,2)
    %timeit(f)
% RuleEngine
    ruleEngine=RuleEngine();
    ruleEngine.activateRules(RuleJointPoint.MissionControlRunOnce,[...
        RuleCode.PriorRulesOfAir]);
    ruleEngine.activateRules(RuleJointPoint.MissionControlCollisionCaseSolution,[...
        RuleCode.CollisionCaseResulution]);
    ruleEngine.activateRules(RuleJointPoint.MissionControlAfterRun,[...
        RuleCode.PostRulesOfAir]);

    utmControl.injectRuleEngine(ruleEngine);
    %g=@()utmControl.runSimulations;
    %timeit(g)

% Simulation run
    for k=1:50
        utmControl.runSimulations;
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
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
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[8,8],'r.-');
        title('UAV3 to UAV4 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')    
