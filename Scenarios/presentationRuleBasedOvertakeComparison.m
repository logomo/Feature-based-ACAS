%% Scenario 7.4.4 Overtake
%% NOTE
% enableTracePlot=true; in Cmnf.m - enable trace plot for merier and
% moother experiense
%% Double
%clear;

%Avoidance setup
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(110,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-40;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleName='SRotor-2000';
    missionControl1.vehicleActualVelocity=2;
    missionControl1.trajectoryColor='b';
    missionControl1.plannedColor='r';
%Second vehicle
    %test waypoints
    waypoints = [Waypoint(80,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-20;20;0];

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
    

  
% MissionControl
    utmControl = UTMControl;
    utmControl.registerMission(missionControl1,false);
    utmControl.registerMission(missionControl2,false);
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
    for k=1:100
        utmControl.runSimulations;
        axis([-45,120,10,25,-2,2]);
    end
    
% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v1position=v1position(:,1:missionControl1.vehicleActualVelocity:end);
    v2position=missionControl2.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        overtake2x=missionTime(1:nCount);
        overtake2y=crashDistance;
        plot(missionTime(1:nCount),crashDistance,'b')
        plot([missionTime(1),missionTime(nCount)],[5,5],'r.-');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')


%% Triple
%clear;

%Avoidance setup
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(110,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-40;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleName='SRotor-2000';
    missionControl1.vehicleActualVelocity=3;
    missionControl1.trajectoryColor='g';
    missionControl1.plannedColor='y';
%Second vehicle
    %test waypoints
    waypoints = [Waypoint(80,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-20;20;0];

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
    

  
% MissionControl
    utmControl = UTMControl;
    utmControl.registerMission(missionControl1,false);
    utmControl.registerMission(missionControl2,false);
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
    for k=1:100
        utmControl.runSimulations;
        axis([-45,120,10,25,-2,2]);
    end
    
% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v1position=v1position(:,1:missionControl1.vehicleActualVelocity:end);
    v2position=missionControl2.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        overtake3x=missionTime(1:nCount);
        overtake3y=crashDistance;
        plot(missionTime(1:nCount),crashDistance,'g')
        plot([missionTime(1),missionTime(nCount)],[5,5],'r.-');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')
%% Quarduple
%clear;

%Avoidance setup
    navigationType=ReachSetCalculation.ACASLike;
    navigationParams=containers.Map;
    navigationParams('separations')=[MovementGroup.Horizontal];
    avoidanceType=ReachSetCalculation.ACASLike;
    avoidanceParams=containers.Map;
    avoidanceParams('separations')=[MovementGroup.Horizontal];

%First vehicle
    %test waypoints
    waypoints = [Waypoint(110,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-40;20;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/6;
    gridVerticalCount=5;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleName='SRotor-2000';
    missionControl1.vehicleActualVelocity=4;
    missionControl1.trajectoryColor='k';
    missionControl1.plannedColor='c';
%Second vehicle
    %test waypoints
    waypoints = [Waypoint(80,20,0)];


    %test vehicle properties
    orientation= [0;0;0];
    position = [-20;20;0];

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
    

  
% MissionControl
    utmControl = UTMControl;
    utmControl.registerMission(missionControl1,false);
    utmControl.registerMission(missionControl2,false);
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
    for k=1:100
        utmControl.runSimulations;
        axis([-45,120,10,25,-2,2]);
    end
    
% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v1position=v1position(:,1:missionControl1.vehicleActualVelocity:end);
    v2position=missionControl2.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        overtake4x=missionTime(1:nCount);
        overtake4y=crashDistance;
        plot(missionTime(1:nCount),crashDistance,'k')
        plot([missionTime(1),missionTime(nCount)],[5,5],'r.-');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')
        
%% Clean up the mess which you made!
figure(1)
title('Overtake maneuver - Comparison')

hold on;

h = zeros(7, 1);
h(1) = plot(NaN,NaN,'-oc','MarkerFaceColor','c');
h(2) = plot(NaN,NaN,'-ob','MarkerFaceColor','b');
h(3) = plot(NaN,NaN,'-og','MarkerFaceColor','g');
h(4) = plot(NaN,NaN,'-ok','MarkerFaceColor','k');
h(5) = plot(NaN,NaN,'-r');
h(6) = plot(NaN,NaN,'-y');
h(7) = plot(NaN,NaN,'-c');
legend(h, 'Overtaken trajectory','Trajectory 2x speed','Trajectory 3x speed','Trajectory 4x speed','Plan 2x speed','Plan 3x speed','Plan 4x speed');
daspect([1,0.2,1]);
view(0,90)
Cmnf.exportFigure('UtmCooperativeOvertakeMultipleTrajectories');

figure(2)
hold on;
h = zeros(4, 1);
h(1) = plot(NaN,NaN,'-r');
h(2) = plot(NaN,NaN,'-b');
h(3) = plot(NaN,NaN,'-g');
h(4) = plot(NaN,NaN,'-k');
legend(h, 'Safety margin','Mutual distance 2x speed','Mutual distance 3x speed','Mutual distance 4x speed');

Cmnf.exportFigure('UtmCooperativeOvertakeMultiplePerformance');

%figure(4)

