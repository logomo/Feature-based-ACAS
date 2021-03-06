%% Simulation 7.4.2 Bule-Based Head-On
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
    missionControl1.vehicleName='SRotor-2000';
    

%Second vehicle
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
    missionControl2.trajectoryColor='m';
    missionControl2.plannedColor='k';
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
    k=0;
    while missionControl1.isActive || missionControl2.isActive
        k=k+1;
        utmControl.runSimulations;
        daspect([1 1 1]);
        axis([-10,50,10,30,-2,2])
        view(0,90)
        drawnow
        Cmnf.exportFigure('UtmCooperativeHeadOn',k);
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
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[10,10],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[10,10]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Crash distance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Cash distance (t)','Safe Margin (t)')
    Cmnf.exportFigure('UtmCooperativeHeadOnPerformance');
%% Trajectory tracking performance
    figure(3)
    missionControl1.plotRealvsPlanTrajectoryStatistics()
    title('Path following performance  UAV1')
    Cmnf.exportFigure('UtmCooperativeHeadOnUAV1PathFollowing');
    
    figure(4)
    missionControl2.plotRealvsPlanTrajectoryStatistics()
    title('Path following performance  UAV2')
    Cmnf.exportFigure('UtmCooperativeHeadOnUAV2PathFollowing');
    
 %% Computaiton Load

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
    Cmnf.exportFigure('RuleBasedHeadOnComputationTime');