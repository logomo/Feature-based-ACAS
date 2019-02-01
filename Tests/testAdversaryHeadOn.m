%% Emergeny head on scenario prototype
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
    

%Ghost vehicle
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
    missionControl2.trajectoryColor=[0 0 0] +0.5;
    missionControl2.plannedColor=[0 0 0] +0.5;
    missionControl2.vehicleName='TurboAcademic-3000';
    
%Adversary
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
    missionControl3=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    %Plot parameters
    missionControl3.trajectoryColor='m';
    missionControl3.plannedColor='k';
    missionControl3.vehicleName='TurboAcademic-3000';
    

  
% UTM control
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
    
%Show avoidanceGrid
    missionControl1.enableRasterRange = true;
    missionControl3.enableRasterRange = true;
    dtv = [];
% Simulation run
    k=0;
    while missionControl1.isActive || missionControl2.isActive
        k=k+1;
        utmControl.runSimulations;
        
        % induce adversary behaviour
        posor=missionControl1.vehicle.getActualPositionOrientation;
        missionControl3.goal = posor(1:3);
        missionControl3.runOnceWithPlot;
        
        % Mutual distance check
        ps1=missionControl1.vehicle.getActualPositionOrientation;
        ps2=missionControl3.vehicle.getActualPositionOrientation;
        distance =norm(ps1(1:3)-ps2(1:3));
        if distance <= 10
            a=1;
            missionControl1.notifyIntruder(3,missionControl3.vehicle.getActualPositionOrientation,1,1,3);
            dtv = [dtv,distance];
        end
        
        daspect([1 1 1]);
        axis([-10,50,5,35,-5,5])
        view(0,90)
        drawnow
        title(['Decision frame ',mat2str(k),'.']);
        Cmnf.exportFigure('UTMAdversary',k);
        if distance < 1.8
            break;
        end
    end
 
 % plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl2.vehicle.getPositionalData;
    v3position=missionControl3.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v2position);
    nCount=min(n1,n2);
    crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    crashDistanceReal = sqrt([1 1 1]*(v1position(:,1:nCount)-v3position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistance);
        Cmnf.fill(newX,newY,[0.5,0.5,0.5]);
        [newX,newY]=Cmnf.preparefillData(missionTime(1:nCount),crashDistanceReal);
        Cmnf.fill(newX,newY,'b');
        %plot([missionTime(1),missionTime(nCount)],[10,10],'r.-');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[10,10]);
        Cmnf.fill(newX,newY,'y','-.');
        [newX,newY]=Cmnf.preparefillData([missionTime(1),missionTime(nCount)],[2,2]);
        Cmnf.fill(newX,newY,'r','-.');
        title('UAV1 to UAV2 distance')
        xlabel('UTM time [s]')
        ylabel('Performance [m]')
        hold off
        axis([missionTime(1),missionTime(nCount),0,max(crashDistance)])
        legend('Expected Crash distance (t)','Crash distance (t)','Safe Margin (t)','Body Margin (t)')
    Cmnf.exportFigure('UtmAdversaryPerformance');
%% Trajectory tracking performance
    %figure(3)
    %missionControl1.plotRealvsPlanTrajectoryStatistics()
    %title('Path following performance  UAV1')
    %Cmnf.exportFigure('UtmCooperativeHeadOnUAV1PathFollowing');
    
    %figure(4)
    %missionControl3.plotRealvsPlanTrajectoryStatistics()
    %title('Path following performance  UAV2')
    %Cmnf.exportFigure('UtmCooperativeHeadOnUAV2PathFollowing');