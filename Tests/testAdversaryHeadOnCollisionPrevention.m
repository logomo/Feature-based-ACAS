%% Scenario prototype Emergency head on
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
    missionControl1.enableRasterRange = false;
    missionControl3.enableRasterRange = false;
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
        axis([-10,50,-5,35,-5,5])
        view(0,90)
        drawnow
        title(['Decision frame ',mat2str(k),'.']);
        Cmnf.exportFigure('UtmAdversaryPursuit',k);
        if distance < 1.8
            break;
        end
        if k == 16
            break;
        end
    end
 
    
% plot situation
    hold on
    handles = [];
    missionControl2.lastTrajectoryPlotHandles = missionControl2.actualTrajectoryPlotHandles;
    missionControl2.plotRemoveLastTrajectoryFromMissionFigure;
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    delete(handles);
    
    % make straight lines
    missionControl1.movementBuffer = zeros(1,10);
    missionControl3.movementBuffer = zeros(1,10);
    
    %Frame 17 1st turning
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Left);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 17.')
    Cmnf.exportFigure('UtmAdversaryPursuit',17);
    delete(handles);
    
    %Frame 18 2nd turning
    hold on
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Left);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 18.')
    Cmnf.exportFigure('UtmAdversaryPursuit',18);
    delete(handles);
    
    %Frame 19 3nd turning
    hold on
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Left);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 17.')
    Cmnf.exportFigure('UtmAdversaryPursuit',19);
    delete(handles);
    
    %Frame 20 4th turning
    hold on
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;

    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 20.')
    Cmnf.exportFigure('UtmAdversaryPursuit',20);
    delete(handles);
    
    %Frame 21 5nd turning
    hold on
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 21.')
    Cmnf.exportFigure('UtmAdversaryPursuit',21);
    delete(handles);
    
    %Frame 22 6th turning
    hold on
    missionControl1.vehicle.fly(MovementType.Right);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 22.')
    Cmnf.exportFigure('UtmAdversaryPursuit',22);
    delete(handles);
    
    
    %Frame 23 Straight
    hold on
    missionControl1.vehicle.fly(MovementType.Straight);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 23.')
    Cmnf.exportFigure('UtmAdversaryPursuit',23);
    delete(handles);
    
    %Frame 24 Straight
    hold on
    missionControl1.vehicle.fly(MovementType.Straight);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 24.')
    Cmnf.exportFigure('UtmAdversaryPursuit',24);
    delete(handles);
    
    %Frame 25 Straight
    hold on
    missionControl1.vehicle.fly(MovementType.Straight);
    missionControl1.plotMissionTrajectory;
    missionControl1.plotRemoveLastTrajectoryFromMissionFigure;
    missionControl3.vehicle.fly(MovementType.Straight);
    missionControl3.plotMissionTrajectory;
    missionControl3.plotRemoveLastTrajectoryFromMissionFigure;
    
    hold on
    handles = [];
    line =[missionControl1.getGlobalCoordinates([10;0;0]),missionControl1.getGlobalCoordinates([-5;0;0])];
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','b');
    handles = [handles,h];
    line =[missionControl3.getGlobalCoordinates([20;0;0]),missionControl3.getGlobalCoordinates([-5;0;0])];
    h=plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','m');
    handles = [handles,h];
    line = missionControl1.vehicle.getActualPositionOrientation;
    h = plot3(line(1,:),line(2,:),line(3,:),'Marker', 'o','MarkerSize', 8, 'Color','r','MarkerFaceColor','r');
    handles = [handles,h];
    %line=[line(1:3) + [Cmnf.rot2D(270,[10;0]) ;0],line(1:3) + [Cmnf.rot2D(270,[-10;0]) ;0]];
    bline =[missionControl3.getGlobalCoordinates([0;-10;0]),missionControl3.getGlobalCoordinates([0;10;0])];
    pk=missionControl3.vehicle.getActualPositionOrientation - missionControl1.vehicle.getActualPositionOrientation;
    line = bline - (pk(1:3)*[1 1]);
    h= plot3(line(1,:),line(2,:),line(3,:),'LineWidth',2,'Color','r');
    handles = [handles,h];
    title('DecitionFrame 25.')
    Cmnf.exportFigure('UtmAdversaryPursuit',25);
    delete(handles);
    hold off
% plot sequence
    v1position=missionControl1.vehicle.getPositionalData;
    v2position=missionControl2.vehicle.getPositionalData;
    v3position=missionControl3.vehicle.getPositionalData;
    missionTime=missionControl1.vehicle.state.time;
    [m1,n1]=size(v1position);
    [m2,n2]=size(v3position);
    nCount=min(n1,n2);
    %crashDistance = sqrt([1 1 1]*(v1position(:,1:nCount)-v2position(:,1:nCount)).^2);
    crashDistanceReal = sqrt([1 1 1]*(v1position(:,1:nCount)-v3position(:,1:nCount)).^2);
    figure(2)
        grid on
        hold on
        %plot(missionTime(1:nCount),crashDistance,'b')
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
        axis([missionTime(1),missionTime(nCount),0,max(crashDistanceReal)])
        legend('Pursuit distance (t)','Safe Margin (t)','Body Margin (t)')
    Cmnf.exportFigure('UtmAdversaryPursuitPerformance');
%% Trajectory tracking performance
    %figure(3)
    %missionControl1.plotRealvsPlanTrajectoryStatistics()
    %title('Path following performance  UAV1')
    %Cmnf.exportFigure('UtmCooperativeHeadOnUAV1PathFollowing');
    
    %figure(4)
    %missionControl3.plotRealvsPlanTrajectoryStatistics()
    %title('Path following performance  UAV2')
    %Cmnf.exportFigure('UtmCooperativeHeadOnUAV2PathFollowing');