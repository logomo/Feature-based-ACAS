navigationType=ReachSetCalculation.ACASLike;
navigationParams=containers.Map;
navigationParams('separations')=[MovementGroup.Horizontal];
avoidanceType=ReachSetCalculation.ACASLike;
avoidanceParams=containers.Map;
avoidanceParams('separations')=[MovementGroup.Horizontal];
plotwaypoint=[ 7.7501;-1.9419;0];

%test vehicle properties
    orientation= [0;0;0];
    position = [0;0;0];
    %test waypoints
    waypoints = [Waypoint(8,-2,0)];
    %Avoidance/Navigation grid 
    gridDistance=5;
    gridHorizontalRange=pi/4-0.3;
    gridHorizontalCount=6;
    gridVerticalRange=0.00000000000000000000000000000000000000000000000000000000000001;
    gridVerticalCount=1;
    
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleId=1;
    missionControl1.vehicleName='Srotmaster 2000';
    missionControl1.goal=waypoints.position;

    
    %Plot empty grid
    missionControl1.avoidanceGrid.plotHorizontalSlice(1,3)
    
    
                    
    % Obstacle cells
    missionControl1.avoidanceGrid.plotDisplayCell(2,6,1,'r')
    missionControl1.avoidanceGrid.plotDisplayCell(3,5,1,'r')
    missionControl1.avoidanceGrid.plotDisplayCell(3,4,1,'r')
    missionControl1.avoidanceGrid.plotDisplayCell(5,3,1,'r')
    missionControl1.avoidanceGrid.plotDisplayCell(5,1,1,'r')
    
    %Unseen Portion
    missionControl1.avoidanceGrid.plotDisplayCell(3:5,6,1,'y')
    missionControl1.avoidanceGrid.plotDisplayCell(4:5,5,1,'y')
    missionControl1.avoidanceGrid.plotDisplayCell(4:5,4,1,'y')

    
    % Reach set projection
    missionControl1.avoidanceGrid.recalculate();
    %missionControl1.avoidanceGrid.plotReachSetColored2()
    
    %unreacable
    orange = [256 103 2]./256;
    missionControl1.avoidanceGrid.plotDisplayCell(1:2,1,1,orange);
    missionControl1.avoidanceGrid.plotDisplayCell(1,2,1,orange);
    missionControl1.avoidanceGrid.plotDisplayCell(1,5:6,1,orange);
   
    %Reachable
    missionControl1.avoidanceGrid.plotDisplayCell(1,3:4,1,'g');
    missionControl1.avoidanceGrid.plotDisplayCell(2,2:5,1,'g');
    missionControl1.avoidanceGrid.plotDisplayCell(3,1:3,1,'g');
    missionControl1.avoidanceGrid.plotDisplayCell(4,1:3,1,'g');
    missionControl1.avoidanceGrid.plotDisplayCell(5,2,1,'g');
    
    
    % Best Trajectory
    [bestTrajectory,waypointUnreachableFlag,collisionFlag]=missionControl1.findBestPath(missionControl1.avoidanceGrid);
    missionControl1.vehicle.forceSimulinkModel=true;
    missionControl1.vehicle.flyBuffer(bestTrajectory.collectMovements)
    missionControl1.vehicle.forceSimulinkModel=false;
    hold on 
    %bestTrajectory.plotTrajectoryWide('c')
    missionControl1.vehicle.plotTrajectoryWide('b')
    hold off
    
    %Future Trajectory
    posor =missionControl1.vehicle.getActualPositionOrientation;
    missionControl2=MissionControl(waypoints,posor(1:3),posor(4:6),gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl2.vehicleId=1;
    missionControl2.vehicleName='Srotmaster 2000';
    missionControl2.goal=waypoints.position;
    
    missionControl3=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    
    
    [bestTrajectory2,waypointUnreachableFlag,collisionFlag]=missionControl2.findBestPath(missionControl2.avoidanceGrid);
    missionControl3.vehicle.forceSimulinkModel=true;
    missionControl3.vehicle.flyBuffer([bestTrajectory.collectMovements,bestTrajectory2.collectMovements])
    missionControl3.vehicle.forceSimulinkModel=false;
    hold on
    missionControl3.vehicle.plotTrajectoryWide('c')
    missionControl1.vehicle.plotTrajectoryWide('b')
    hold off
    
    %plot Points
    hold on
    plot(plotwaypoint(1),plotwaypoint(2), 'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
    plot(0,0, 'Marker','square','MarkerFaceColor','b','MarkerEdgeColor','r','MarkerSize',15);
    plot(2.94,-0.653, 'Marker','p','MarkerFaceColor','c','MarkerEdgeColor','m','MarkerSize',15);
    plot(4.852,-1.165, 'Marker','p','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',15);
    hold off
    
    
    % Legend
    hold on
    %Points
    pp0 = plot(NaN,NaN,'Marker','none','MarkerFaceColor','w','MarkerEdgeColor','w','MarkerSize',15,'LineStyle','none');
    pp1 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15,'LineStyle','none');
    pp2 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','b','MarkerEdgeColor','r','MarkerSize',15,'LineStyle','none');
    pp3 = plot(NaN,NaN,'Marker','p','MarkerFaceColor','c','MarkerEdgeColor','m','MarkerSize',15,'LineStyle','none');
    pp4 = plot(NaN,NaN,'Marker','p','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',15,'LineStyle','none');
    %Cells
    cc0 = plot(NaN,NaN,'Marker','none','MarkerFaceColor','w','MarkerEdgeColor','w','MarkerSize',15,'LineStyle','none');
    cc1 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','w','MarkerEdgeColor','k','MarkerSize',15,'LineStyle','none');
    cc2 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',15,'LineStyle','none');
    cc3 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',15,'LineStyle','none');
    cc4 = plot(NaN,NaN,'Marker','square','MarkerFaceColor',orange,'MarkerEdgeColor','k','MarkerSize',15,'LineStyle','none');
    cc5 = plot(NaN,NaN,'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','k','MarkerSize',15,'LineStyle','none');
    
    tt0 = plot(NaN,NaN,'Marker','none','MarkerFaceColor','w','MarkerEdgeColor','w','MarkerSize',15,'LineStyle','none');
    tt1 = plot(NaN,NaN,'LineStyle','-','Color','g','LineWidth',4);
    tt2 = plot(NaN,NaN,'LineStyle','-','Color','c','LineWidth',4);
    tt3 = plot(NaN,NaN,'LineStyle','-','Color','b','LineWidth',4);
    
    hold off
    h=[pp0, pp1,pp2,pp3,pp4,cc0,cc1,cc2,cc3,cc4,cc5,tt0,tt3,tt2];
    legend(h, 'Points of Interest:',...
                    '- Goal',...
                    '- Initial position',...
                    '- Next decision point',...
                    '- Avoidance Goal',...
              'Cell status:',...
                    '- Free',...
                    '- Obstacle',...
                    '- Unknown',...
                    '- Unreachable',...
                    '- Reachable',...
              'Trajectories:',...
                    '- Planned',...
                    '- Future');
   
   %Sizing
   title('Future decisions')
   daspect([1,1,1])
   axis off
   set(gcf, 'Position',[848   523   732   420])
   Cmnf.exportFigure('AvoidanceRun-07-FutureDecisions')
    