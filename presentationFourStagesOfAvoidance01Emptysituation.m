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
    
    
                    
    
    %plot Points
    hold on
    plot(plotwaypoint(1),plotwaypoint(2), 'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
    plot(0,0, 'Marker','square','MarkerFaceColor','b','MarkerEdgeColor','r','MarkerSize',15);
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
    
    hold off
    h=[pp0, pp1,pp2,cc0,cc1];
    legend(h, 'Points of Interest:',...
                    '- Goal',...
                    '- Initial position',...
              'Cell status:',...
                    '- Unknown');
   hold off
   
   %Sizing
   title('The initial situation')
   daspect([1,1,1])
   set(gcf, 'Position',[848   523   732   420])
    axis off
    Cmnf.exportFigure('AvoidanceRun-01-Initial-Situation')
   
    