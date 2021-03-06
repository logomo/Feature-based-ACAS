%% Create UTM + situation awarness
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
    position = [0;0;0];

    %Avoidance/Navigation grid 
    gridDistance=10;
    gridHorizontalRange=pi/4;
    gridHorizontalCount=7;
    gridVerticalRange=pi/30;
    gridVerticalCount=1;


    %Create mission control object
    missionControl1=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams);
    missionControl1.vehicleName='SRotor-2000';
    



%% Rule body starts
mc=missionControl1;
ag=mc.avoidanceGrid;
vel=mc.vehicleActualVelocity;
root=ag.reachSet;
posOr=mc.vehicle.getActualPositionOrientation;
pos=posOr(1:3);
or=posOr(4:6);
for k=1:7
ag.resetGrid
baseangle=pi/2/7;
collisionPoint=[cos((4-k)*baseangle);sin((4-k)*baseangle);0];
collisionPoint=(collisionPoint./norm(collisionPoint)).*11;

%% Node application function
% test node - move to the right
%node=root.leafs(3);
%[flagDistanceFeasibility,flagHeadingFeasibility] = node.calculateFlagsNode(mc,vel,pos,or,collisionPoint,ruleSafetyMargin);
nodePassed=root.calculateOperatibleSpace(mc,vel,...
                pos,or,collisionPoint,...
                0)
            
%% Rule application
nodeDisabled = root.applyOperatibleSpace;
ag.recalculate;
mc.plotGridSlice(mc.avoidanceGrid,StatisticType.Reachability,k)

%% plot collision point
subplot(1,2,1)
hold on
title('')
plot3(11.5,0,0,'.w')
plot3([0,collisionPoint(1)],[0,collisionPoint(2)],[0,collisionPoint(3)],'-.m')
plot3(collisionPoint(1),collisionPoint(2),collisionPoint(3),'om','MarkerFaceColor','m')
set(gca, 'xticklabel', []);
set(gca, 'yticklabel', []);
xlabel('');
ylabel('');
axis off
grid off
hold off

subplot(1,2,2)
hold on
title('')
plot(11.5,0,'.w')
plot([0,collisionPoint(1)],[0,collisionPoint(2)],'-.m')
plot(collisionPoint(1),collisionPoint(2),'om','MarkerFaceColor','m')
set(gca, 'xticklabel', []);
set(gca, 'yticklabel', []);
xlabel('');
ylabel('');
axis off
grid off
hold off

end