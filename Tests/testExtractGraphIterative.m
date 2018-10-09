%Avoidance/Navigation grid 
gridDistance=10;
gridHorizontalRange=pi/4;
gridHorizontalCount=7;
gridVerticalRange=pi/6;
gridVerticalCount=5;
ag=AvoidanceGrid(0,gridDistance,-gridHorizontalRange,gridHorizontalRange,-gridVerticalRange,gridVerticalRange,10,gridHorizontalCount,gridVerticalCount);

%calculate standardized reach set
farCount=7;
nearCount=1;
ag.debug=1;
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
ag.debug=0;
ag.resetGrid;
%create standard graph
rsg=ReachSetGraph(ag);

%create obstacle and put into avoidance grid
os1=ObstacleSphere([6;0;0],1.5,ObstacleType.Detected);
rsg.putObstacle(os1);

%plot Standard graphs
figure(1)
rsg.plot(StatisticType.Obstacle)
hold on 
os1.plot
hold off

figure(2)
rsg.plot(StatisticType.Visibility)
hold on 
os1.plot
hold off

figure(3)
rsg.plot(StatisticType.Reachability)
hold on 
os1.plot
hold off

%Find paths
[costs,paths]=rsg.findPaths;

%generate trajectories
trajectories=rsg.generateTrajectories(costs,paths);

%create trajectory register for avoidance grid
trajectoryRegister = ag.reachSet.getTrajectoryRegister;

%assess feasibility of execution
[unfeasibleIDs,feasibleIDs,feasibilityMap]=rsg.assessExistance(trajectoryRegister);
reachableIDs = find(costs <= 1000);
executableIDs = intersect(reachableIDs,feasibleIDs);
inexecutableIDs = intersect(reachableIDs,unfeasibleIDs);


ag.resetGrid;
for k=1:length(executableIDs)
    execTraj = rsg.trajectories(executableIDs(k));
    execPaths = feasibilityMap(executableIDs(k));
    figure(4+k)
    rsg.plot(StatisticType.Reachability)
    hold on
    for exp=execPaths
        exp.plotTrajectoryWide;
    end
    execTraj.plot('m');
    title(['Djikstra trajectory ID:',mat2str(execTraj.id),'with goal cell [l,h,v], ',mat2str(execTraj.goalCell.ijkIndex)]);
    hold off
end


for l=1:length(inexecutableIDs)
    execTraj = rsg.trajectories(inexecutableIDs(l));
    figure(4+k+l)
    rsg.plot(StatisticType.Reachability)
    hold on
    execTraj.plot('r');
    title(['Djikstra trajectory ID:',mat2str(execTraj.id),'with goal cell [l,h,v], ',mat2str(execTraj.goalCell.ijkIndex)]);
    hold off
end

