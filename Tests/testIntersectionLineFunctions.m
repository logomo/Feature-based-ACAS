%% Line intersection symbolic implementaiton prototype
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);

point = [0;0;0];
velocity = [1;0.2;0.3];
ccs=ag.getLineIntersection(point,velocity);
f = @()ag.getLineIntersection(point,velocity);
for k = 1:length(ccs)
    ccs(k).cell.pObstacle = 1;
end

figure(1)
subplot(5,1,1)
ag.plotHorizontalSlice(1,StatisticType.Obstacle)
subplot(5,1,2)
ag.plotHorizontalSlice(2,StatisticType.Obstacle)
subplot(5,1,3)
ag.plotHorizontalSlice(3,StatisticType.Obstacle)
subplot(5,1,4)
ag.plotHorizontalSlice(4,StatisticType.Obstacle)
subplot(5,1,5)
ag.plotHorizontalSlice(5,StatisticType.Obstacle)
timeit(f)