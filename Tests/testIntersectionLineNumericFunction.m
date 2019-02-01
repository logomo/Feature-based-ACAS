%% Line intersection numeric implementaiton prototype
clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
% fill in linear model maunally
ag.linearModel=LinearizedModel;

adversaryPoint = [10;0;0];
adversaryVelocity = [-1;0;0];

cells=ag.getLineIntersectionNumeric(adversaryPoint,adversaryVelocity);
f= @() ag.getLineIntersectionNumeric(adversaryPoint,adversaryVelocity);

for k = 1:length(cells)
    cells(k).cell.pObstacle = 1;
end
%  TODO process more line examles

ag.plotHorizontalSlice(3,StatisticType.Obstacle)
timeit(f)