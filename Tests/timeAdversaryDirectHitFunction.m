%% Test for time based adversary 
clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
farCount = 8;   % spread 8 trajectories far away from center
nearCount = 1;  % spread 1 trajectory close to cell center
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);


%adversaryPoint = [6;8;0];               
%adversaryVelocity = [0;-1;0];           
adversaryPoint = [10;0;0];              %Adversary initial location
adversaryVelocity = [-0.5;0;0];         %Adversary initial velocity
adversaryTimeError = 0;                 %time error
adversaryRadius = 1;                    %radius of adversary
%Create adversary object
ta=TimedAdversaryVehicle(adversaryPoint,adversaryVelocity,adversaryRadius,adversaryTimeError);
%Set adversary functions
    ta.flagTimeIntersection= 1;     %Using time based intersection or just line intersection
    ta.flagFutureMovements= 1;      %Marking dangerous future movements ?
    ta.flagBallIntersection= 0;     %Marking ball intersection in timed movements ?
    ta.flagSpread=0;                %MArking possible spread of timed intersections
cnt=ta.findIntersection(ag);        %find intersections with avoidance grid
points = ag.putTimedAdversarial(ta); % put timed adversarial into avoidance grid
f=@() ta.findIntersection(ag);       % measure time of f
g=@() ag.putTimedAdversarial(ta);    % measure time of f
timeit(f)
timeit(g)

% run general recalculation procedure for avoidance grid
ag.recalculate
% begin plot for
figure(1)
subplot(2,3,1)
ag.plotReachSetColored(StatisticType.Reachability)

subplot(2,3,2)
ag.plotHorizontalSlice(1,StatisticType.Reachability)
title('Layer 1 (top)')

subplot(2,3,3)
ag.plotHorizontalSlice(2,StatisticType.Reachability)
title('Layer 2 (top-middle)')

subplot(2,3,4)
ag.plotHorizontalSlice(3,StatisticType.Reachability)
title('Layer 3 (middle)')

subplot(2,3,5)
ag.plotHorizontalSlice(4,StatisticType.Reachability)
title('Layer 4 (bottom-middle)')

subplot(2,3,6)
ag.plotHorizontalSlice(5,StatisticType.Reachability)
title('Layer 5 (bottom)')

figure(2)
subplot(2,3,1)
ag.plotReachSetColored(StatisticType.Obstacle)

subplot(2,3,2)
ag.plotHorizontalSlice(1,StatisticType.Obstacle)
title('Layer 1 (top)')

subplot(2,3,3)
ag.plotHorizontalSlice(2,StatisticType.Obstacle)
title('Layer 2 (top-middle)')

subplot(2,3,4)
ag.plotHorizontalSlice(3,StatisticType.Obstacle)
title('Layer 3 (middle)')

subplot(2,3,5)
ag.plotHorizontalSlice(4,StatisticType.Obstacle)
title('Layer 4 (bottom-middle)')

subplot(2,3,6)
ag.plotHorizontalSlice(5,StatisticType.Obstacle)
title('Layer 5 (bottom)')