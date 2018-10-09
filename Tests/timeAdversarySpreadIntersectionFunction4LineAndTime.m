clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);


adversaryPoint = [6;8;0];               %Adversary initial location
adversaryVelocity = [0;-1;0];           %Adversary initial velocity
adversaryTimeError = 1;                 %time error
adversaryRadius = 1;                    %radius of adversary
thetaSpread = pi/8;
phiSpread = pi/12;
%Create adversary object
ta=TimedAdversaryVehicle(adversaryPoint,adversaryVelocity,adversaryRadius,adversaryTimeError,thetaSpread,phiSpread);
 ta.flagTimeIntersection = 1; %Enforce time
  ta.flagFutureMovements = 0;
 ta.flagBallIntersection = 0;
           ta.flagSpread = 1; %Use spread
       ta.flagOnlySpread = 0; %Use line
          
ag.putTimedAdversarial(ta);

% run general recalculation procedure for avoidance grid
ag.recalculate


% begin plot for
figure(9)
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

figure(10)
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


