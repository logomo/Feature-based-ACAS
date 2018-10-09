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
           ta.flagSpread = 0; %Use spread
       ta.flagOnlySpread = 0; %Use line
          
ag.putTimedAdversarial(ta);

% run general recalculation procedure for avoidance grid
ag.recalculate

advTraj=[[6;8;0],[6;-8;0]];

% begin plot for
figure(1)
subplot(1,2,1)
ag.plotReachSetColored(StatisticType.Reachability)
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
hold off

subplot(1,2,2)
ag.plotHorizontalSlice(3,StatisticType.Reachability)
title('Layer 3 (middle)')
hold on 
plot(advTraj(1,:),advTraj(2,:),'b');
hold off


figure(2)
subplot(1,2,1)
ag.plotReachSetColored(StatisticType.Obstacle)
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
hold off
title('Obstacle probability (red-green)')

subplot(1,2,2)
ag.plotHorizontalSlice(3,StatisticType.Obstacle)
hold on 
plot(advTraj(1,:),advTraj(2,:),'b');
hold off
title('Layer 3 (middle)')



