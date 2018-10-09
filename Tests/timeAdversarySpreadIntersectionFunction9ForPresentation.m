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
 ta.flagTimeIntersection = 0; %Enforce time
  ta.flagFutureMovements = 0;
 ta.flagBallIntersection = 0;
           ta.flagSpread = 1; %Use spread
       ta.flagOnlySpread = 1; %DO not Use line
          
ag.putTimedAdversarial(ta);

% run general recalculation procedure for avoidance grid
ag.recalculate

advTraj=[[6;8;0],[6;-7;0]];
rl=[advTraj(1:2,1),Cmnf.rot2D(pi/12,(advTraj(1:2,2)-advTraj(1:2,1))) + advTraj(1:2,1)];
rr=[advTraj(1:2,1),Cmnf.rot2D(-pi/12,(advTraj(1:2,2)-advTraj(1:2,1))) + advTraj(1:2,1)];
arc=[sin(linspace(-pi/12,pi/12,20));-cos(linspace(-pi/12,pi/12,20))]*norm(advTraj(1:2,1)-advTraj(1:2,2))+ advTraj(1:2,1)*ones(1,20);

% begin plot for
figure(1)
subplot(2,3,1)
ag.plotReachSetColored(StatisticType.Reachability)
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
plot3(rr(1,:),rr(2,:),[0,0],'b');
plot3(rl(1,:),rl(2,:),[0,0],'b');
plot3(arc(1,:),arc(2,:),zeros(1,20),'b');
hold off

subplot(2,3,2)
ag.plotHorizontalSlice(1,StatisticType.Reachability)
title('Layer 1 (top)')

subplot(2,3,3)
ag.plotHorizontalSlice(2,StatisticType.Reachability)
title('Layer 2 (top-middle)')

subplot(2,3,4)
ag.plotHorizontalSlice(3,StatisticType.Reachability)
title('Layer 3 (middle)')
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
plot3(rr(1,:),rr(2,:),[0,0],'b');
plot3(rl(1,:),rl(2,:),[0,0],'b');
plot3(arc(1,:),arc(2,:),zeros(1,20),'b');
hold off


subplot(2,3,5)
ag.plotHorizontalSlice(4,StatisticType.Reachability)
title('Layer 4 (bottom-middle)')

subplot(2,3,6)
ag.plotHorizontalSlice(5,StatisticType.Reachability)
title('Layer 5 (bottom)')

figure(2)
subplot(2,3,1)
ag.plotReachSetColored(StatisticType.Obstacle)
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
plot3(rr(1,:),rr(2,:),[0,0],'b');
plot3(rl(1,:),rl(2,:),[0,0],'b');
plot3(arc(1,:),arc(2,:),zeros(1,20),'b');
hold off
title('Obstacle probability (red-green)')

subplot(2,3,2)
ag.plotHorizontalSlice(1,StatisticType.Obstacle)
title('Layer 1 (top)')

subplot(2,3,3)
ag.plotHorizontalSlice(2,StatisticType.Obstacle)
title('Layer 2 (top-middle)')

subplot(2,3,4)
ag.plotHorizontalSlice(3,StatisticType.Obstacle)
hold on
plot3(advTraj(1,:),advTraj(2,:),advTraj(3,:),'b');
plot3(rr(1,:),rr(2,:),[0,0],'b');
plot3(rl(1,:),rl(2,:),[0,0],'b');
plot3(arc(1,:),arc(2,:),zeros(1,20),'b');
hold off
title('Layer 3 (middle)')

subplot(2,3,5)
ag.plotHorizontalSlice(4,StatisticType.Obstacle)
title('Layer 4 (bottom-middle)')

subplot(2,3,6)
ag.plotHorizontalSlice(5,StatisticType.Obstacle)
title('Layer 5 (bottom)')

ic=[];
for i=ta.spreadCells
    ic=[ic,i.position];
end
figure(3)
grid on
ag.plotRasterRange([0,0,0,0,0,0])
hold on
plot3(ic(1,:),ic(2,:),ic(3,:),'*r')
title('Intersection coverage')
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')


