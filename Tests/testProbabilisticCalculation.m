%% Raw development of reach set 
%Create testing grid
clear
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
% testing grid should be loaded in future, this is for approach sanity
% check

%put random obstacles on 5 layers with offset 3+, for 3x3 random horizontal
%xvertical cells 
for k=randi(7,1,5)+3 
    for l=randi(7,1,3)
        for m=randi(5,1,3)
            ag.layers(k).cells(l,m).pObstacle = 0.2 + rand*0.8;
        end
    end
end

% run general recalculation procedure for avoidance grid
ag.recalculate
% begin plot for

figure(4)
subplot(2,3,1)
ag.plotReachSetColored(StatisticType.Reachability)

subplot(2,3,2)
ag.plotHorizontalSlice(1,StatisticType.Reachability)
xlabel('x [m]')
ylabel('y [m]')
title('Layer 1 (top)')

subplot(2,3,3)
ag.plotHorizontalSlice(2,StatisticType.Reachability)
title('Layer 2 (top-middle)')

subplot(2,3,4)
ag.plotHorizontalSlice(3,StatisticType.Reachability)
title('Layer 3 (middle)')

subplot(2,3,5)
ag.plotHorizontalSlice(4,StatisticType.Reachability)
ylabel('y [m]')
title('Layer 4 (bottom-middle)')

subplot(2,3,6)
ag.plotHorizontalSlice(5,StatisticType.Reachability)
title('Layer 5 (bottom)')


