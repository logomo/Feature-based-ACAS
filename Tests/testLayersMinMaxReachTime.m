%% Layer entry/leave time calculation prototype
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag=AvoidanceGrid(0,20,-pi/4,pi/4,-pi/6,pi/6,20,7,5);
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);

layers=ag.layers;
llayers= length(layers);
lpass = zeros(2,llayers);
for k=1:llayers
    lpass(:,k)=[layers(k).minimalEntryTime;layers(k).maximalLeaveTime];
end

figure(1)
grid on
hold on
plot(1:llayers,lpass(1,:),'--ob')
plot(1:llayers,lpass(2,:),'--or')
plot(1:llayers,lpass(2,:)-lpass(1,:),'--g')
hold off
lgd=legend('Maximum leave time','Minimal entry time','Layer time');
lgd.Location='northwest';
xlabel('LayerID [index]')
ylabel('Time [s]')
title ('Layer entry/leave/spent time')

figure(2)
plot(1:llayers,lpass(2,:)-lpass(1,:),'--g')
grid on
xlabel('LayerID [index]')
ylabel('Time [s]')
title ('Layer spent time')

f=@()ag.reachSet.calculateProbability;
timeit(f)

