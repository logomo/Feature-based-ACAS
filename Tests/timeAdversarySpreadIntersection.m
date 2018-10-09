clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);


adversaryPoint = [6;8;0];               %Adversary initial location
adversaryVelocity = [0;-1;0];           %Adversary initial velocity
%adversaryPoint = [10;0;0];
%adversaryVelocity = [-0.5;0;0];
adversaryTimeError = 1;                 %time error
adversaryRadius = 1;                    %radius of adversary
thetaSpread = pi/8;
phiSpread = pi/12;
%Create adversary object
ta=TimedAdversaryVehicle(adversaryPoint,adversaryVelocity,adversaryRadius,adversaryTimeError,thetaSpread,phiSpread);
ta.flagTimeIntersection=1;
adversarial=ta;
intersections=ta.findIntersectionEllipseCells(ag);
if adversarial.flagSpread
    adversaryLinearVelocity=norm(adversarial.velocity,2);                   
    adversaryTimeError=adversarial.distanceError;                               
    predictionError = adversaryLinearVelocity*adversaryTimeError;
    cells=adversarial.spreadCells;
    ta.spreadCells        
    for wCell=cells
        x=[];y=[];z=[];p=[];t=[];
        if (adversarial.flagTimeIntersection)
            cellArrivalTime = wCell.cell.minimalEntryTime;
            cellLeaveTime = wCell.cell.maximalLeaveTime;
            cellIntersection = wCell.cell.toleratedDeviation/adversaryLinearVelocity;
            minBoundary = cellArrivalTime - predictionError - cellIntersection;         % minimal cell entry boundary
            maxBoundary = cellLeaveTime + predictionError + cellIntersection;           % maximal cell leave boundary
            for i=1:length(wCell.time)
                arrivalTime = wCell.time(i);
                if arrivalTime >= minBoundary && maxBoundary>=arrivalTime
                    x=[x,wCell.position(1,i)];
                    y=[y,wCell.position(2,i)];
                    z=[z,wCell.position(3,i)];
                    p=[p,wCell.probability(i)];
                    t=[t,wCell.time(i)];
                end
            end
        else
            x=wCell.position(1,:);
            y=wCell.position(2,:);
            z=wCell.position(3,:);
            p=wCell.probability;
            t=wCell.time;
        end
        if ~isempty(p)
            pos=[mean(x);mean(y);mean(z)];
            p=mean(p);
            t=mean(t);
            av=Cmnf.getAdversaryVector(pos,p,t);
            wCell.cell.pAdversary=[wCell.cell.pAdversary,av];
        end
    end
end

% run general recalculation procedure for avoidance grid
ag.recalculate

% for sake of debug plot
ic=[];
for i=intersections
    ic=[ic,i.position];
end
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

figure(3)
grid on
ag.plotRasterRange([0,0,0,0,0,0])
hold on
plot3(ic(1,:),ic(2,:),ic(3,:),'*r')
title('Intersection coverage')
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
