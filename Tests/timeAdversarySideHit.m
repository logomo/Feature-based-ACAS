%% Test for adversary performance early penertation - scenario
clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);


adversaryPoint = [6;8;0];
adversaryVelocity = [0;-1;0];
%adversaryPoint = [10;0;0];
%adversaryVelocity = [-0.5;0;0];
adversaryTimeError = 0;
adversaryRadius = 1;

cells=ag.getLineIntersectionNumeric(adversaryPoint,adversaryVelocity);
f= @() ag.getLineIntersectionNumeric(adversaryPoint,adversaryVelocity);
timeit(f)


%calculate Adversary
adversaryLinearVelocity=norm(adversaryVelocity,2);
predictionError = adversaryLinearVelocity*adversaryTimeError;
intersectionCells=[];
futureMovementCells=[];
for k=1:length(cells);
    wCell = cells(k);
    arrivalTime = wCell.time;
    cellArrivalTime = wCell.cell.minimalEntryTime;
    cellLeaveTime = wCell.cell.maximalLeaveTime;
    cellIntersection = wCell.cell.toleratedDeviation/adversaryLinearVelocity;
    minBoundary = cellArrivalTime - predictionError - cellIntersection;
    maxBoundary = cellLeaveTime + predictionError + cellIntersection;
    maxLeave = cellLeaveTime - predictionError ;
    if arrivalTime >= minBoundary && maxBoundary>=arrivalTime
        if (Cmnf.debugAdversary)
            intersection=[minBoundary,arrivalTime,maxBoundary,k]
        end
        intersectionCells=[intersectionCells,wCell];
    end
    
    if arrivalTime >= maxLeave
        if (Cmnf.debugAdversary)
            futureTrajectory=[arrivalTime,maxLeave,k]
        end
        futureMovementCells=[futureMovementCells,wCell];
    end
end

%let us expand intersection to range
%Cmnf.harmonicDistribution(60,70,70)

for wCell = intersectionCells
    candidates = Cmnf.findCellsInRange(ag,wCell.cell,wCell.position,adversaryRadius);
    intAdvVec=Cmnf.getAdversaryVector(wCell.position,1,wCell.time);
    wCell.cell.pAdversary=[wCell.cell.pAdversary,intAdvVec];
    for c=candidates
        if  c.isDirect
            advVec = Cmnf.getAdversaryVector(wCell.position,1,wCell.time);
            c.cell.pAdversary = [c.cell.pAdversary,advVec];
        else
            max=c.cell.toleratedDeviation + c.checkDistance;
            min=c.checkDistance;
            value=c.realDistance;
            probability=Cmnf.harmonicDistribution(min,max,value);
            advVec = Cmnf.getAdversaryVector(wCell.position,probability,wCell.time);
            c.cell.pAdversary = [c.cell.pAdversary,advVec];
        end
        if (Cmnf.debugAdversary)
            c.cell.ijkIndex
            c.cell.pAdversary
        end
    end
end


% future movement cell probability calculation
if ~isempty(intersectionCells)
    for wCell=futureMovementCells 
        entryTime = wCell.cell.minimalEntryTime;
        leavleTime = wCell.cell.maximalLeaveTime;
        base = wCell.time;
        cellLeaveTime = wCell.cell.maximalLeaveTime;
        gain = cellLeaveTime - predictionError;
        probability =  gain/base;
        futAdvec=Cmnf.getAdversaryVector(wCell.position,probability,wCell.time);
        wCell.cell.pAdversary=[wCell.cell.pAdversary,futAdvec];
        wCell.cell.ijkIndex
    end
end
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