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
       ta.flagOnlySpread = 0; %DO not Use line intersection
obj.avoidanceGrid.resetGrid
obj.avoidanceGrid.putTimedAdversarial(ta)
obj.avoidanceGrid.recalculate
obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Reachability,2)
obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Obstacle,3)
obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Visibility,4)