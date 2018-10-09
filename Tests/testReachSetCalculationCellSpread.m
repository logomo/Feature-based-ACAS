ag = AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
params = containers.Map;
params('farCount') = 8;
params('nearCount') = 1;
% first 
ReachSetCalculation.createReachSet(ag,ReachSetCalculation.CellSpread,params);