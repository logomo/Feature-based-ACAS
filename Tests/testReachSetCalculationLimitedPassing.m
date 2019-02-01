%% Test limited passing reach set creation
ag = AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
params = containers.Map;
params('passRatio')=7;
% first 
ReachSetCalculation.createReachSet(ag,ReachSetCalculation.LimitedPassing,params);