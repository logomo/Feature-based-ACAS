%% Test Acas-x like reach set creation

ag = AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
params = containers.Map;
params('separations')=[MovementGroup.Horizontal,MovementGroup.Vertical,MovementGroup.Slash,MovementGroup.Backslash];
% first 
ReachSetCalculation.createReachSet(ag,ReachSetCalculation.ACASLike,params);
ag.plotReachSet