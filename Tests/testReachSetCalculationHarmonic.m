%% Test turn minimizing Reach set approximation creation
ag = AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
% first 
ReachSetCalculation.createReachSet(ag,ReachSetCalculation.Harmonic);