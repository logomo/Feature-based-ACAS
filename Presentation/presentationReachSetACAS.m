clear;
ag = AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
params = containers.Map;
%params('separations')=[MovementGroup.Horizontal,MovementGroup.Vertical,MovementGroup.Slash,MovementGroup.Backslash];
%params('separations')=[MovementGroup.Horizontal,MovementGroup.Vertical];
%params('separations')=[MovementGroup.Slash,MovementGroup.Backslash];
%params('separations')=[MovementGroup.Horizontal];
params('separations')=[MovementGroup.Vertical];
% first 
ReachSetCalculation.createReachSet(ag,ReachSetCalculation.ACASLike,params);
ag.plotReachSet