%% Test performance of 3 different tree search algorithms
clear all
% create avoidance grid for Harmonic spread method
ag1=AvoidanceGrid(0,15,-pi/4,pi/4,-pi/6,pi/6,15,7,5);
ag1.precalculateHarmonicReachSet(LinearizedModel());
f1=@()ag1.precalculateHarmonicReachSet(LinearizedModel());
figure(1)
ag1.plotReachSet

%create avoidance grid for cell spread algorithm - seems as most feasible
%method, because it generates iregular trajectories ...
farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag2=AvoidanceGrid(0,15,-pi/4,pi/4,-pi/6,pi/6,15,7,5);
ag2.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
f2=@()ag2.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
figure(2)
ag2.plotReachSet

%create avoidance grid limited passing algorithm
%generates chaotic routes, does not guarantee versatibility
passRatio=7;    % up to 7 trajectories with same footprint
ag3=AvoidanceGrid(0,15,-pi/4,pi/4,-pi/6,pi/6,15,7,5);
ag3.precalculateLimitedPassingReachSet(LinearizedModel(),passRatio);
f3=@()ag3.precalculateLimitedPassingReachSet(LinearizedModel(),passRatio);
figure(3)
ag3.plotReachSet

%% Calculation timer
[timeit(f1),timeit(f2),timeit(f3)]
