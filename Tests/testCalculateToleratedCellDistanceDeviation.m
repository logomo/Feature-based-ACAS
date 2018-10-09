clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
deviations=ag.calculateToleratedDistanceDeviation;

figure(1)
plot(1:ag.countLayers,deviations,'-or');
title('Cell center to cell boundary maximal deviation')
xlabel('Layer [index]')
ylabel('Distance [m]')
