clear;
%Create avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
% fill in linear model maunally
ag.linearModel=LinearizedModel;

adversaryPoint = [10;0;0];
adversaryVelocity = [-1;0;0];

cells=ag.getLineIntersectionNumeric(adversaryPoint,adversaryVelocity);
initialCell = cells(6).cell;
point =initialCell.center;
range = 1;

r=Cmnf.findCellsInRange(ag,initialCell,point,range);
f=@() Cmnf.findCellsInRange(ag,initialCell,point,range);
timeit(f)
%TODO add ranges to various tiles + tolerance offseting for each cell in
%layer
