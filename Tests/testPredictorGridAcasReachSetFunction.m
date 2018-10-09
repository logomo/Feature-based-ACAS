%%Constraindes expansion algorithm
lm=LinearizedModel;

%Set up separations (TODO Cmnf configuration item)
separations = [MovementGroup.Horizontal , MovementGroup.Vertical, MovementGroup.Slash, MovementGroup.Backslash];

%Avoidance grid 15 layers
ag=AvoidanceGrid(0,15,-pi/4,pi/4,-pi/6,pi/6,15,7,5);

ag.precalculateACASReachSet(lm,separations)


nodesCount=ag.reachSet.countNodes(0);
routeCount=ag.reachSet.countNodes(1);

leafs=ag.reachSet.collectLeafs;
figure(1)
ag.plotRasterRange([0,0,0,0,0,0])
for k=1:length(leafs)
    hold on
    st=leafs(k).state;
    plot3(st(1,:),st(2,:),st(3,:))
    hold off
end
grid on
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
title(['Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])