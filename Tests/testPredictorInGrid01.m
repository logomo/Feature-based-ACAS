%% Hash tree search based on limitation of passing cells
clear
lm=LinearizedModel;
%Avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
%ag.plotRasterRange([0,0,0,0,0,0])
%Trajectory counter
trajectoryRegister = containers.Map;
tMax = 7;
%root build up
root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero movement
root.root=root;

%build full map
%root.expand(ag,lm,trajectoryRegister,tMax);
stack = [root];
while ~length(stack)==0
    worker = stack(1);
    stack(1)=[];
    stack=[stack,worker.expand(ag,lm,trajectoryRegister,tMax)];
end
root.reachFinalLayerTest(10);
root.pruneBasedOnFinalLayer
root.pruneLastSpread(10)
ag.recreateAssociations(root);
ag.countAssociations;


nodesCount=root.countNodes(0);
routeCount=root.countNodes(1);
leafs=root.collectLeafs;
figure(1)
ag.plotRasterRange([0,0,0,0,0,0])
for k=1:length(leafs)
    hold on
    st=leafs(k).state;
    plot3(st(1,:),st(2,:),st(3,:))
    hold off
end

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
title(['Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])