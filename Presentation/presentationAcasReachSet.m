%%Constraindes expansion algorithm
lm=LinearizedModel;
%Avoidance grid
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
%ag.plotRasterRange([0,0,0,0,0,0])

%root build up
root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero movement
root.root=root;

%Set up separations (TODO Cmnf configuration item)
separations = [MovementGroup.Vertical,MovementGroup.Horizontal,MovementGroup.Slash,MovementGroup.Backslash];

%Expand that root, you know you want it!
root.expandACAS(ag,lm,separations);
% Wavefront to the grid
for k=1:10
    ag.layers(k).expandACAS(lm,separations);
    layerIndex=k;
    nodesCount=root.countNodes(0);
    routeCount=root.countNodes(1);
    leafs=root.collectLeafs;
    figure(layerIndex);
    ag.plotRasterRange([0,0,0,0,0,0])
    for l=1:length(leafs)
        hold on
        st=leafs(l).state;
        plot3(st(1,:),st(2,:),st(3,:))
        hold off
    end
    grid on 
    view(108.8000,31.7600);
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    title(['Layer: ',mat2str(layerIndex), ' Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])
    daspect([1 1 1])
	Cmnf.exportFigure('acasLikeRS',k);
end

% Test only final layer survival
root.reachFinalLayerTest(10);

% Obliterate
root.pruneBasedOnFinalLayer

% Obliterate last sread
root.pruneLastSpread(10)

% Recreate associations
ag.recreateAssociations(root);

% Recaluclate trajectory cost
root.calculateCost;

nodesCount=root.countNodes(0);
routeCount=root.countNodes(1);
leafs=root.collectLeafs;
figure(11)
ag.plotRasterRange([0,0,0,0,0,0])
for k=1:length(leafs)
    hold on
    st=leafs(k).state;
    plot3(st(1,:),st(2,:),st(3,:))
    hold off
end
grid on 
view(108.8000,31.7600);
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
title(['After Pruning, Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])
daspect([1 1 1])
Cmnf.exportFigure('acasLikeRSAfterPruning');