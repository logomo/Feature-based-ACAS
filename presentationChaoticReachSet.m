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
layerIndex=0;
lastLayerIndex =0;
while ~length(stack)==0
    worker = stack(1);
    if worker ~= root
        ll=[];
        for k=1:length(stack)
            l=stack(k).referencedCell.layerIndex;
            ll=[ll,l];
        end
        layerIndex=min(ll);
    end
    if layerIndex~=lastLayerIndex
        lastLayerIndex=layerIndex;
        nodesCount=root.countNodes(0);
        routeCount=root.countNodes(1);
        leafs=root.collectLeafs;
        figure(layerIndex);
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
        title(['Layer: ',mat2str(layerIndex), ' Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])
        daspect([1 1 1])
        Cmnf.exportFigure('CoverageRS',layerIndex);
    end
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
Cmnf.exportFigure('CoverageRSAfterPruning');
