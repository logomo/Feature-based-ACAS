%Avoidance/Navigation grid 
gridDistance=10;
gridHorizontalRange=pi/4;
gridHorizontalCount=7;
gridVerticalRange=pi/6;
gridVerticalCount=5;
ag=AvoidanceGrid(0,gridDistance,-gridHorizontalRange,gridHorizontalRange,-gridVerticalRange,gridVerticalRange,10,gridHorizontalCount,gridVerticalCount);

%calculate standardized reach set
farCount=7;
nearCount=1;
ag.debug=1;
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);
ag.debug=0;
ag.resetGrid;

root = ag.reachSet;
lastLayer = 10;
hCellRange = [1 7];
vCellRange = [3 3];
trajectories = root.selectTrajectories(hCellRange,vCellRange);
sanityCheck= true;
for k=1:length(trajectories)
    sanityCheck=sanityCheck &&trajectories(k).checkTrajectoryRange(hCellRange,vCellRange);
end
register=containers.Map;
graphRoot=GraphNode(1,0,[0;0;0]);
register('root')=graphRoot;
list=[graphRoot];
id=1;
for k=1:length(trajectories)
    wTrajectory=trajectories(k);
    successor=0;
    while true
        % determine hash code
        if (wTrajectory ~= wTrajectory.root)
            hash=wTrajectory.mycell;
        else
            hash='root';
        end

        %check existance othervise create
        if ~register.isKey(hash)
           id=id+1;
           node = GraphNode(id,wTrajectory.referencedCell,wTrajectory.referencedCell.center);
           register(hash)=node;
           list =[list,node];
        end

        %load node
        node = register(hash);
        %
        if successor~=0
            node.registerLink(successor);
        end
        successor=node;

        %finish after processing trajectory
        if (wTrajectory == wTrajectory.root) 
            break;
        end

        %step back
        wTrajectory=wTrajectory.parrent;
    end
end
figure(1)
for k=1:length(trajectories)
    hold on 
    trajectories(k).plotTrajectoryWide('c')
    hold off
end
graphRoot.plotColoredGraph;
title('Extracted graph G from reach set slice')
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')

