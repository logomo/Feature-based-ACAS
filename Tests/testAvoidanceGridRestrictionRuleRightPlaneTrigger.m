%% Rule right plane heading trigger raw script 
utm=utmControl;
cc=utm.createCollisionCase(1,2);
firstId=cc.firstId;
secondId=cc.secondId;
firstMissionControl = cc.firstMissionControl;
secondMissionControl = cc.secondMissionControl;

ruleSafetyMargin=cc.ruleSafetyMargin;

%% Rule body starts
mc=secondMissionControl;
ag=mc.avoidanceGrid;
vel=mc.vehicleActualVelocity;
root=ag.reachSet;
posOr=mc.vehicle.getActualPositionOrientation;
pos=posOr(1:3);
or=posOr(4:6);
collisionPoint=cc.collisionPoint;

%% Node application function
% test node - move to the right
%node=root.leafs(3);
%[flagDistanceFeasibility,flagHeadingFeasibility] = node.calculateFlagsNode(mc,vel,pos,or,collisionPoint,ruleSafetyMargin);
nodePassed=root.calculateOperatibleSpace(mc,vel,...
                pos,or,collisionPoint,...
                ruleSafetyMargin)
            
%% Rule application
nodeDisabled = root.applyOperatibleSpace;

%% Just dispay functions
ag.recalculate;
mc.plotGridSlice(mc.avoidanceGrid,StatisticType.Reachability,2)

            
            
            