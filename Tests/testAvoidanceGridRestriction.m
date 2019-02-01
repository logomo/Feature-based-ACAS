%% Simple avoidance grid restriction rule
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
node=root.leafs(3);
% Grab node local position orientation
nodeLocPosOr=node.getLocalPositionOrientation;
% 1st triplet in state is position (local)
nodeLocPos=nodeLocPosOr(1:3);
% 2nd triplet in state is velocity (local)
nodeLocOr=nodeLocPosOr(4:6);
% Global just rotate the local position of node to vehicle orientation
nodeGlobPos=mc.getGlobalCoordinates(nodeLocPos);
% Well velocity is not that easy
% X - right hand rotation base 
nodeLocVelocity=[vel;0;0];
% Apply vehicle rotation
vehicleVel=Cmnf.rot3D(or(1),or(2),or(3),nodeLocVelocity);
% Apply after movement rotation to get true heading
trajectoryVel=Cmnf.rot3D(nodeLocOr(1),nodeLocOr(2),nodeLocOr(3),vehicleVel);
% Add global position to get second point of vector
nodeGlobVelocity=nodeGlobPos+trajectoryVel;

% comparable safety margin
comSafetyMargin = norm(nodeGlobPos-collisionPoint);


% compare heading plane
compCol=collisionPoint - nodeGlobPos;
compVel=trajectoryVel; %=nodeGlobVelocity-nodeGlobPos;

% Normalize to vehicle heading,
compColNorm=Cmnf.rot3D(-nodeLocOr(1),-nodeLocOr(2),-nodeLocOr(3),compCol);
compColLoc=Cmnf.rot3D(-or(1),-or(2),-or(3),compColNorm);
% Create comparison
%   [x;y;z] y+ and y0 good for us,

flagDistanceFeasibility = comSafetyMargin >= ruleSafetyMargin;
flagHeadingFeasibility = compColLoc(2) >=0;


