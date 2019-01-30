obj=utmControl;
fid=1;
sid=2;
fmc= obj.missionControls(fid);
smc= obj.missionControls(sid);
Cmnf.logc(obj,['Testing collision ID: ',mat2str(fmc.vehicleId),' ',fmc.vehicleName,' agains ', 'ID: ',mat2str(smc.vehicleId),' ',smc.vehicleName]);

% test if vehicles are active
if ~(fmc.isActive && smc.isActive)
    Cmnf.logc(obj,'-- Failed -- One or more vehicles are inactive');
    r=-1;
    return;
else
    Cmnf.logc(obj,'-- Passed -- Vehicles are active');
end

%Load their last positions
fpm=obj.positionNotifications(fid);
spm=obj.positionNotifications(sid);

%Check flight level
if fpm.mainFlightLevel == spm.mainFlightLevel ||... 
   fpm.mainFlightLevel == spm.passingFlightLevel || ...
   spm.fpm.mainFlightLevel ==  fpm.passingFlightLevel
    Cmnf.logc(obj,'-- Passed -- Detected crossing flight levels');
else
    Cmnf.logc(obj,'-- Failed -- No crossing in flight levels');
    r=-2;
end

%Initialize safety margin
collisionCategory=CollisionCategory.Unknown;
fsm=fpm.getSafetyMargin(CollisionCategory.Unknown);
ssm=spm.getSafetyMargin(CollisionCategory.Unknown);
defaultSafetyMargin=max(fsm,ssm);
Cmnf.logc(obj,['Safetymargin ',fmc.vehicleName,': ',mat2str(fsm),' m, ', smc.vehicleName, ': ',mat2str(ssm),' m']);
Cmnf.logc(obj,['-- SafetyMargin -- set as ',mat2str(defaultSafetyMargin),' m']);

%Trajectory prediction test
ftraj=fmc.predictTrajectory;
straj=smc.predictTrajectory;
distanceVector=sqrt([1,1,1]*((ftraj - straj).^2));
conditionVector = distanceVector < defaultSafetyMargin;
Cmnf.logc(obj,['-- ProjectedTrajectory -- distance vector:  ',mat2str(distanceVector)]);
Cmnf.logc(obj,['-- ProjectedTrajectory -- condition vector: ',mat2str(conditionVector)]);
collisionFlag = false;
if sum(conditionVector) > 0
    Cmnf.logc(obj,['-- ProjectedTrajectory -- collision detected, raising flag']);
    collisionFlag = true;
end

% determine initial roles
frole=UavIntruderRole.Emergencing;
srole=UavIntruderRole.Emergencing;
emergencyFlag=true;
% vehicle with higher category has the right of the way
if fpm.vehicleCategory ~= spm.vehicleCategory
    Cmnf.logc(obj,['-- ClassComp -- Vehicle Category mismatch ',...
        mat2str(fpm.vehicleCategory),' vs. ',mat2str(spm.vehicleCategory)]);
    if fpm.vehicleCategory < spm.vehicleCategory
        frole=UavIntruderRole.RightOfTheWay;
        srole=UavIntruderRole.Emergencing;
        Cmnf.logc(obj,['-- ClassComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
    end
    if fpm.vehicleCategory > spm.vehicleCategory
        frole=UavIntruderRole.Emergencing;
        srole=UavIntruderRole.RightOfTheWay;
        Cmnf.logc(obj,['-- ClassComp -- Vehicle ',smc.vehicleName,' has the right of way']);
    end
else
    Cmnf.logc(obj,['-- ClassComp -- Vehicle Category match ',...
        mat2str(fpm.vehicleCategory),' vs. ',mat2str(spm.vehicleCategory)]);
    %Vehicle with higher manuevurabilty has higher priority
    if fpm.manCat ~= spm.manCat
         Cmnf.logc(obj,['-- ManCatComp -- Manuevaribility category miss match ',...
             mat2str(fpm.manCat),' vs. ',mat2str(spm.manCat)]);
         if fpm.manCat < spm.manCat
             frole=UavIntruderRole.RightOfTheWay;
             srole=UavIntruderRole.Emergencing;
             Cmnf.logc(obj,['-- ManCatComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
         end
         if fpm.manCat > spm.manCat
             frole=UavIntruderRole.Emergencing;
             srole=UavIntruderRole.RightOfTheWay;
             Cmnf.logc(obj,['-- ManCatComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
         end
    else
        Cmnf.logc(obj,['-- ManCatComp -- Manuevaribility category match ',...
             mat2str(fpm.manCat),' vs. ',mat2str(spm.manCat)]);
        emergencyFlag=false;
        Cmnf.logc(obj,'-- Emergency Mode -- deactivated, starting cooperatove avoidance');
    end
end
% Now is time to measure who is greater


%Linear intersection test
fpos=fpm.getPosition; % first vehicle position
fvel=fpm.velocity;    % first vehicle velocity
spos=spm.getPosition; % second vehicle position
svel=spm.velocity;    % second vehicle velocity 
[lineFlag,intersectionPoint,intersectionTime,intersectionDistance]=...
    Cmnf.calculateLinearClash(fpos,fvel,spos,svel);
Cmnf.logc(obj,['-- LinearProjection -- flag         :',mat2str(lineFlag)]);
Cmnf.logc(obj,['-- LinearProjection -- intersection :',mat2str(intersectionPoint),' [m,m,m]']);
Cmnf.logc(obj,['-- LinearProjection -- time to crash:',mat2str(intersectionTime), ' s']);
Cmnf.logc(obj,['-- LinearProjection -- min distance :',mat2str(intersectionDistance),' m']);

%Now we have a situation
if lineFlag && intersectionDistance< defaultSafetyMargin
    Cmnf.logc(obj,['-- Rules of the Air -- start']);
    
    %Calculate angle of approach
    fnor=fpos-intersectionPoint; % normalize first position to intersection point
    snor=spos-intersectionPoint; % normalize second position to intersection point
    angles=Cmnf.minimalAnglesBetweenVectors(fnor,snor);
    angleOfApproach=angles(3);
    Cmnf.logc(obj,['                    -- first normalized pos:  ',mat2str(fnor)]);
    Cmnf.logc(obj,['                    -- second normalized pos: ',mat2str(snor)]);
    Cmnf.logc(obj,['                    -- min angles:            ',mat2str(angles)]);
    Cmnf.logc(obj,['-- Rules of the Air -- ANGLE OF APPROACH:     ',mat2str(angleOfApproach), ' deg']);
    
    % By the rule of right hand estimate who has the right of the way
    stheta=-atan2(snor(2),snor(1));
    fnrot=Cmnf.rot2D(stheta,fnor(1:2));
    fntheta=-atan2(fnrot(2),fnrot(1));
    Cmnf.logc(obj,['                    -- fntheta:',mat2str(fntheta)]);
    if fntheta >=0
        flagFirstRightOfWay=false;
        flagSecondRightOfWay=true;
        Cmnf.logc(obj,['-- Rules of the Air -- MASTER: ',mat2str(smc.vehicleName), ]);
        Cmnf.logc(obj,['-- Rules of the Air -- SLAVE:  ',mat2str(fmc.vehicleName), ]);
    else
        flagFirstRightOfWay=true;
        flagSecondRightOfWay=false;
        Cmnf.logc(obj,['-- Rules of the Air -- MASTER: ',mat2str(fmc.vehicleName), ]);
        Cmnf.logc(obj,['-- Rules of the Air -- SLAVE:  ',mat2str(smc.vehicleName), ]);
    end
    
    % based on angle of approach estimate the avoidance maneuver type 
    if angleOfApproach >= 130
        collisionCategory=CollisionCategory.HeadOnApproach;
        Cmnf.logc(obj,['-- Rules of the Air -- MANEUVER: HEAD ON APPROACH' ]);
    else if angleOfApproach >= 70
            collisionCategory=CollisionCategory.Converging;
            Cmnf.logc(obj,['-- Rules of the Air -- MANEUVER: CONVERGING' ]);
        else
            collisionCategory=CollisionCategory.Overtaking;
            Cmnf.logc(obj,['-- Rules of the Air -- MANEUVER: OVERTAKE' ]);
        end
    end
    % based on the situation set roles 
    if collisionCategory==CollisionCategory.HeadOnApproach;
        frole=UavIntruderRole.Roundabouting;
        srole=UavIntruderRole.Roundabouting;
        Cmnf.logc(obj,['-- Rules of the Air -- ROLES: Roundabouting/Roundabouting' ]);
    end
    if collisionCategory==CollisionCategory.Converging;
        if flagFirstRightOfWay
            frole=UavIntruderRole.RightOfTheWay;
            srole=UavIntruderRole.Converging;
            Cmnf.logc(obj,['-- Rules of the Air -- ROLES: RightOfTheWay/Converging' ]);
        end
        if flagSecondRightOfWay
            frole=UavIntruderRole.Converging;
            srole=UavIntruderRole.RightOfTheWay;
            Cmnf.logc(obj,['-- Rules of the Air -- ROLES: Converging/RightOfTheWay' ]);
        end
    end
    if collisionCategory==CollisionCategory.Overtaking;
        if flagFirstRightOfWay
            frole=UavIntruderRole.RightOfTheWay;
            srole=UavIntruderRole.Overtaking;
            Cmnf.logc(obj,['-- Rules of the Air -- ROLES: RightOfTheWay/Overtaking' ]);
        end
        if flagSecondRightOfWay
            frole=UavIntruderRole.Overtaking;
            srole=UavIntruderRole.RightOfTheWay;
            Cmnf.logc(obj,['-- Rules of the Air -- ROLES: Overtaking/RightOfTheWay' ]);
        end
    end
    frasm=fpm.getSafetyMargin(collisionCategory);
    srasm=spm.getSafetyMargin(collisionCategory);
    rulesOfAirSafetyMargin=max(frasm,srasm);
    Cmnf.logc(obj,['                    -- rule of air safety margin criterion,',mat2str(collisionCategory)]);
    Cmnf.logc(obj,['                    -- first safety margin criterion,',mat2str(frasm), '[m]']);
    Cmnf.logc(obj,['                    -- second safety margin criterion,',mat2str(srasm), '[m]']);
    Cmnf.logc(obj,['                    -- Rules Of Air safety margin criterion,',mat2str(rulesOfAirSafetyMargin), '[m]']);
    if intersectionDistance < rulesOfAirSafetyMargin
        emergencyFlag = false;
        Cmnf.logc(obj,['-- Rules of the Air -- achieved']);
    else
        emergencyFlag = true;
        Cmnf.logc(obj,['-- Rules of the Air -- not achieved, proceed with emergency maneuvers']);
    end
else
    emergencyFlag = true;
    Cmnf.logc(obj,['-- Rules of the Air -- not achieved']); % this should not happened => well still covered with emergency avoidance -> all panic button
end

%Create and fill collision case object
cc=CollisionCase;

cc.utmCreationTimeFrame=obj.actualDecisionFrame;
cc.utmResultionTimeFrame=Inf;                %Infinum at time of reation

%MissionControl
cc.firstId=fid;
cc.secondId=sid;
cc.firstMissionControl=fmc;
cc.secondMissionControl=smc;

%Position/velocity
cc.firstPosition=fpm.getPosition;
cc.secondPosition=spm.getPosition;
cc.firstVelocity=fpm.velocity;
cc.secondVelocity=spm.velocity;

%trajectory prediction
cc.firstTrajectory=ftraj;
cc.secondTrajectory=straj;
cc.distanceVector=distanceVector;

%linearPrediction
cc.collisionPoint=intersectionPoint;
if ~emergencyFlag
    cc.angleOfApproach=angleOfApproach;
end
cc.linearIntersectionTime=intersectionTime;
cc.linearIntersectionDistance=intersectionDistance;
if ~emergencyFlag
    cc.flagFirstRightOfWay=flagFirstRightOfWay;
    cc.flagSecondRightOfWay=flagSecondRightOfWay;
    cc.stheta=stheta; 
    cc.fnrot=fnrot;
    cc.fntheta=fntheta; 
end

%safetyMargins
cc.defaultSafetyMargin=defaultSafetyMargin;
cc.ruleSafetyMargin=rulesOfAirSafetyMargin;

%collision category
cc.collisionCategory=collisionCategory;

%vehicle roles
cc.firstAvoidanceRole=frole;
cc.secondAvoidanceRole=srole;

%flags
cc.collisionFlag=collisionFlag;                                 
cc.emergencyAvoidanceFlag=emergencyFlag;
cc.trajectoryIntersectionFlag=sum(conditionVector) > 0;
if ~emergencyFlag
    cc.linearIntersectionFlag=lineFlag;  
    cc.flagFirstRightOfWay=flagFirstRightOfWay;
    cc.flagSecondRightOfWay=flagSecondRightOfWay;
else
    cc.linearIntersectionFlag=false;  
    cc.flagFirstRightOfWay=false;
    cc.flagSecondRightOfWay=false;
end
r=cc;
    


