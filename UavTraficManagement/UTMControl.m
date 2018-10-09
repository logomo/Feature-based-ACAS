classdef UTMControl<RullableObject
    %UTMCONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        actualDecisionFrame=0;
        vehicleCount=0;
        missionControls=[];
        positionNotifications=[];
        %flags
        showStatFlag=[];
        collisionCases=containers.Map;
    end
    
    methods
        function obj=UTMControl()
            obj.actualDecisionFrame=0;
            obj.vehicleCount=0;
            obj.missionControls=[];
            obj.positionNotifications=[];
            %flags
            obj.showStatFlag=[];
            obj.collisionCases=containers.Map;
        end
        %% Add vehicle into mission Control
        function r=registerMission(obj,missionControl,statFlag)
            if nargin<2
                statFlag=false;
            end
            obj.vehicleCount=obj.vehicleCount+1;
            missionControl.vehicleId = obj.vehicleCount;
            obj.missionControls=[obj.missionControls,missionControl];
            obj.positionNotifications=[obj.positionNotifications,missionControl.getVehiclePositionNotification];
            obj.showStatFlag=[obj.showStatFlag,statFlag];
            Cmnf.logc(obj,['Adding mission id:', mat2str(missionControl.vehicleId),' with vehicle ', missionControl.vehicleName]);
            missionControl.plotMissionStaticContent;
            r=missionControl;
        end
        
        %% Active mission Count
        function r=activeMissionsCount(obj)
            r=0;
            for ms=obj.missionControls
                if ms.isActive
                    r=r+1;
                end
            end
        end
        
        %% runSimulations
        function r=runSimulations(obj,withPlot)
            if(nargin < 2)
                withPlot=true;
            end
            amc = obj.activeMissionsCount;
            if amc == 0
                Cmnf.logc(obj,'No active mission found, -- Terminate session --');
                r=0;
            else
                Cmnf.logc(obj,[mat2str(amc),'Active mission(s) found, proceeding with simulation']);
            end
            % Create collision cases before simulation run
            Cmnf.logc(obj,'[CollisionCases] Collision cases creation start')
            obj.createCollisionCases;
            Cmnf.logc(obj,'[CollisionCases] Collision cases creation end')
            
        
            
            % Standard simmulation 
            for k=1:length(obj.missionControls)
                mc=obj.missionControls(k);
                Cmnf.logc(obj,['Checking ',mat2str(k),' id:', mat2str(mc.vehicleId), ' name: ',mat2str(mc.vehicleName)]);
                if mc.isActive
                    Cmnf.logc(obj,['-- Active -- id:', mat2str(mc.vehicleId), ' name: ',mat2str(mc.vehicleName),'-- proceed simulation']);
                    for l=1:mc.vehicleActualVelocity
                        if withPlot
                            mc.runOnceWithPlot(1,obj.showStatFlag(k));
                        else
                            mc.runOnce(obj.showStatFlag(k));
                        end
                    end
                else    
                    Cmnf.logc(obj,['-- Inactive -- id:', mat2str(mc.vehicleId), ' name: ',mat2str(mc.vehicleName),'-- cease simulation']);
                end
            end
            %Gather after simulation messages
            obj.positionNotifications=[];
            for k=1:length(obj.missionControls)
                mc=obj.missionControls(k);
                msg=mc.getVehiclePositionNotification;
                msg.flagMissionActive=mc.isActive;
                obj.positionNotifications=[obj.positionNotifications,msg];
            end
            %forcefed figure 1 title
            if amc ~= 0
                figure(1);
                title(['Decision frame ',mat2str(obj.actualDecisionFrame),'.'])
                obj.actualDecisionFrame=obj.actualDecisionFrame+1;
            end
            
            r=amc;
        end
        
        %% Create collision Case
        function r=createCollisionCase(obj,fid,sid)
            %Create and fill collision case object
            cc=CollisionCase;
            fmc= obj.missionControls(fid);
            smc= obj.missionControls(sid);
            Cmnf.logcft(cc,['Time frame: ',mat2str(obj.actualDecisionFrame),'. ']);
            Cmnf.logcft(cc,['Testing collision ID: ',mat2str(fmc.vehicleId),' ',fmc.vehicleName,' agains ', 'ID: ',mat2str(smc.vehicleId),' ',smc.vehicleName]);

            % test if vehicles are active
            if ~(fmc.isActive && smc.isActive)
                Cmnf.logcft(cc,'-- Failed -- One or more vehicles are inactive');
                r=-1;
                return;
            else
                Cmnf.logcft(cc,'-- Passed -- Vehicles are active');
            end

            %Load their last positions
            fpm=obj.positionNotifications(fid);
            spm=obj.positionNotifications(sid);

            %Check flight level
            if fpm.mainFlightLevel == spm.mainFlightLevel ||... 
               fpm.mainFlightLevel == spm.passingFlightLevel || ...
               spm.fpm.mainFlightLevel ==  fpm.passingFlightLevel
                Cmnf.logcft(cc,'-- Passed -- Detected crossing flight levels');
            else
                Cmnf.logcft(cc,'-- Failed -- No crossing in flight levels');
                r=-2;
            end

            %Initialize safety margin
            collisionCategory=CollisionCategory.Unknown;
            fsm=fpm.getSafetyMargin(CollisionCategory.Unknown);
            ssm=spm.getSafetyMargin(CollisionCategory.Unknown);
            defaultSafetyMargin=max(fsm,ssm);
            Cmnf.logcft(cc,['Safetymargin ',fmc.vehicleName,': ',mat2str(fsm),' m, ', smc.vehicleName, ': ',mat2str(ssm),' m']);
            Cmnf.logcft(cc,['-- SafetyMargin -- set as ',mat2str(defaultSafetyMargin),' m']);

            %Trajectory prediction test
            ftraj=fmc.predictTrajectory;
            straj=smc.predictTrajectory;
            distanceVector=sqrt([1,1,1]*((ftraj - straj).^2));
            conditionVector = distanceVector < defaultSafetyMargin;
            Cmnf.logcft(cc,['-- ProjectedTrajectory -- distance vector:  ',mat2str(distanceVector)]);
            Cmnf.logcft(cc,['-- ProjectedTrajectory -- condition vector: ',mat2str(conditionVector)]);
            collisionFlag = false;
            if sum(conditionVector) > 0
                Cmnf.logcft(cc,['-- ProjectedTrajectory -- collision detected, raising flag']);
                collisionFlag = true;
            end

            % determine initial roles
            frole=UavIntruderRole.Emergencing;
            srole=UavIntruderRole.Emergencing;
            emergencyFlag=true;
            % vehicle with higher category has the right of the way
            if fpm.vehicleCategory ~= spm.vehicleCategory
                Cmnf.logcft(cc,['-- ClassComp -- Vehicle Category mismatch ',...
                    mat2str(fpm.vehicleCategory),' vs. ',mat2str(spm.vehicleCategory)]);
                if fpm.vehicleCategory < spm.vehicleCategory
                    frole=UavIntruderRole.RightOfTheWay;
                    srole=UavIntruderRole.Emergencing;
                    Cmnf.logcft(cc,['-- ClassComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
                end
                if fpm.vehicleCategory > spm.vehicleCategory
                    frole=UavIntruderRole.Emergencing;
                    srole=UavIntruderRole.RightOfTheWay;
                    Cmnf.logcft(cc,['-- ClassComp -- Vehicle ',smc.vehicleName,' has the right of way']);
                end
            else
                Cmnf.logcft(cc,['-- ClassComp -- Vehicle Category match ',...
                    mat2str(fpm.vehicleCategory),' vs. ',mat2str(spm.vehicleCategory)]);
                %Vehicle with higher manuevurabilty has higher priority
                if fpm.manCat ~= spm.manCat
                     Cmnf.logcft(cc,['-- ManCatComp -- Manuevaribility category miss match ',...
                         mat2str(fpm.manCat),' vs. ',mat2str(spm.manCat)]);
                     if fpm.manCat < spm.manCat
                         frole=UavIntruderRole.RightOfTheWay;
                         srole=UavIntruderRole.Emergencing;
                         Cmnf.logcft(cc,['-- ManCatComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
                     end
                     if fpm.manCat > spm.manCat
                         frole=UavIntruderRole.Emergencing;
                         srole=UavIntruderRole.RightOfTheWay;
                         Cmnf.logcft(cc,['-- ManCatComp -- Vehicle ',fmc.vehicleName,' has the right of way']);
                     end
                else
                    Cmnf.logcft(cc,['-- ManCatComp -- Manuevaribility category match ',...
                         mat2str(fpm.manCat),' vs. ',mat2str(spm.manCat)]);
                    emergencyFlag=false;
                    Cmnf.logcft(cc,'-- Emergency Mode -- deactivated, starting cooperatove avoidance');
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
            Cmnf.logcft(cc,['-- LinearProjection -- flag         :',mat2str(lineFlag)]);
            Cmnf.logcft(cc,['-- LinearProjection -- intersection :',mat2str(intersectionPoint),' [m,m,m]']);
            Cmnf.logcft(cc,['-- LinearProjection -- time to crash:',mat2str(intersectionTime), ' s']);
            Cmnf.logcft(cc,['-- LinearProjection -- min distance :',mat2str(intersectionDistance),' m']);

            %Now we have a situation
            if lineFlag && intersectionDistance< defaultSafetyMargin
                Cmnf.logcft(cc,['-- Rules of the Air -- start']);

                %Calculate angle of approach
                fnor=fpos-intersectionPoint; % normalize first position to intersection point
                snor=spos-intersectionPoint; % normalize second position to intersection point
                %angles=Cmnf.minimalAnglesBetweenVectors(fnor,snor);
                %angleOfApproach=angles(3);
                angleOfApproach=Cmnf.minimalAnglesBetweenVectors(fnor,snor);
                Cmnf.logcft(cc,['                    -- first normalized pos:  ',mat2str(fnor)]);
                Cmnf.logcft(cc,['                    -- second normalized pos: ',mat2str(snor)]);
                Cmnf.logcft(cc,['-- Rules of the Air -- ANGLE OF APPROACH:     ',mat2str(angleOfApproach), ' deg']);

                % By the rule of right hand estimate who has the right of the way
                stheta=-atan2(snor(2),snor(1));
                fnrot=Cmnf.rot2D(stheta,fnor(1:2));
                fntheta=-atan2(fnrot(2),fnrot(1));
                Cmnf.logcft(cc,['                    -- fntheta:',mat2str(fntheta)]);
                if fntheta >=0
                    flagFirstRightOfWay=false;
                    flagSecondRightOfWay=true;
                    Cmnf.logcft(cc,['-- Rules of the Air -- MASTER: ',mat2str(smc.vehicleName), ]);
                    Cmnf.logcft(cc,['-- Rules of the Air -- SLAVE:  ',mat2str(fmc.vehicleName), ]);
                else
                    flagFirstRightOfWay=true;
                    flagSecondRightOfWay=false;
                    Cmnf.logcft(cc,['-- Rules of the Air -- MASTER: ',mat2str(fmc.vehicleName), ]);
                    Cmnf.logcft(cc,['-- Rules of the Air -- SLAVE:  ',mat2str(smc.vehicleName), ]);
                end

                % based on angle of approach estimate the avoidance maneuver type 
                if angleOfApproach >= 130
                    collisionCategory=CollisionCategory.HeadOnApproach;
                    Cmnf.logcft(cc,['-- Rules of the Air -- MANEUVER: HEAD ON APPROACH' ]);
                else if angleOfApproach >= 70
                        collisionCategory=CollisionCategory.Converging;
                        Cmnf.logcft(cc,['-- Rules of the Air -- MANEUVER: CONVERGING' ]);
                    else
                        collisionCategory=CollisionCategory.Overtaking;
                        Cmnf.logcft(cc,['-- Rules of the Air -- MANEUVER: OVERTAKE' ]);
                    end
                end
                % based on the situation set roles 
                if collisionCategory==CollisionCategory.HeadOnApproach;
                    frole=UavIntruderRole.Roundabouting;
                    srole=UavIntruderRole.Roundabouting;
                    Cmnf.logcft(cc,['-- Rules of the Air -- ROLES: Roundabouting/Roundabouting' ]);
                end
                if collisionCategory==CollisionCategory.Converging;
                    if flagFirstRightOfWay
                        frole=UavIntruderRole.RightOfTheWay;
                        srole=UavIntruderRole.Converging;
                        Cmnf.logcft(cc,['-- Rules of the Air -- ROLES: RightOfTheWay/Converging' ]);
                    end
                    if flagSecondRightOfWay
                        frole=UavIntruderRole.Converging;
                        srole=UavIntruderRole.RightOfTheWay;
                        Cmnf.logcft(cc,['-- Rules of the Air -- ROLES: Converging/RightOfTheWay' ]);
                    end
                end
                if collisionCategory==CollisionCategory.Overtaking
                    if fmc.vehicleActualVelocity < smc.vehicleActualVelocity
                        frole=UavIntruderRole.RightOfTheWay;
                        srole=UavIntruderRole.Overtaking;
                        Cmnf.logcft(cc,['-- Rules of the Air -- ROLES: RightOfTheWay/Overtaking' ]);
                    else
                        frole=UavIntruderRole.Overtaking;
                        srole=UavIntruderRole.RightOfTheWay;
                        Cmnf.logcft(cc,['-- Rules of the Air -- ROLES: Overtaking/RightOfTheWay' ]);
                    end
                    if fmc.vehicleActualVelocity == smc.vehicleActualVelocity
                        %%Logic when speed is the same ?
                    end
                end
                frasm=fpm.getSafetyMargin(collisionCategory);
                srasm=spm.getSafetyMargin(collisionCategory);
                rulesOfAirSafetyMargin=max(frasm,srasm);
                Cmnf.logcft(cc,['                    -- rule of air safety margin criterion,',mat2str(collisionCategory)]);
                Cmnf.logcft(cc,['                    -- first safety margin criterion,',mat2str(frasm), '[m]']);
                Cmnf.logcft(cc,['                    -- second safety margin criterion,',mat2str(srasm), '[m]']);
                Cmnf.logcft(cc,['                    -- Rules Of Air safety margin criterion,',mat2str(rulesOfAirSafetyMargin), '[m]']);
                if intersectionDistance < rulesOfAirSafetyMargin
                    emergencyFlag = false;
                    Cmnf.logcft(cc,['-- Rules of the Air -- achieved']);
                else
                    emergencyFlag = true;
                    Cmnf.logcft(cc,['-- Rules of the Air -- not achieved, proceed with emergency maneuvers']);
                end
            else
                emergencyFlag = true;
                Cmnf.logcft(cc,['-- Rules of the Air -- not achieved']); % this should not happened => well still covered with emergency avoidance -> all panic button
            end

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
            cc.firstOrientation=fpm.orientation;
            cc.secondOrienatation=spm.orientation;
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
            if ~emergencyFlag
                cc.ruleSafetyMargin=rulesOfAirSafetyMargin;
            end

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
            
            if cc.collisionFlag  || cc.trajectoryIntersectionFlag || cc.linearIntersectionFlag
                r=cc;
            else
                r=-4; % there is no collision ocuring from rules nor trajectory nor other means
            end
        end
        %% Create collision case hash
        function r=hashCollisionCase(obj,firstId,secondId)
            r=['CollisionCase ','firstId: ',mat2str(firstId),' - secondId: ',mat2str(secondId)];
        end
        %% Select collision case based on Hash
        function r=selectColissionCase(obj,firstId,secondId)
            hash=obj.hashCollisionCase(firstId,secondId);
            existKeyFlag=obj.collisionCases.isKey(hash);
            if existKeyFlag
                r=obj.collisionCases(hash);
            else
                r=0;
            end
        end
        
        %% Select collision cases for id
        function r=selectCollisionCases(obj,vehicleID)
            r=[];
            for k=1:length(obj.missionControls)
                mc=obj.missionControls(k);
                compId=mc.vehicleId;
                if compId ~= vehicleID
                    ids=sort([compId,vehicleID]);
                    candidate=obj.selectColissionCase(ids(1),ids(2));
                    if candidate ~= 0
                        r=[r,candidate];
                    end
                end
            end
        end
        
        %% Link collision case 
        function r=linkCollisionCase(obj,collisionCase)
            if (~isobject(collisionCase))||~strcmp(class(collisionCase),'CollisionCase')
                r=-1; % return codes everywhere
                return
            end
            %helper variables
            map = obj.collisionCases;
            fid = collisionCase.firstId;
            sid = collisionCase.secondId;
            candidate=obj.selectColissionCase(fid,sid);
            if isobject(candidate)&&strcmp(class(candidate),'CollisionCase')
                hash=obj.hashCollisionCase(fid,sid);
                map(hash)=collisionCase;
                collisionCase.linkedCollisionCase=candidate;
                % if the candidate is not closed yet -> Resulution time
                % frame == Inf, 
                if candidate.utmResultionTimeFrame == Inf
                    % Then set head to old head
                    collisionCase.headCollisionCase=candidate.headCollisionCase;
                else
                    % Create new head otherwise
                    collisionCase.headCollisionCase=collisionCase;
                end
            else
                hash=obj.hashCollisionCase(fid,sid);
                map(hash)=collisionCase;
                collisionCase.linkedCollisionCase=0;
                % collision case is first interaction of this type,
                % therefore it is necessary to set it as own head
                collisionCase.headCollisionCase=collisionCase;
            end
            r=0;
        end
       
        %% Create collision Cases
        function r=createCollisionCases(obj)
            for k=1:(length(obj.missionControls) - 1)
                if (obj.missionControls(k).isActive)
                    Cmnf.logc(obj,['[Collision cases] Mission id: ' ,mat2str(k), ' vehicleName: '...
                        obj.missionControls(k).vehicleName, 'is active proceeding to find match']);
                    for l=(k+1):length(obj.missionControls)
                        if (obj.missionControls(l).isActive)
                            Cmnf.logc(obj,['[Collision cases] Mission id: ' ,mat2str(l), ' vehicleName: '...
                                            obj.missionControls(l).vehicleName, 'is active proceeding collision case']);
                            collisionCase=obj.createCollisionCase(k,l);
                            linkResultCode=obj.linkCollisionCase(collisionCase);
                            % Well all good storries and collisions come to
                            % end, also this one ....
                            if linkResultCode ~= 0 
                                closingCase=obj.selectColissionCase(k,l);
                                if closingCase ~= 0 && closingCase.utmResultionTimeFrame == Inf
                                    closingCase.utmResultionTimeFrame=obj.actualDecisionFrame;
                                end
                            end
                        end
                    end
                end
            end
            r=0;
        end
        
        %% showCollisionCaseTrace % debuf function
        % utmControl.showCollisioncaseTrace(1,2,1)
        function r=showCollisioncaseTrace(obj,firstId,secondId,depth)
            if nargin == 3
                depth=inf;
            end
            reached=1;
            candidate=selectColissionCase(obj,firstId,secondId);
            if isobject(candidate) && strcmp(class(candidate),'CollisionCase')
               while reached <= depth 
                   fprintf(['#########################################\n Record: ',...
                          mat2str(reached),...
                          '.\n#########################################\n']);
                   if Cmnf.enabledForcedTrace
                       candidate.trace;
                   else
                       candidate
                   end
                   
                   candidate=candidate.linkedCollisionCase;
                   reached=reached+1;
                   if candidate==0
                       break;
                   end
               end
            end
        end
        
        %% Rule engine create context (DO NOT CALL DIRECTLY)
        function r=createContextRuleEngine(obj)
            createContextRuleEngine@RullableObject(obj);
            obj.reContext('utmControl')=obj;
            r=obj.reContext;
        end
        
        %% Rule engine injection method
        function r=injectRuleEngine(obj,ruleEngine)
            masterFlag=injectRuleEngine@RullableObject(obj,ruleEngine);
            % TODO injection body
            mcs=obj.missionControls;
            for mc=mcs
                mc.injectRuleEngine(ruleEngine);
                mc.appendContextRuleEngine(obj);
            end
            % End of Injection
            r=masterFlag;
        end
    end
end

