classdef CollisionCase < LoggableObject
    %COLLISIONCASE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        utmCreationTimeFrame=0;                   %timeFrame when case was created
        utmResultionTimeFrame=Inf;                %timeFrame when case was closed Inf if open
        
        %MissionControl
        firstId
        secondId
        firstMissionControl
        secondMissionControl
        
        %Position/velocity
        firstPosition
        secondPosition
        firstVelocity
        secondVelocity
        firstOrientation
        secondOrienatation
        
        %trajectory prediction
        firstTrajectory
        secondTrajectory
        distanceVector
        
        %linearPrediction
        collisionPoint
        angleOfApproach
        linearIntersectionTime
        linearIntersectionDistance
        stheta   %% second offset to base angle
        fnrot    %% first rotation
        fntheta  %% first normalized in relation to second angle (>=0 first have a right)
        
        %safetyMargins
        defaultSafetyMargin
        ruleSafetyMargin
        
        %collision category
        collisionCategory
        
        %vehicle roles
        firstAvoidanceRole
        secondAvoidanceRole
        
        %flags
        collisionFlag                                 % collision from trajectories prediction or linear intersection method - action needed if true
        emergencyAvoidanceFlag                        % true one or both emergency avoidance
        trajectoryIntersectionFlag                    % indicates trajectory intersection method
        linearIntersectionFlag                        % indicates trajectory intersection method success
        flagFirstRightOfWay
        flagSecondRightOfWay
        
        %linkedStructure
        headCollisionCase=0;
        linkedCollisionCase=0;
    end
    
    methods
        %% alive mission control test function
        % Test mission control active status
        function r=testMissionControls(obj)
            r=obj.firstMissionControl.isActive &&...
              obj.secondMissionControl.isActive;
        end
        
        %% open Collision case test;
        function r=isOpenCase(obj)
            r=obj.utmResultionTimeFrame == Inf;
        end
        %% get head
        function r=getHeadCase(obj)
            if obj.headCollisionCase==0
                r=obj;
            else
                r=obj.headCollisionCase;
            end
        end
    end
    
end

