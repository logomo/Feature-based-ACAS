classdef CollisionCase < LoggableObject
    %COLLISIONCASE Data shell for calculated collision case (Refer to 6.6)
    
    properties
        utmCreationTimeFrame=0;                   %timeFrame when case was created
        utmResultionTimeFrame=Inf;                %timeFrame when case was closed Inf if open
        
        %MissionControl
        firstId                                   %UAS 1 UQ ID
        secondId                                  %UAS 2 UQ ID
        firstMissionControl                       %UAS 1 MissionControl object reference
        secondMissionControl                      %UAS 2 MissionControl object reference   
        
        %Position/velocity
        firstPosition                             %UAS 1 position GCF
        secondPosition                            %UAS 2 position GCF 
        firstVelocity                             %UAS 1 velocity relative to GCF
        secondVelocity                            %UAS 2 velocity relative to GCF
        firstOrientation                          %UAS 1 orientaiton in GCF
        secondOrienatation                        %UAS 2 orientation in GCF
        
        %trajectory prediction
        firstTrajectory                           %UAS 1 event horizont trajectory prediction
        secondTrajectory                          %UAS 2 event horizont trajectory prediction
        distanceVector                            %The |a-b| norm between UAS1/2 trajectory predictions, used as check vector
        
        %linearPrediction
        collisionPoint                            % 3D GCF point of collision or MCA or Well clear Breach
        angleOfApproach                           % mionimal planar angle betweenn CP, UAS1 UAS2
        linearIntersectionTime                    % time when linear intersection happens
        linearIntersectionDistance                % distance between UAS 1 UAS 2 trajectory predictions
        stheta                                    % [helper] Second offset to base angle         
        fnrot                                     % [helper] 1first rotation
        fntheta                                   % [helper] first normalized in relation to second angle (>=0 first have a right)
        
        %safetyMargins
        defaultSafetyMargin                       % Default safety margin imposed by UAS 1/2 class and situation
        ruleSafetyMargin                          % Rule enforced safety margin in case the specific events
        
        %collision category
        collisionCategory                         % Collision Category enumeration member, type of the situation
        
        %vehicle roles
        firstAvoidanceRole                        % UAS 1 avoidance role
        secondAvoidanceRole                       % UAS 2 avoidance role
        
        %flags
        collisionFlag                                 % collision from trajectories prediction or linear intersection method - action needed if true
        emergencyAvoidanceFlag                        % true one or both emergency avoidance
        trajectoryIntersectionFlag                    % indicates trajectory intersection method
        linearIntersectionFlag                        % indicates trajectory intersection method success
        flagFirstRightOfWay                           % indicates UAS 1 right of the way
        flagSecondRightOfWay                          % indicates UAS 2 right of the way
        
        %linkedStructure
        headCollisionCase=0;                          % reference to first detection/resolution of the situation
        linkedCollisionCase=0;                        % reference to next time frame collision case (they are linked from detection to closure)
    end
    
    methods
        %% alive mission control test function
        % Test mission control active status
        function r=testMissionControls(obj)
            % Test mission control active status
            r=obj.firstMissionControl.isActive &&...
              obj.secondMissionControl.isActive;
        end
        
        %% open Collision case test;
        function r=isOpenCase(obj)
            %open Collision case test;
            r=obj.utmResultionTimeFrame == Inf;
        end
        %% get head
        function r=getHeadCase(obj)
            % Get head collision case
            if obj.headCollisionCase==0
                r=obj;
            else
                r=obj.headCollisionCase;
            end
        end
    end
    
end

