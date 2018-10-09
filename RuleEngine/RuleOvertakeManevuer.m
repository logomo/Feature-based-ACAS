classdef RuleOvertakeManevuer<AbstractRule
    %TESTRULE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       vehicleId,
       collisionCase,
       collisionPoint,
       missionControl,
       avoidanceGrid,
       velocity,
       reachSet,
       position,
       orientation,
       collisionCategory,
       avoidanceRole,
       safetyMargin,
    end
    
    methods
        function obj = RuleOvertakeManevuer(context,jointPointCode,ruleCode)
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            priorFlag=parseContext@AbstractRule(obj);
            % Additional parse context functionality here
            % START
            obj.vehicleId=obj.context('vehicleId');
            obj.collisionCase=obj.context('collisionCase');
            obj.collisionPoint=obj.context('collisionPoint');
            obj.missionControl=obj.context('missionControl');
            obj.avoidanceGrid=obj.context('avoidanceGrid');
            obj.velocity=obj.context('velocity');
            obj.reachSet=obj.context('reachSet');
            obj.position=obj.context('position');
            obj.orientation=obj.context('orientation');
            obj.collisionCategory=obj.context('collisionCategory');
            obj.avoidanceRole=obj.context('avoidanceRole');
            obj.safetyMargin=obj.context('safetyMargin');
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            % Load information from context
            if obj.collisionCase.firstId == obj.missionControl.vehicleId
                overtakenMissionControl = obj.collisionCase.secondMissionControl;
                overtakenOrientation = obj.collisionCase.headCollisionCase.secondOrienatation;
            else
                overtakenMissionControl = obj.collisionCase.firstMissionControl;
                overtakenOrientation = obj.collisionCase.headCollisionCase.firstOrientation;
            end
            overtakenPosOr = overtakenMissionControl.vehicle.getActualPositionOrientation;
            myPosOr = obj.missionControl.vehicle.getActualPositionOrientation;
            overtakenPosition = overtakenPosOr(1:3);
            myPosition = myPosOr(1:3);
            %calculatedivergence point
            fp=obj.collisionCase.headCollisionCase.firstPosition;
            sp=obj.collisionCase.headCollisionCase.secondPosition;
            fv=obj.collisionCase.headCollisionCase.firstVelocity;
            sv=obj.collisionCase.headCollisionCase.secondVelocity;
            speedDiff=abs(norm(fv)-norm(sv));
            reachWPMargin=Cmnf.vehicleSpeed*Cmnf.simStep*2;;
            overtakeMiddle=sqrt((fp-sp)'*(fp-sp) +(obj.safetyMargin)^2);
            overtakeMiddle=floor(overtakeMiddle) + 2*speedDiff + reachWPMargin; 

            % set divergence point
            actGoal=obj.missionControl.goal;
            newGoal=obj.collisionCase.headCollisionCase.collisionPoint ...
                    + Cmnf.rot3Dvec([0;-obj.safetyMargin-speedDiff;0],overtakenOrientation);
            newReturn=obj.collisionCase.headCollisionCase.collisionPoint... 
                    + Cmnf.rot3Dvec([overtakeMiddle;-obj.safetyMargin-speedDiff;0],overtakenOrientation);
            if norm(actGoal-newGoal)~=0 && myPosition(1)<overtakenPosition(1)
                obj.missionControl.goal= newGoal;
                obj.missionControl.waypoints=[Waypoint(obj.missionControl.goal),Waypoint(newReturn),obj.missionControl.waypoints];
                obj.missionControl.goal
            end
            %manContext('collisionPoint')=overtakenPosition;
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

