classdef AbstractConstraint<LoggableObject
    %ABSTRACTOBSTACLE represents abstract constraints
    
    properties
        id=0;                               % Can not BE 0 sholuld be set by MissionControl
        center=[0;0;0]                      % XYZ coordiantes of constraint [GLOBAL]
        safetyMargin=5                      % Default protection zone 5 meters
        type=CostraintType.Static           % Constraint type
        breachibility=CostraintType.Hard;   % By default constraint is hard
        points=[0;0;0]                      % point set for plot purposes
        % Moving constraint type             
        initialPosition                     % initial position at time 0
        initialTime                         % initial time when moving obstacle have been added
        velocityVector                      % velocity vector 
    end
    
    methods
        %% Common methods - for constraints the point intersection method is obsolete, it is possible to generate points - see poly plot methods
        function r=getPoints(obj)
            %Gets points of the obstacle
            r=obj.points;
        end
        
        %%Moving constraints functionality
        function r=dynamize(obj,position,velocity,initialTime)
            %Moving constraints functionalit
            if nargin <=3
                obj.initialTime = 0;
            else
                obj.initialTime = initialTime;
            end
            obj.initialPosition = position;
            obj.velocityVector = velocity;
            obj.type=CostraintType.Moving;
        end
        
        function r=applyMovement(obj,mc)
            % applies movement on the constraint
            simTime=max(mc.vehicle.state.time);
            relTime=simTime - obj.initialTime;
            obj.center=obj.initialPosition + obj.velocityVector*relTime;
            % change Z boundaries
            offset=obj.velocityVector;
            zoff=offset(3);
            obj.startHeight = obj.startHeight + zoff;
            obj.endHeight = obj.endHeight + zoff;
            obj.plotStartHeight = obj.plotStartHeight + zoff;
            obj.plotEndHeight = obj.plotEndHeight + zoff;
            r=0;
        end
        
        function r=estimateCenter(obj,time)
            %estimates new center of abstract constraint
            relTime=time - obj.initialTime;
            r=obj.initialPosition + obj.velocityVector*relTime;
        end
                
      
        % Do not override - only for presentation purposes
        function r=getColor(obj)
            %Returns color of constraints
            r=[1,1,1];
            if obj.type == CostraintType.Static
               r=[1 0 0]; 
            end
            if obj.type == CostraintType.Moving
               r=[0 0 1]; 
            end
        end
        
        %override in geral class
        function r=isIntersection(obj,posor,mc)
            %Cell intersection calculation
            r = true;
        end
        
        % override in general class
        function r=inRange(obj,mc)
            %Range check function
            r= true;
        end
        
        function handles=plot(obj)
            %Plot abstract constraint
            handles=[];
            [m,n]=size(obj.points);
            hold on
            h=plot3(obj.center(1),obj.center(2),obj.center(3),'Marker','o','MarkerFaceColor',obj.getColor,'MarkerEdgeColor',[0 0 0]);
            handles=[h];
            for k=1:n
                h=plot3(obj.points(1,k),obj.points(2,k),obj.points(3,k), 'Marker','o','MarkerFaceColor',obj.getColor,'MarkerEdgeColor',[0 0 0]);
                handles=[handles,h];
            end
            hold off
        end
        
        function r=getLogString(obj)
            %Creates log string - override
            r='';
        end
    end
    
end

