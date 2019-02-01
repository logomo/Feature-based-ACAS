classdef AbstractObstacle<LoggableObject
    %ABSTRACTOBSTACLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id=0;                       % Can not BE 0 sholuld be set by MissionControl
        center=[0;0;0]              % XYZ coordiantes of obstacle [GLOBAL]
        collisionRange=0            % Collision range (m) MIN
        intersectionRange=0         % Intersection range (m) MAX
        points=[0;0;0]              % Obstacle points XYZ [GLOBAL]
        type=ObstacleType.Detected  % ObstacleType-detected/map
    end
    
    methods
        %% Common methods
        function r=getPoints(obj)
            %Returns points for obstacle
            r=obj.points;
        end
        
        function r=getColor(obj)
            %Returns color of obstacle
            r=[1,1,1];
            if obj.type == ObstacleType.Detected
               r=[1 0 0]; 
            end
            if obj.type == ObstacleType.Map
               r=[0 0 1]; 
            end
        end
        
        function r=isCollision(obj,position)
            %Default collision check with UAS at position
            distance=norm(obj.center-position,2);
            r = distance <= obj.collisionRange;
        end
        
        function r=isIntersection(obj,position,visibilityFieldRange)
            %Is there an intersection with visible world ?
            distance=norm(obj.center-position,2)-obj.intersectionRange;
            r = distance <= visibilityFieldRange;
        end
        
        function plot(obj)
            %Default plot function for obstacle as set of points
            [m,n]=size(obj.points);
            hold on
            plot3(obj.center(1),obj.center(2),obj.center(3),'Marker','o','MarkerFaceColor',obj.getColor,'MarkerEdgeColor',[0 0 0]);
            for k=1:n
                plot3(obj.points(1,k),obj.points(2,k),obj.points(3,k), 'Marker','o','MarkerFaceColor',obj.getColor,'MarkerEdgeColor',[0 0 0]);
            end
            hold off
        end
        function r=getLogString(obj)
            r='';
        end
    end
    
end

