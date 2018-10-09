classdef BarrelConstraint<AbstractConstraint
    %BARRELCONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        startHeight = -5
        endHeight = 5
        plotStartHeight = -5
        plotEndHeight = -5
        radius = 0
    end
    
    methods
        function obj = BarrelConstraint(center,radius, sh,eh,psh,peh)
            [m,n]=size(center);
            if m == 2 || n == 2
                if n == 2
                    center = [center,0]';
                else
                    center = [center;0];
                end
            end
            obj.center = center;
            obj.radius = radius;
            obj.type = CostraintType.Static;
            if nargin < 3
                obj.startHeight = -5;
            else
                obj.startHeight = sh;
            end
            if nargin < 4
                obj.endHeight = 5;
            else
                obj.endHeight = eh;
            end
            
            if nargin < 5
                obj.plotStartHeight = obj.startHeight;
                obj.plotEndHeight = obj.endHeight;
            else
                obj.plotStartHeight = psh;
                obj.plotEndHeight = peh;
            end
            
            if obj.plotStartHeight == -inf 
                obj.plotStartHeight = -5;
            end
            
            if obj.plotEndHeight == inf
                obj.plotEndHeight = 5;
            end
        end
        
        %override in geral class
        function r=isIntersection(obj,posor,mc,node)
            lcenter=mc.getLocalCoordiantes(obj.center);
            % if constraint is moving we need to use time intersection
            if obj.type == CostraintType.Moving
                gvelocity = obj.velocityVector + mc.vehicle.actualPositionOrientation(1:3);
                lvelocity = mc.getLocalCoordiantes(gvelocity);
                futureTime =max(node.state(7,:));
                % bring constraint barrel close based on time 
                posor=posor(1:3)-(lvelocity*futureTime);
            end
            llow=mc.getLocalCoordiantes([obj.center(1:2);obj.startHeight]);
            lup=mc.getLocalCoordiantes([obj.center(1:2);obj.endHeight]);
            xy=posor(1:2);
            z =posor(3);
            horizontalRange = obj.safetyMargin + obj.radius;
            xyFlag = sqrt(sum((xy - lcenter(1:2)).^2)) <= horizontalRange;
            zFlag= llow(3) <= z && lup(3) >= z;
            r = xyFlag && zFlag;
        end
        
        % override in general class
        function r=inRange(obj,mc)
            posor = mc.vehicle.getActualPositionOrientation;
            xy = posor(1:2);
            z = posor(3);
            gridRange = max([mc.avoidanceGrid.dEnd,mc.navigationGrid.dEnd]);
            horizontalRange = obj.safetyMargin + obj.radius +gridRange;
            verticalRange = gridRange;
            if obj.type == CostraintType.Moving
                horizontalRange = horizontalRange + 2*norm(obj.velocityVector);
                verticalRange = verticalRange + 2*norm(obj.velocityVector);
            end
            % calculation
            % xy condition
            xyDist=sqrt(sum((obj.center(1:2) - xy).^2));
            xyFlag= xyDist<=horizontalRange;
            
            % z condition
            inRangeFlag = obj.startHeight <=z && obj.endHeight >= z;
            bottomFlag = sqrt(sum(([obj.center(1:2);obj.startHeight]-posor(1:3)).^2)) <= verticalRange;
            topFlag = sqrt(sum(([obj.center(1:2);obj.endHeight]-posor(1:3)).^2)) <= verticalRange;
            zFlag = inRangeFlag || bottomFlag || topFlag;
            r= xyFlag && zFlag;
        end
        
        function handles=plot(obj)
           o = obj.radius*2*pi; 
           steps = round(o/Cmnf.obstaclePrecision);
           % protocyrcle = [[0;0],[obj.radius*cos(linspace(-pi,pi,steps));obj.radius*sin(linspace(-pi,pi,steps))]];
           % upper = [protocyrcle; obj.plotEndHeight*ones(1,steps+1)] + obj.center*ones(1,steps+1);
           % lower = [protocyrcle; obj.plotStartHeight*ones(1,steps+1)] + obj.center*ones(1,steps+1);
           %P=[upper,lower]';
           hold on
           %warning on
           %shp = alphaShape(P);
           %plot(shp,'FaceColor', obj.getColor,'EdgeColor', obj.getColor);
           %plot3(upper(1,:),upper(2,:),upper(3,:),'k');
           %plot3(lower(1,:),lower(2,:),lower(3,:),'k');
           %warning off
           protocyrcle = [[0;0],[obj.radius*cos(linspace(-pi,pi,steps));obj.radius*sin(linspace(-pi,pi,steps))]];
           upper = [protocyrcle; obj.plotEndHeight*ones(1,steps+1)] + obj.center*ones(1,steps+1);
           lower = [protocyrcle; obj.plotStartHeight*ones(1,steps+1)] + obj.center*ones(1,steps+1);
           P=[];
           for k= 2:steps+1
               P=[P,upper(:,k),lower(:,k)];
           end
           %P=[P,upper,lower];
           P=P';
           k = boundary(P);            
           h1=trisurf(k,P(:,1),P(:,2),P(:,3),'FaceColor', obj.getColor,'EdgeColor', obj.getColor,'FaceAlpha',1);
           P=[upper,lower];
           P=P';
           k = boundary(P);            
           h2=trisurf(k,P(:,1),P(:,2),P(:,3),'FaceColor', obj.getColor,'EdgeColor', obj.getColor,'FaceAlpha',1);
           h3=plot3(upper(1,2:steps+1),upper(2,2:steps+1),upper(3,2:steps+1),'k');
           h4=plot3(lower(1,2:steps+1),lower(2,2:steps+1),lower(3,2:steps+1),'k');
           handles=[h1,h2,h3,h4];
           hold off
        end
        
        function r=getLogString(obj)
            r=['radius: ',mat2str(obj.radius),', start height: ',mat2str(obj.startHeight),', end height: ',mat2str(obj.endHeight)];
        end
    end
end

