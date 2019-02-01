classdef ObstacleSphere<AbstractObstacle
    %OBSTACLESPHERE Sphere obstacle
    
    properties
        radius; % radius in meters
    end
    
    methods
        function obj = ObstacleSphere(center,radius,type)
            %Constructor for sphere obstacle,
            %   center - XYZ GCF
            %   radius - meters
            %   type - ObstacleType enuemration member
            
            if nargin <=2
                type=ObstacleType.Detected;
            end
            obj.type=type;
            obj.center=center;
            obj.collisionRange=radius;
            obj.intersectionRange=radius;
            obj.radius=radius;
            o=2*pi*radius;
            steps = round(o/Cmnf.obstaclePrecision);
            protocyrcle = [radius*cos(linspace(-pi,pi,steps));radius*sin(linspace(-pi,pi,steps))];
            protoball= [protocyrcle(1,:),protocyrcle(1,:),zeros(1,steps);
                        protocyrcle(2,:),zeros(1,steps),protocyrcle(1,:);
                        zeros(1,steps),protocyrcle(2,:),protocyrcle(2,:)];
            obj.points = [protoball];
            %figure(1)
            %plot3(protoball(1,:),protoball(2,:),protoball(3,:),'r*');
            steps = round((pi*radius)/Cmnf.obstaclePrecision);
            steps = linspace(-pi/2,pi/2,steps);
            radiuses = (radius*cos(steps));
            zoffset=radius*sin(steps);
            l= length(radiuses);
            %figure(2)
            %hold on
            for k=1:l
                steps = round(2*pi*radiuses(k)/Cmnf.obstaclePrecision);
                protocyrcle = [radiuses(k)*cos(linspace(-pi,pi,steps));radiuses(k)*sin(linspace(-pi,pi,steps))];
                ofpcyrcle = [protocyrcle;zoffset(k)*ones(1,steps)];
                obj.points = [obj.points,ofpcyrcle];
                %plot3(ofpcyrcle(1,:),ofpcyrcle(2,:),ofpcyrcle(3,:),'r*');
            end
            %hold off
            obj.points = obj.points + obj.center*ones(1,length(obj.points));
            %figure(3)
            %plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),'r*');
        end
        
        function r=getLogString(obj)
            %[Override] Creates logString  for logging functions
            r=[' radius: ',mat2str(obj.radius),' m'];
        end
    end
    
end

