classdef MazeMatrix<LoggableObject
    %MAZEMATRIX Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mazeMap
        sideLength = 10;
        offsetMargin = 0; %Make the buildings bigger
        safetyMargin = 2.5;
        lastSnapshot=[];
        startPosition=[];
        finalWaypoint=[];
    end
    
    methods
        function obj = MazeMatrix(mazeMap)
            % 0 free space;
            % 1-4 Static constraint
            % 5 - start 
            % 6 - end
            obj.mazeMap = mazeMap;
        end
        
        function r=generateMazeObstacles(obj)
            r=[];
            scale= (obj.sideLength-2*obj.offsetMargin)/2;
            [m,n] = size(obj.mazeMap);
            for k=1:m
                for l=1:n
                    val=obj.mazeMap(k,l);
                    x_off=l-1;
                    y_off=m-k;
                    x_val=x_off*obj.sideLength + obj.sideLength/2;
                    y_val=y_off*obj.sideLength + obj.sideLength/2;
                    if val >=1 && val <=4
                        if val ~= 4 
                            ang=rand*pi;
                        else
                            ang=0;
                        end
                        const=ExamplePolyConstraint(val,[x_val;y_val;0],scale,ang);
                        const.safetyMargin=obj.safetyMargin;
                        r=[r,const];
                    end
                    %start waypoint
                    if val == 5
                        obj.startPosition=[x_val;y_val;0];
                    end
                    %
                    if val == 6
                        obj.finalWaypoint=[x_val;y_val;0];
                    end
                end
            end
            obj.lastSnapshot=r;
        end
        
        function plot(obj)
            if isempty(obj.lastSnapshot)
                obj.lastSnapshot = obj.generateMazeObstacles();
            end
            hold on 
            for cc=obj.lastSnapshot
                cc.plot()
            end
            hold off
        end
    end
end

