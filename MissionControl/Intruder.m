classdef Intruder<handle
    %INTRUDER Mission control non cooperative intruder data structure (ADS-B like message)
    
    properties
        id                                      %  identification number of intruder;
        localPosition                           %  local position at detection time
        localVelocity                           %  local velocity at detection time
        posTime                                 %  [global position;time] Matrix
        detectionTime                           %  time when intruder was detected 
        liveTime=16                             %  time to live standard 20s
        cooperative = false;                    % indication if intruder is cooperative
        intersectionConfig=IntersectionConfig   % Default non cooperative Intersection configuration
    end
    
    methods
        function obj=Intruder(localPosition,localVelocity,detectionTime)
            %Constructor to create object
            obj.localPosition=localPosition;
            obj.localVelocity=localVelocity;
            obj.detectionTime=detectionTime;
        end
        
        %flagFunction 
        function r=isFirstDetection(obj,simulationTime)
            %Check if intruder is known or it was detected first time
            r=obj.detectionTime == simulationTime;
        end
        
        function r=isDetected(obj,simulationTime)
            %Check if intruder has been detected by sensor
            mint =obj.detectionTime;
            maxt =obj.detectionTime+obj.liveTime;
            r= mint<=simulationTime && maxt >= simulationTime;
        end
        
        %singleIntruderPlot
        function plotTimedTrajectory(obj,simTime)
            %Plot function for mission control enviroment
            posTime=obj.posTime;
            time=posTime(4,:);
            indexes = find(time<=simTime);
            plotPos=posTime(1:3,indexes);
            [m,n]=size(plotPos);
            hold on 
            plot3(plotPos(1,:),plotPos(2,:),plotPos(3,:),'m');
            hold off

            hold on 
            for k=1:n

                if k==n
                    c='r';
                else
                    c= 'm';
                end

                plot3(plotPos(1,k),plotPos(2,k),plotPos(3,k), 'Marker','o','MarkerFaceColor',c,'MarkerEdgeColor',c);
            end
            hold off
        end
    end
    
end

