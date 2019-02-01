classdef FlightLog<LoggableObject
    %FLIGHTLOG Wrapper class to log flight status over mission control timeframes
    
    properties
        %Common information
        simulationTime;             % Time of the simulation
        %Vehicle information
        vehiclePosOrBefore          % Before movement applicaiton pos XYZ  GCF
        executedMovement            % Movement type applied this frame
        vehiclePosOrAfter           % After movement application pos XYZ GCF
        movementBuffer              % State of movement buffer [list of existing movements]
        %Mission information
        waypointID                  % Actual goal waypoint ID
        waypoint                    % Actual goal XYZ GCF
        %Obstacle information
        intersections               % static obstacle intersected cells
        obstacleHits                % count of lidar hits
        staticObstacleFlag          % indication if static obstacle was encountered in this frame
        %Intruder information       
        detectedIntruders           % list of detected intruders
        intruderHits                % List of cells where intruders can intersect
        intruderFlag                % indicaiton if intruder was entcountered this time frame
        %Mission flags
        waypointReachedFlag         % indicaiton if goal waypoint has been reached this time
        waypointUnreachableFlag     % indication if goal waypoint has been marked as unreachable this time frame
        finalWaypointReachedFlag    % indicaiton if last mission waypoint has been reached this time frame
        collisionFlag               % indication if collision happened this time frame
        obstacleFlag                % indication if obstacle collision happened in this frame
        forcerReplaningFlag         % indicaiton if event forcing replaning of trajectory was raised
    end
    
    methods
        function showMissionStatus(obj)
            % Formatted console output to show mission status
            %   - usage for debug,
            %   - refer to Cmnf.logc function
            Cmnf.logc(obj,['Mission status time: ',mat2str(obj.simulationTime),' s']);
            %Vehicle information
            Cmnf.logc(obj,'>> Vehicle status:');
            Cmnf.logc(obj,['    -position start:    ',mat2str(obj.vehiclePosOrBefore),' [m,m,m]']);
            Cmnf.logc(obj,['    -movements:         ',Cmnf.movementBuffer2String(obj.movementBuffer)]);
            Cmnf.logc(obj,['    -executed movement: ',Cmnf.movementBuffer2String(obj.executedMovement)]);
            Cmnf.logc(obj,['    -position end:      ',mat2str(obj.vehiclePosOrAfter),' [m,m,m]']);
            
            %Mission information
            Cmnf.logc(obj,'>> Mission status:');
            Cmnf.logc(obj,['    -wapointID: ',mat2str(obj.waypointID)]);
            Cmnf.logc(obj,['    -waypoint:  ',mat2str(obj.waypoint),' [m,m,m]']);
            
            %Obstacle information
            Cmnf.logc(obj,'>> Obstacle status:');
            Cmnf.logc(obj,['    -in range: ',mat2str(length(obj.intersections))]);
            Cmnf.logc(obj,['    -hits:     ',mat2str(obj.obstacleHits)]);
            Cmnf.logc(obj,['    -flag:     ',Cmnf.bolleanString(obj.staticObstacleFlag)]);
            
            %Intruder information
            Cmnf.logc(obj,'>> Intruder status:');
            Cmnf.logc(obj,['    -active: ',mat2str(length(obj.detectedIntruders))]);
            Cmnf.logc(obj,['    -hits:   ',mat2str(obj.intruderHits)]);
            Cmnf.logc(obj,['    -flag:   ',Cmnf.bolleanString(obj.intruderFlag)]);
            %Mission flags
            Cmnf.logc(obj,'>> Misison flags:');
            Cmnf.logc(obj,['    -waypoint reach:   ',Cmnf.bolleanString(obj.waypointReachedFlag)]);
            Cmnf.logc(obj,['    -waypoint hidered: ',Cmnf.bolleanString(obj.waypointUnreachableFlag)]);
            Cmnf.logc(obj,['    -mission finished: ',Cmnf.bolleanString(obj.finalWaypointReachedFlag)]);
            Cmnf.logc(obj,['    -collission[!!]:   ',Cmnf.bolleanString(obj.collisionFlag)]);
            Cmnf.logc(obj,['    -obstacle:         ',Cmnf.bolleanString(obj.obstacleFlag)]);
            Cmnf.logc(obj,['    -replaning:        ',Cmnf.bolleanString(obj.forcerReplaningFlag)]);
        end
    end
    
end

