classdef FlightLog<LoggableObject
    %FLIGHTLOG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Common information
        simulationTime;
        %Vehicle information
        vehiclePosOrBefore
        executedMovement
        vehiclePosOrAfter
        movementBuffer
        %Mission information
        waypointID
        waypoint
        %Obstacle information
        intersections
        obstacleHits
        staticObstacleFlag
        %Intruder information
        detectedIntruders
        intruderHits
        intruderFlag
        %Mission flags
        waypointReachedFlag
        waypointUnreachableFlag
        finalWaypointReachedFlag
        collisionFlag
        obstacleFlag
        forcerReplaningFlag
    end
    
    methods
        function showMissionStatus(obj)
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

