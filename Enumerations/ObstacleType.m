classdef ObstacleType<uint32
    %OBSTACLETYPE Type of AbstractObstacle and subclasses 
    
    enumeration
        Map(1)      %This obstacle is handled as a Map sourced obstacle
        Detected(2) %This obstacle is handled as a Real physical obstacle
    end
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

