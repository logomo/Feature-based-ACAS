classdef ObstacleType<uint32
    %OBSTACLETYPE Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        Map(1)
        Detected(2)
    end
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

