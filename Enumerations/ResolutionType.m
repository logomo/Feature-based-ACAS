classdef ResolutionType<uint32
    %RESOLUTIONTYPE Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        StayOnCourse(1),
        RightSideAvoidance(2),
        RoundaboutAvoidance(3),
        EmergencyAvoidance(4),
    end   
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

