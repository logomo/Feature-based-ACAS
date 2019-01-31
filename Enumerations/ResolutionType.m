classdef ResolutionType<uint32
    %RESOLUTIONTYPE Collision Case resolution type sent in resolution message to UAS
    
    enumeration
        StayOnCourse(1),            %UAS has Right of the waym stay on original heading, until avoidance is compled
        RightSideAvoidance(2),      %UAS must avoid collision point from right side, keep velocity
        RoundaboutAvoidance(3),     %UAS must joint and follow roundabout
        EmergencyAvoidance(4),      %UAS is enforced into emergency mode (Every UAS for itself ...)
    end   
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

