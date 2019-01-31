classdef FlightLevel<uint32
    %FLIGHTLEVEL Flight level enumeration, contains only few for UTM/CollisionCase showcasing
    
    enumeration
        FL460(46000), %Level abowe
        FL450(45000), %Sample working level
        FL440(44000), %Level below
    end   
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

