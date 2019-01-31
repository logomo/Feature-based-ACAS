classdef CollisionCategory<uint32
    %CollisionCategory CAtegorization of supported collision types see UTM/CollisionCase
    
    enumeration
        HeadOnApproach(1)   %Head on approach
        Overtaking(2)       %Overtake manuever
        Converging(3)       %Converging
        Unknown(4)          %Error State
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

