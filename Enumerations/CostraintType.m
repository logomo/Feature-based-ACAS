classdef CostraintType<uint32
    %CostraintType Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        Static(1)
        Moving(2)
        Soft(3)
        Hard(4)
    end
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

