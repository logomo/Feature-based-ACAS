classdef CostraintType<uint32
    %CostraintType DEfines the type of virtual constraints, use static/moving, soft/hard spec
    enumeration
        Static(1)   %Stay at same spot over time
        Moving(2)   %Move along linear path
        Soft(3)     %Breachable constraint
        Hard(4)     %Unbreachable constraint
    end
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

