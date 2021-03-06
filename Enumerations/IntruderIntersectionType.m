classdef IntruderIntersectionType < uint32
    %IntruderIntersectionType Type of intruder intersection with avoidance grid
    enumeration
        Time(0),              %Use time of arival intersection mode
        FutureMovements(1),   %Use future movements prediction (Enforces behind craft avoidance)
        Ball(2),              %consider body volume in intersection, needs a parameter in map
        Spread(3),            %Consider manuevurability uncertainity
        OnlySpread(4),        %Neglect other behaviour except manuevurability
        NonCooperative(5),    %Spread+body+future movements - worst case intersection
        Cooperative(6),       %Line+body+timed - avearage case intersection
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
        
        function r=getIntersectionConfig(enums)
            %Generates Intersection Config object based on options listing,
            %   enums - enumeration member unique list [1..*]
            r = IntersectionConfig();
            if nargin ~= 0
                r.flagTimeIntersection = false;    %Enforce time
                r.flagFutureMovements = false;     %Enforce future movements
                r.flagBallIntersection = false;    %Enforce ball intersection
                r.flagSpread = false;              %Use spread
                r.flagOnlySpread = false;          %DO not Use line intersection
                for enum=enums
                    if enum==IntruderIntersectionType.Time
                        Cmnf.logc(r,'Config: timed interesection enabled');
                        r.flagTimeIntersection = 1;
                    end
                    if enum == IntruderIntersectionType.FutureMovements
                        Cmnf.logc(r,'Config: future movements interesection enabled');
                        r.flagFutureMovements = 1;
                    end
                    if enum == IntruderIntersectionType.Ball
                        Cmnf.logc(r,'Config: ball interesection enabled');
                        r.flagBallIntersection = 1;
                    end
                    if enum == IntruderIntersectionType.Spread
                        Cmnf.logc(r,'Config: uncertaininty spread interesection enabled');
                        r.flagSpread=1;
                    end
                    if enum == IntruderIntersectionType.OnlySpread
                        Cmnf.logc(r,'Config: line intersection supressed under spread');
                        r.flagOnlySpread=1;
                    end
                    if enum == IntruderIntersectionType.NonCooperative
                        r.flagTimeIntersection = 0;    %No time
                        r.flagFutureMovements = 0;     %No future movements
                        r.flagBallIntersection = 0;    %No body volume intersection
                        r.flagSpread = 1;              %Use spread
                        r.flagOnlySpread = 0;          %Linear is not supressed
                    end
                    if enum == IntruderIntersectionType.Cooperative
                        r.flagTimeIntersection = 1;    %Enforce time (Know velocity)
                        r.flagFutureMovements = 0;     %No future movements
                        r.flagBallIntersection = 1;    %Enforce body volume (known s_m)
                        r.flagSpread = 0;              %Use spread
                        r.flagOnlySpread = 0;          %Linear is not supressed
                    end
                end
            end
        end
        
        function r=isComplex(enum)
            r=IntruderIntersectionType.Cooperative == enum ||...
                IntruderIntersectionType.NonCooperative == enum;
        end
        
    end
   
end

