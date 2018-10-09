classdef IntersectionConfig
    %INTERSECTIONCONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        flagTimeIntersection = false;    %Enforce time
        flagFutureMovements = false;     %Enforce future movements
        flagBallIntersection = false;    %Enforce ball intersection
        flagSpread = true;               %Use spread
        flagOnlySpread = false;          %DO not Use line intersection
        timeError = 1;                   %time error
        radius = 1;                      %radius of adversary
        thetaSpread = pi/8;              %horizontal spread +- cone definition 0-pi/2
        phiSpread = pi/12;               %vertical spread +- cone definition 
    end
    
    methods
        function obj = IntersectionConfig(time, future, ball,spread,onlySpread)
            if nargin ==0
                % go with default object config
            else
            obj.flagTimeIntersection = time;    
            obj.flagFutureMovements = future;     
            obj.flagBallIntersection = ball;    
            obj.flagSpread = spread;              
            obj.flagOnlySpread = onlySpread;          
            end
        end
    end
end

