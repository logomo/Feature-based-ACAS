classdef VehiclePositionNotification<handle
    %VEHICLEPOSITIONNOTIFICATION refer to section 6.5, the position notification wrapper class 
    
    properties
        %position
        latitude;                                               %X vector in sim enviroment (GCS)
        longitude;                                              %Y vector in sim enviroment (GCS) 
        altitude;                                               %Z vector in sim enviroment (GCS)
        
        %heading 
        orientation;                                            %[alpha,beta,gamma]
        velocity;                                               %[dx,dy,dz] in sim enviroment (GCS)
        
        %flightLevels
        mainFlightLevel=FlightLevel.FL450;                      %Just default flight level for testing
        passingFlightLevel=FlightLevel.FL450;                   %There is no crossing functionality for this prototype
        
        %ICAO categorization
        vehicleCategory=VehicleCategory.UAVAutonomous;          %AutonomousUAV 
        manCat=ManeuverabilityCategory.FullPropulsionGliding;   %Full propulsion enabled no VTOL (Gliding mode)
        %SafetyMargins
        smUniversal=5;                                          % Universal Safety margin [1..1] (m)
        smOvertake=5;                                           % Overtake maneuver margin [0..1] (m)
        smConverging=8;                                         % Converging maneuver margin [0..1] (m)
        smHeadOn=5;                                             % Head On Approach margin [0..1] (m)
        wakeConeDiameter=0;                                     % wake turbulence cone diameter [0..1] (m)
        wakeConeRadius=0*pi;                                    % waje cone spread radius [0..1] (m)
        %flags
        flagMissionActive=true;
        
    end
    
    methods
        function obj=VehiclePositionNotification(pos,or,vel,sm)
            %Constructor methodr
            %   pos - position gloc
            %   or - orientation gloc
            %   vel - velocity gloc
            %   sm - universal safety margin
            obj.latitude=pos(1);                                               
            obj.longitude=pos(2);                                              
            obj.altitude=pos(3);
            obj.orientation=or;
            obj.velocity=vel;
            obj.smUniversal=sm;
        end
        
        
        function r=getPosition(obj)
            % Gets position vector
            %   r = lat/lon/alt [m,m,m] (imperial units only in spec xo xo xo)
            r=[obj.latitude;obj.longitude;obj.altitude];
        end
        
        
        function r=getSafetyMargin(obj,colCat)
            % Gets safety margin
            % IN   colCat CollisionCategory enumeration member
            % OUT  r - safety margin for given situation [m]
            if nargin <2
                colCat = CollisionCategory.Unknown;
            end
            
            %If there is unknown danger, just use te maximum offset
            if colCat==CollisionCategory.Unknown
                r=max(obj.getSafetyMarginVector());
                return;
            end
            
            %This does not reflect original logic (TODO)
            if colCat==CollisionCategory.Overtaking
                r=max(obj.smUniversal,obj.smOvertake);
                return;
            end
            
            %This does not reflect original logic
            if colCat==CollisionCategory.HeadOnApproach
                r=max(obj.smUniversal,obj.smHeadOn);
                return;
            end
            
            %This does not reflect original logic
            if colCat==CollisionCategory.Converging
                r=max(obj.smUniversal,obj.smConverging);
                return;
            end
        end
        
        function r=getSafetyMarginVector(obj)
            % Helper function all safety margins coupled as vector for minmax
            % functions
            %   r - vector [m,m,m,m,m,m,m,m,m,m,....]
            r=[obj.smUniversal;obj.smOvertake;obj.smConverging;obj.smHeadOn;obj.wakeConeDiameter];
        end
    end
    
end

