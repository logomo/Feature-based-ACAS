classdef VehiclePositionNotification<handle
    %VEHICLEPOSITIONNOTIFICATION Summary of this class goes here
    %   Detailed explanation goes here
    
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
        %Constructor methodr
        %   pos - position gloc
        %   or - orientation gloc
        %   vel - velocity gloc
        %   sm - universal safety margin
        function obj=VehiclePositionNotification(pos,or,vel,sm)
            obj.latitude=pos(1);                                               
            obj.longitude=pos(2);                                              
            obj.altitude=pos(3);
            obj.orientation=or;
            obj.velocity=vel;
            obj.smUniversal=sm;
        end
        
        % Gets position vector
        %   r = lat/lon/alt [m,m,m] (imperial units only in spec xo xo xo)
        function r=getPosition(obj)
            r=[obj.latitude;obj.longitude;obj.altitude];
        end
        
        % Gets safety margin
        % IN   colCat CollisionCategory enumeration member
        % OUT  r - safety margin for given situation [m]
        function r=getSafetyMargin(obj,colCat)
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
        % Helper function all safety margins coupled as vector for minmax
        % functions
        %   r - vector [m,m,m,m,m,m,m,m,m,m,....]
        function r=getSafetyMarginVector(obj)
            r=[obj.smUniversal;obj.smOvertake;obj.smConverging;obj.smHeadOn;obj.wakeConeDiameter];
        end
    end
    
end

