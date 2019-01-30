classdef TimedAdversaryVehicle<AbstractAdversaryVehicle
    %TIMEDADVERSARYVEHICLE Time based adversary vehicle for intruder
    %intersection models
    
    properties
        position                % Position XYZ in avoidance grid local coordinates
        velocity                % Velocity XYZ in avoidance grid local coordinates
        radius                  % Radius ball (r_b) of vehicle or Safety Margin (s_m) of unknown adversary
        distanceError           % Assumed time error of vehicle model (real trajectory deviation)
        thetaSpread             % Assumed Horizontal spread
        phiSpread               % Assumed Vertical spread
        intersectionCells=[];   % Line Intersection cells
        timeMovementCells=[];   % Line Intersection matching time criterion
        futureMovementCells=[]; % Future Intersection matching criterion
        ballRadiusCells = [];   % Register pf ball radius cell in case of direct movement intersection
        spreadCells = [];       % Spread cells intersection
        %flags
        flagTimeIntersection = 1; % Use time intersection indicator
        flagFutureMovements = 1;  % Use future movements intersection indicator
        flagBallIntersection = 1; % Use ball intersection indicator
        flagSpread = 1;           % Use spread intersection indicator
        flagOnlySpread =0;        % Use only spread in final grid fusion
        flagCalculated =0;        % Indication if Adversary intersection have been calculated 
    end
    
    methods
        
        function obj=TimedAdversaryVehicle(position,velocity,radius,...
                                           distanceError,thetaSpread,phiSpread)
        	% Object contructor - initializes structures
            %   position - vehlicle XYZ position in Avoidance grid coordinate
            %              frame
            %   velocity - vehicle XYZ velocity in Avoidance grid coordinate
            %              frame
            %   radius - vehicle body radius - if not given set to 0 [m]
            %   distanceError - linear estimator error [m]
            %   thetaSpread - horizontal uncertainity spread [rad]
            %   phiSpread - vertical uncertainity spread [rad]
            obj.position=position;
            obj.velocity=velocity;
            if nargin > 2
                obj.radius=radius;
            else
                obj.radius=0;
            end
            if nargin > 3
                obj.distanceError = distanceError;
            else
                obj.distanceError = 0;
            end
            if nargin >4
                obj.thetaSpread = thetaSpread;
                obj.phiSpread = phiSpread;
            else
                obj.thetaSpread = 0;
                obj.phiSpread = 0;
            end
            
            %Flag setup
            obj.flagSpread = obj.thetaSpread > 0 || obj.phiSpread > 0;      %If there is no spread just forget about it ...
            obj.flagBallIntersection = obj.radius ~=0;                      %If there is no radius just no ball spread ...
        end
        
        
        function r=findIntersection(obj,ag)
            % Findst intersections based on flags in given avoidance grid
            %   ag - avoidance grid object
            %   r - count of hitted intersections
            %   DISCLAIMER - function does not modify AVOIDANCE GRID
            r=0;
            if ~obj.flagCalculated                                          %Test calculation 
                rf=obj.findDirectIntersectionCells(ag);                     % find direct intersection - no time constraint - mandatory step
                r=r+length(rf);
                if obj.flagTimeIntersection || obj.flagFutureMovements      % if intersection is timed
                    [tmc,fmc]=obj.findTimeIntersectionCells;
                    r=r+length(tmc);
                    r=r+length(fmc);
                end
                if obj.flagBallIntersection && obj.flagTimeIntersection     % if ball intersection is active find ball intersection
                    balls=obj.findIntersectionBalls(ag);
                    for b=balls
                        r=r+length(b.coating)+1;
                    end
                end
                
                if obj.flagSpread
                    tst=obj.findIntersectionEllipseCells(ag);
                    r=r+length(tst);
                end
                obj.flagCalculated=1;
            end
        end
        
        
        function r=findDirectIntersectionCells(obj,ag)                       
            % Find direct intersection in avoidance grid 
            %   ag - avoidance grid object
            %   r - list of intersection cells with intersection time
            r=ag.getLineIntersectionNumeric(obj.position,obj.velocity);
            obj.intersectionCells = r;
        end
        
        
        function [timeMovementCells,futureMovementCells]=...
                                        findTimeIntersectionCells(obj)
            % Finds movement intersection in time and future intersections
            %   timeMovementCells - list of cells within boundary
            %   futureMovementCells - list of cells after intersection
            adversaryLinearVelocity=norm(obj.velocity,2);                   %Normalize velocity og linear model
            adversaryTimeError=obj.distanceError;                               %set time error of predictor
            cells=obj.intersectionCells;
            predictionError = adversaryLinearVelocity*adversaryTimeError;   %Predictor error calculation in [m]
            intersectionCells=[];
            futureMovementCells=[];
            for k=1:length(cells);                                          %For each direct intersection celll
                wCell = cells(k);
                arrivalTime = wCell.time;                                   % Arrival time  to celll - main coparison
                cellArrivalTime = wCell.cell.minimalEntryTime;
                cellLeaveTime = wCell.cell.maximalLeaveTime;
                cellIntersection = wCell.cell.toleratedDeviation/adversaryLinearVelocity;   % tolerated cell intersection deviation
                minBoundary = cellArrivalTime - predictionError - cellIntersection;         % minimal cell entry boundary
                maxBoundary = cellLeaveTime + predictionError + cellIntersection;           % maximal cell leave boundary
                maxLeave = cellLeaveTime - predictionError ;                                % maximal cell Leave boundary for future cells
                if arrivalTime >= minBoundary && maxBoundary>=arrivalTime
                    if (Cmnf.debugAdversary)
                        intersection=[minBoundary,arrivalTime,maxBoundary,k]
                    end
                    wCell.probability=1;                                    % Cell probability is set to 1 in case of intersection cell                          
                    intersectionCells=[intersectionCells,wCell];            % cell is registered into intersection cells
                end

                if arrivalTime >= maxLeave
                    if (Cmnf.debugAdversary)
                        futureTrajectory=[arrivalTime,maxLeave,k]
                    end
                    base = wCell.time;
                    cellLeaveTime = wCell.cell.maximalLeaveTime;
                    gain = cellLeaveTime - predictionError;
                    probability =  gain/base;
                    wCell.probability=probability;
                    futureMovementCells=[futureMovementCells,wCell];
                end
            end
            obj.timeMovementCells=intersectionCells;
            timeMovementCells=intersectionCells;
            obj.futureMovementCells=futureMovementCells;
        end
        
        
        function r=findIntersectionBalls(obj,ag)                    
            % Find intersection Balls in direct intersection cells
            %   ag - avoidance grid object 
            %   r - list of IntersectionGridBall objects
            obj.ballRadiusCells = [];
            for wCell = obj.timeMovementCells
                candidates = Cmnf.findCellsInRange(ag,wCell.cell,...
                                                   wCell.position,...
                                                   obj.radius);
                wBall=IntersectionGridBall(wCell,[]);
                for c=candidates
                    if  c.isDirect
                        ci=IntersectionGridCell(c.cell,...
                                                wCell.cell.center,...
                                                wCell.time);
                        ci.probability=1;
                        wBall.coating=[wBall.coating,ci];
                    else
                        max=c.cell.toleratedDeviation + c.checkDistance;
                        min=c.checkDistance;
                        value=c.realDistance;
                        probability=Cmnf.harmonicDistribution(min,max,...
                                                              value);
                        ci=IntersectionGridCell(c.cell,...
                                                wCell.cell.center,...
                                                wCell.time);
                        ci.probability=probability;
                        wBall.coating=[wBall.coating,ci];
                    end
                    if (Cmnf.debugAdversary)
                        c.cell.ijkIndex
                        ci.probability
                    end
                end
                obj.ballRadiusCells=[obj.ballRadiusCells,wBall];
            end
            r=obj.ballRadiusCells;
        end
        
        function r=findIntersectionEllipseCells(obj,ag)
            % Find intersection with avoidance grid for spreac intruder
            % intersection
            intersections=[];
            flagEnter=0;
            flagLeave=0;
            maximumGridRange= 2*(ag.dEnd -ag.dStart);
            distances=0:ag.stepLayer:maximumGridRange;
            for d=distances
                candidates=obj.generateIntersectionEllipse(d,ag.stepLayer);
                selected=Cmnf.intersectPointCloud(ag,candidates);
                if ~isempty(selected)
                    if flagEnter == 0
                        flagEnter=1;
                    end
                    intersections=[intersections,selected];
                else
                    if flagEnter == 1
                        flagLeave = 1;
                        break
                    end
                end
            end
            obj.spreadCells=intersections;
            r=obj.spreadCells;
        end
        
        function r=generateIntersectionEllipse(obj,distance,stepSize) 
            % Generates intersection ellipsoid slice for spread
            % intersection based on slice distance and point step size
            position = obj.position;
            velocity=  obj.velocity;
            thetaSpread=obj.thetaSpread;
            phiSpread=obj.phiSpread;
            debug=0;

            % Generate covariant elyptical slice on  direct distance
            a=sin(thetaSpread)*distance;                                
            b=sin(phiSpread)*distance;                                  
            flag = 1;
            r=[];                                                  
            while flag == 1 || a >= stepSize || b >= stepSize
                cira = 2*pi*a;
                cirb = 2*pi*b;
                cir = (cira+cirb)/2;
                stepCount = round(cir/stepSize);                            %calculate step count according to numeric aproximator
                if stepCount < Cmnf.eliSteps
                    stepCount = Cmnf.eliSteps;
                end
                if a==0 && b ==0
                    stepCount = 1;
                end
                flag = 0;
                t=linspace(0,2*pi,stepCount);
                elip = [distance*ones(1,stepCount);a*cos(t);b*sin(t)];      %generate proto elypse for given a,b params
                if debug == 1
                    hold on
                        figure(1)
                        plot3(elip(1,:),elip(2,:),elip(3,:),'b');
                    hold off
                end
                r=[r, elip];
                if (a-stepSize) >= stepSize && (b-stepSize) >= stepSize
                    a=a-stepSize;
                    b=b-stepSize;
                else if (a-stepSize) >= stepSize
                        a=a-stepSize;
                    else if (b-stepSize) >= stepSize
                            b=b-stepSize;
                        else break;
                        end
                    end    
                end
            end
            r= [r,[distance;0;0]];
            % calculate probabilistic distribution with appeal on
            % centristic approach
            n=r(2:3,:);                                                     %standard random distributuin initialization
            covh=n(1,:)*(n(1,:)')/(length(n)-1);                            %covariance of x parameters
            covv=n(2,:)*(n(2,:)')/(length(n)-1);                            %covariance of y parameters
            pdfh=normpdf(n(1,:),0,sqrt(covh));                              %probability density function horizontal
            pdfv=normpdf(n(2,:),0,sqrt(covv));                              %probability density function vertical
            %sanity check for NaN if zero distribution
            for k = 1:length(pdfh)                                          %pdf for zero covariance == inf
                if (isnan(pdfh(k)))
                    pdfh(k) = 1;
                end
                if (isnan(pdfv(k)))
                    pdfv(k) = 1;
                end
            end
            pdf = (pdfh + pdfv)/2;                                          %use mean for horizontal+vertical pdf
            r=[r;pdf];
            %Normalize the results TODO decide if its necessary
            %r(4,:)=r(4,:)./sum(r(4,:));
            if debug==1
                hold on
                    figure(2);
                    plot3(r(2,:),r(3,:),r(4,:));
                hold off 
            end
            % calculate time of arival for each point
            velocityNormalized = norm(velocity,2);                               % time of arival for each point at given velocity
            [m,n]=size(r');
            tim = zeros(1,m);
            for k= 1:m
                tim(k)= norm(r(1:3,k))/velocityNormalized;
            end
            r=[r;tim];
            if debug ==1
                hold on
                    figure(3)
                    plot3(r(2,:),r(3,:),r(5,:));
                hold off
            end
            % rotation 
            mat = r(1:3,:);                                                 %standard rotation and offseting for global coordinates
            mat = Cmnf.align3Dvec(velocity,mat);
            % offsetting
            vec = position;
            mat = Cmnf.ofst(vec,mat);
            % merginf final result
            r(1:3,:)=mat;                                                  
            if debug ==1
                hold on
                    figure(4)
                    plot3(r(1,:),r(2,:),r(3,:));
                hold off
            end
        end
    end
    
end

