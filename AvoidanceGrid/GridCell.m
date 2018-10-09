classdef GridCell < handle
    %GRIDCELL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Cell boundary
        dStart
        dEnd
        thetaStart
        thetaEnd
        phiStart
        phiEnd
        
        % Probability for one cell
        pReachability % agregated reachibility
        pVisibility   % agregated visibility
        pObstacle     % agregated obstacle probability
        pDecision     % agregated decision rating
        pFeasibility  % final feasibility
        pAdversary    % probability of adversary type AdversaryVehicle TimedAdversaryVehicle
        rReachability % rule rating for reachibility
        rVisibility   % rule rating for visibility
        rObstacle     % rule rating for obstacle probability
        rDecision     % rule rating for decision rating
        pDetectedObstacle = [];
        pHinderedVisibility = [];
        pMapObstacle = [];
        % Cell center (according to general transformation to XYZ)
        center
        % Reference parameters
        ijkIndex
        layerIndex
        horizontalIndex
        verticalIndex
        parrentGrid
        parrentLayer
        %Graphic data
        plotData=0
        %Trajectories
        trajectories=[];
        arrivalTimes=[];
        minimalEntryTime=0;
        maximalLeaveTime=0;
        toleratedDeviation=0;
        %CellSurface calculation
        outerCellSurface=0;
        innerCellSurface=0;
    end
        
    methods
        %% Constructor
        function obj=GridCell(ds,de,ts,te,ps,pe)
            if (nargin ~=0)
                %initialize properties
                obj.dStart=ds;
                obj.dEnd=de;
                obj.thetaStart=ts;
                obj.thetaEnd=te;
                obj.phiStart=ps;
                obj.phiEnd=pe;
                %initial probability distribution
                obj.pReachability=1;
                obj.pVisibility=1;
                obj.pObstacle=0;
                obj.pDecision=1;
                obj.pAdversary = [];
                %calculate center
                d_avg = (ds+de)/2;
                theta_avg = (ts+te)/2;
                phi_avg = (ps+pe)/2;
                obj.center = Cmnf.plan2euc(d_avg,theta_avg,phi_avg);
            end
        end
        
        function resetCell(obj)
            obj.pReachability=~isempty(obj.trajectories);
            obj.pVisibility=1;
            obj.pObstacle=0;
            obj.pDecision=1;
            obj.rReachability=1;
            obj.rVisibility=1;
            obj.rObstacle=1;     
            obj.rDecision=1;
            obj.pAdversary = [];
            obj.pDetectedObstacle = [];
            obj.pHinderedVisibility = [];
            obj.pMapObstacle = [];
        end
        %% Probabilistic calculation
        function r=recalculateProbabilities(obj)
            obj.recalculateObsbtacleAndVisibilityProbability;
            obj.pObstacle = (max([obj.recalculateAdversarySpreadProbability,...
                                 obj.pObstacle]))*obj.rObstacle;
            obj.pFeasibility=(1-obj.pObstacle)*obj.pReachability * obj.pVisibility;
            r=[obj.pFeasibility,obj.pObstacle,obj.pReachability,obj.pVisibility];
        end
        
        function recalculateObsbtacleAndVisibilityProbability(obj)
            detectedObstacles=obj.pDetectedObstacle;
            hinderedVisibility=obj.pHinderedVisibility;
            mapObstacles=obj.pMapObstacle;
            
            %calculate visibility
            [m,n]=size(hinderedVisibility);
            if n==0
                obj.pVisibility=1;
            else
                % various cell have probability of obstacle 0.3,0.4 ...
                % visibility = 1- all previous cells obstacle probability
                totalHindrance = sum(hinderedVisibility(2,:));
                obj.pVisibility= (1-totalHindrance)*obj.rVisibility;
                if obj.pVisibility <=0 %sanity check
                    obj.pVisibility =0;
                end
            end
            % calculate detected obstacle 
            pDetectedObstacle=0;
            [m,n]=size(detectedObstacles);
            if n > 0
                pDetectedObstacle=sum(detectedObstacles(2,:));
                if pDetectedObstacle >=1
                    pDetectedObstacle=1; % sanity check
                end
            end
            % calculate map obstacle
            pMapObstacle=0;
            [m,n]=size(mapObstacles);
            if n>0
                pMapObstacle = sum(mapObstacles(2,:))
                if pMapObstacle >=1
                    pMapObstacle=1;
                end
                pMapObstacle=pMapObstacle*(1-obj.pVisibility);
            end
            %set final pObstacle value
            obj.pObstacle=max([pMapObstacle,pDetectedObstacle]);
        end
            
        function r=recalculateAdversarySpreadProbability(obj)
            [m,n]= size(obj.pAdversary);
            if n >0
                r = max(obj.pAdversary(4,:));% max of adversary probabilities
            else
                r=0;
            end
        end
        
        function r=recalculateReachibility(obj)
            if isempty(obj.trajectories)
                obj.pReachability=0;
            else
                res = [];
                for k=1:length(obj.trajectories)
                    res=[res,obj.trajectories(k).pReachibility];
                end
                obj.pReachability=max(res);
            end
            r=obj.pReachability*obj.rReachability;
        end
        %% Plot data gathering
        %Probabilistic cell plot
        function r=buildPlotData(obj)
            
            %outer arc
            arclen= (2*pi*obj.dEnd) * ((obj.thetaEnd-obj.thetaStart)/(2*pi));
            arcstc= round(arclen/Cmnf.precision);
            if  arcstc < Cmnf.steps
                arcstc = Cmnf.steps;
            end
            arcstp= linspace(obj.thetaStart,obj.thetaEnd,arcstc);
            huparc = [obj.dEnd*cos(arcstp);obj.dEnd*sin(arcstp)];
            %huparc = Cmnf.rot2D(pi/2,huparc);
            
            %inner arc
            if(obj.dStart~=0)
                arclen= (2*pi*obj.dStart) * ((obj.thetaEnd-obj.thetaStart)/(2*pi));
                arcstc= round(arclen/Cmnf.precision);
                if  arcstc < Cmnf.steps
                    arcstc = Cmnf.steps;
                end
                arcstp= linspace(obj.thetaStart,obj.thetaEnd,arcstc);
                hdownarc = [obj.dStart*cos(arcstp);obj.dStart*sin(arcstp)];
                %hdownarc = Cmnf.rot2D(pi/2,hdownarc);
            else
                hdownarc = [0;0];
            end
            
            % build fill arc for horizontal data
            huparc = huparc(:,linspace(length(huparc),1,length(huparc)));
            hfill = [hdownarc,huparc];
            %fill(hfill(1,:),hfill(2,:),[0.6,.6,.6])
            
            %outer arc
            arclen= (2*pi*obj.dEnd) * ((obj.phiEnd-obj.phiStart)/(2*pi));
            arcstc= round(arclen/Cmnf.precision);
            if  arcstc < Cmnf.steps
                arcstc = Cmnf.steps;
            end
            arcstp= linspace(obj.phiStart,obj.phiEnd,arcstc);
            vuparc = [obj.dEnd*cos(arcstp);obj.dEnd*sin(arcstp)];
            vuparc = Cmnf.rot2D(pi/2,vuparc);
            
            %inner arc
            if(obj.dStart~=0)
                arclen= (2*pi*obj.dStart) * ((obj.phiEnd-obj.phiStart)/(2*pi));
                arcstc= round(arclen/Cmnf.precision);
                if  arcstc < Cmnf.steps
                    arcstc = Cmnf.steps;
                end
                arcstp= linspace(obj.phiStart,obj.phiEnd,arcstc);
                vdownarc = [obj.dStart*cos(arcstp);obj.dStart*sin(arcstp)];
                vdownarc = Cmnf.rot2D(pi/2,vdownarc);
            else
                vdownarc = [0;0];
            end
            % build fill arc for vertical data
            vuparc = vuparc(:,linspace(length(vuparc),1,length(vuparc)));
            vfill = [vdownarc,vuparc];
            %fill(vfill(1,:),vfill(2,:),[1,0,0])
            %hold off
            %hfill(1,:)=-hfill(1,:);                                         %axis orientation in plot fix 
            %vfill(1,:)=-vfill(1,:);
            r=GridCellPlotData(hfill,vfill,0);
        end
        
        function r=getPlotData(obj)
            if ishandle(obj.plotData) && ~strcmp(get(obj.plotData,'type'),'GridCellPlotData')
                obj.plotData = obj.buildPlotData;
            end
            obj.recalculateProbabilities;
            obj.plotData.probabilities = [obj.pReachability,obj.pVisibility,obj.pObstacle,obj.pDecision,obj.pFeasibility];
            r=obj.plotData;
        end
        %% Trajectory register
        function r=calculateArrivalTime(obj)
            if ~isempty(obj.trajectories)
                traj = obj.trajectories;
                cnt = length(traj);
                r= zeros(2,cnt);
                for k = 1:cnt
                    r(:,k)=traj(k).calculateTrajectoryEntryLeave;
                end
                obj.arrivalTimes=r;
                obj.minimalEntryTime=min(r(1,:));
                obj.maximalLeaveTime=max(r(2,:));
            end
        end
        
        function r=registerTrajectory(obj,traj)
            r=traj;
            obj.trajectories=[obj.trajectories,traj];
        end
        
        function r=expand(obj,farCount,nearCount)
            
            if length(obj.trajectories) > 0
                dist = [];
                straightLine=0;
                for k=1:length(obj.trajectories)
                    dist=[dist,obj.trajectories(k).cellDistance];
                    if obj.trajectories(k).directTrajectoryFlag
                        straightLine=obj.trajectories(k);
                    end
                end
                fc = min([farCount, length(dist)]);
                nc = min([nearCount, length(dist)]);
                [sortedX,sortingIndices] = sort(dist,'descend');
                nearCandidates = obj.trajectories(sortingIndices(1:nc));
                [sortedX,sortingIndices] = sort(dist);
                farCandidates = obj.trajectories(sortingIndices(1:fc));
                candidates =  [nearCandidates, farCandidates];
                if straightLine ~=0
                    candidates =[candidates,straightLine];
                end
                r= [candidates]; 
                [obj.ijkIndex];
                [fc,nc];
            else
                r= [];
            end
        end
        
        function r=expandRapid(obj)
            r=[];
            if length(obj.trajectories) > 0
                for k=0:8
                    r=[r, obj.findBestExpandCandidate(k)];
                end
            else
                r= [];
            end
        end
        
        function r=findBestExpandCandidate(obj,mt)
            if length(obj.trajectories) > 0
                index = [];
                value = [];
                for k=1:length(obj.trajectories)
                    test=obj.trajectories(k);
                    if test.movement == mt
                        index =[index,k];
                        value = [value,test.cellDistance];
                    end
                end
                if length(value)> 0
                    if mt == 0
                        [x,y]=sort(value);
                    else
                        [x,y]=sort(value,'descend');
                    end
                    r = obj.trajectories(index(y(1)));
                    return;
                else
                    r=[];
                    return;
                end
            end
            r=[];
        end
        
        % Expand ACAS style for trajectories in cell
        %   obj - self reference
        %   lm - linearized model for predictor
        %   separations - list of available MovementGroups
        function r=expandACAS(obj,lm,separations)
            r=[];
            % First just all candidates
            % candidates=obj.trajectories;
            
            % Harmonic expansion
            %for k=0:8
            %     candidates=[candidates, obj.findBestExpandCandidate(k)];
            %end
            
            % Similar hash (Coverage is priority)
            candidates =[];                                                 % Create candidate list
            if ~isempty(obj.trajectories)                                   % check count of trajectories
                hashes=[];
                for candidate=obj.trajectories                              % gather all hash footprints from belonging trajectories
                    hashes=[hashes,string(candidate.hash)];         
                end
                
                %uhashes=unique(hashes);
                % UNIQUE DOES NOT WORK ON STRING SETS
                nHashes=[];                                                 %workaroud - unique for string set implementation
                for uhash=hashes                                            % expected load ~300 hashes bouble comparator will work fine ...
                    existant =0;
                    for com=nHashes
                        if strcmp(com,uhash)
                            existant = existant + 1;
                        end
                    end
                    if existant == 0 % Unique hash occured
                        nHashes=[nHashes,uhash];
                    end
                end
                
                for nhash = nHashes                                         % For set of unique trajectory footrpints (nHashes)
                    for candidate=obj.trajectories
                        if strcmp(nhash,candidate.hash) 
                            candidates=[candidates,candidate];
                        end
                    end
                end
                
                mtypes=[];
                for candidate=obj.trajectories
                    mtypes = [candidate.movement]; 
                end
                for mtype=mtypes
                    for candidate=obj.trajectories
                        if candidate.movement == mtype
                            candidates=[candidate,candidates];
                            break;
                        end
                    end
                end
                
                % Always prefer straight movement path expansion
                for candidate=obj.trajectories
                    if candidate.isACASPreffered
                        candidates=[candidate,candidates];
                    end
                end
            end
            
            if length(candidates) > 0                                       % Check working candidates if there are no candidates to expansion, there is no reason to run this costy part of code
                movements=[];
                for separation=separations                                  % Gather movements for each type of separation enabled
                    movements=[movements,MovementGroup.getMovementGroupMembers(separation)];               
                end
                movements=unique(movements);                                % Make unique list, there is no reason to keep multiple listings
                movements=sort(movements);
                restriction = min(length(candidates),...                    % Calculate restrictions, restriction is minimum of candidates count
                    round(Cmnf.acasSplitRatio*length(movements)));          % and Possible applicable movement types given by ACAS ratio
                for k=1:restriction                                         % For first n(restriction) movements apply expansion
                    r=[r, candidates(k).expandACAS(obj.parrentGrid,...
                                                   lm,separations)];
                end
            else
                r= [];                                                      % return expanded movements
            end
        end
        
        
        %% Surface treshold calculation (LiDAR density problem)
        % calculates surface of passing cell for mean distance
        %   meanDistance - mean distance of hits
        %   DISCLAIMER - DO NOT USE THIS TYPE OF INTEGRATION unstable as
        %   hell
        function r=calculateSurface(obj,meanDistance)
            r=0;
            if obj.phiStart <0 && obj.phiEnd <=0
                r=meanDistance^2*(cos(abs(obj.phiEnd))-cos(abs(obj.phiStart)))*(obj.thetaEnd-obj.thetaStart);
            end
            if obj.phiStart >=0 && obj.phiEnd >0
                r=meanDistance^2*(cos(obj.phiStart)-cos(obj.phiEnd))*(obj.thetaEnd-obj.thetaStart);
            end
            if obj.phiStart < 0 && obj.phiEnd > 0
                r1=meanDistance^2*abs((cos(obj.phiStart)))*(obj.thetaEnd-obj.thetaStart);
                r2=meanDistance^2*(cos(obj.phiEnd))*(obj.thetaEnd-obj.thetaStart);
                sliceArea=(4*pi*meanDistance^2)/((2*pi)/(obj.thetaEnd-obj.thetaStart));
                r= sliceArea -r1-r2;
            end
            if r <0
                a=1;
            end
            r=r;
        end
        % Calculates surface of passing cell for mean distance
        %   meanDistance - mean distance of hits
        %   THIS ONE VERY STABLE
        function r=calculateSurface2(obj,meanDistance)
            r=0;
            if obj.phiStart <0 && obj.phiEnd <=0
                r=meanDistance^2*(sin(abs(obj.phiStart))-sin(abs(obj.phiEnd)))*(obj.thetaEnd-obj.thetaStart);
            end
            if obj.phiStart >=0 && obj.phiEnd >0
                r=meanDistance^2*(sin(obj.phiEnd)-sin(obj.phiStart))*(obj.thetaEnd-obj.thetaStart);
            end
            if obj.phiStart < 0 && obj.phiEnd > 0
                r1=meanDistance^2*abs(sin(obj.phiStart))*(obj.thetaEnd-obj.thetaStart);
                r2=meanDistance^2*(sin(obj.phiEnd))*(obj.thetaEnd-obj.thetaStart);
                r= r1+r2;
            end
            r=r;
        end
        
        % Calculates inner and outer surface of cell
        function r=calculateSurfaceInnerOuter(obj)
            obj.outerCellSurface=obj.calculateSurface2(obj.dEnd);
            if obj.dStart ~= 0                                              %there is no inner surface if cell is near the starting position .....
                obj.innerCellSurface=obj.calculateSurface2(obj.dStart);
            end
            r=[obj.innerCellSurface;obj.outerCellSurface];
        end
    end
    
end

