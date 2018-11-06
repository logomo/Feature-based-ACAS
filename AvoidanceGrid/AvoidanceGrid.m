classdef AvoidanceGrid<RullableObject
    %AVOIDANCEGRID Probabilistic implementation of infamous avoidance grid
    %   Contains reachibility and probability calculation and data fusion
    
    properties
        dStart          % distance start - keep it 0, untested for other values
        dEnd            % end distance - field of range, avoidance range, positive real number >> dStart
        thetaStart      % starting horizontal range - no need to be simetric
        thetaEnd        % ending horizontal range - no need to be simetric
        phiStart        % starting vertical range - no need to be simetric
        phiEnd          % ending vertical range - no need to be simetric
        countLayers     % count of layers - user defined count of layers
        stepLayer       % distance of step - calculated i in c_ijk
        countHorizontal % count of horizontal cells - j parameter in c_ijk
        stepHorizontal  % value of horizontal cell step - rad
        countVertical   % count of vertical cells - k parameter in c_ijk
        stepVertical    % value of vertical cell step - rad
        layers          % ordered array of layers
        adversarials    % list of detected adversarials
        linearModel     % linear model used to calculate Reach set (just reference) - Do not use in OA
        reachSet        % reach set for given vehicle model ... 3 new juicy methods to do it
        cellMap         % intersection structure for cell map
        debug=0;          % debug flag
        rasterColor='--k' % rasterColor
    end
    
    methods
        % Constructor function
        %   ds - distanceStart [m] - 0 please keep it 0
        %   de - distanceEnd [m]
        %   ts - horizontal angle start [rad]
        %   te - horizontal angle end [rad]
        %   ps - vertical angle start [rad]
        %   pe - vertical angle end [rad]
        %   lc - layer count [positive integer] - must match unit distance
        %        of movement automaton
        %   hc - horizontal cell count [positive integer]
        %   vc - vertical cell count [positive integer]
        function obj=AvoidanceGrid(ds,de,ts,te,ps,pe,lc,hc,vc)
            Cmnf.logc(obj,'Creating avoidance Grid')
            obj.dStart=ds;
            obj.dEnd=de;
            Cmnf.logc(obj,['--Range:',mat2str(obj.dEnd)]);
            obj.thetaStart=ts;
            obj.thetaEnd=te;
            Cmnf.logc(obj,['--Horizontal range:[',mat2str(rad2deg(obj.thetaStart)),'deg, ',mat2str(rad2deg(obj.thetaEnd)),' deg]']);
            obj.phiStart=ps;
            obj.phiEnd=pe;
            Cmnf.logc(obj,['--Vertical range:[',mat2str(rad2deg(obj.phiStart)),'deg, ',mat2str(rad2deg(obj.phiEnd)),' deg]']);
            obj.countVertical=vc;
            Cmnf.logc(obj,['--Vertical cell count:',mat2str(obj.countVertical)]);
            obj.countHorizontal=hc;
            Cmnf.logc(obj,['--Horizontal cell count:',mat2str(obj.countHorizontal)]);
            obj.countLayers=lc;
            Cmnf.logc(obj,['--Layers :',mat2str(obj.countLayers)]);            
            obj.stepLayer = (de-ds)/lc;
            Cmnf.logc(obj,['--Layer length :',mat2str(obj.stepLayer)]);            
            obj.stepHorizontal = (te-ts)/hc;
            Cmnf.logc(obj,['--Horizontal widenes :',mat2str(rad2deg(obj.stepHorizontal)),' deg']);            
            obj.stepVertical = (pe-ps)/vc;
            Cmnf.logc(obj,['--Vertical widenes :',mat2str(rad2deg(obj.stepVertical)),' deg']);
            Cmnf.logc(obj,'Creating layers');
            wlay(lc) = GridLayer();
            slay = linspace(ds,de,lc+1);
            for k=1:(length(slay)-1)
                wlay(k)=GridLayer(slay(k),slay(k+1),ts,te,ps,pe,hc,vc);
            end
            obj.layers=wlay;
            obj.adversarials = [];
            Cmnf.logc(obj,'Linking layers, cells');
            obj.linkIt;                                                     % Object must be crosslinked before usage
            %obj.calculateCellMap;                                          % Object must have cell map for intersection function calculaction
            Cmnf.logc(obj,'Calculating tolerated deviation');
            obj.calculateToleratedDistanceDeviation;                        % calculate layer distance deviations (it copies distance deviations to cells)
        end
        
        %Reset function
        function resetGrid(obj)
            obj.reachSet.resetParameters;
            obj.adversarials=[];
            for k=1:length(obj.layers)
                obj.layers(k).resetLayer;
            end
        end

        %Linker function
        function linkIt(obj)
            for k=1:obj.countLayers
                obj.layers(k).layerIndex = k;
                obj.layers(k).parrentGrid = obj;
                obj.layers(k).linkIt;
            end
        end
        %cellMapCalculation function [center,ijkMap]
        function calculateCellMap(obj)
            cellMap=[];                                                     
            for k= 1:length(obj.layers)
                for l=1:obj.layers(k).horizontalCellCount
                    for m = 1:obj.layers(k).verticalCellCount
                        cell = obj.layers(k).cells(l,m);
                        center = cell.center;
                        position = [k;l;m];
                        cellMap=[cellMap,[center;position]];
                    end
                end
            end
            obj.cellMap=cellMap;
        end
        function r=calculateToleratedDistanceDeviation(obj)
            r = zeros(1,obj.countLayers);
            for k = 1:obj.countLayers
                r(k)= obj.layers(k).calculateToleratedDistanceDeviation;
            end
        end
        %% Getter functions
        % Get cells defined by ranges
        %   lrange - layer range [ordered set of positive integers]
        %   hrange - horizontal range [ordered set of positive integers]
        %   vrange - vertical range [ordered set of positive integers]
        function r=getCells(obj,lrange,hrange,vrange)
            count = length(lrange)*length(hrange)*length(vrange);
            iter = 1;
            r(count) = GridCell();
            for i=lrange
                for j=hrange
                    for k=vrange
                     r(iter) = obj.layers(i).cells(j,k);
                     iter = iter + 1;
                    end
                end
            end
        end
        
        % Get cells defined by list, with advanced sanity check
        %   list - column list of cell coordinates
        function r=getCellsListedSanity(obj,list)
            [m,n] = size(list);
            if m ~=3
                r=[];
                return;
            end
            candidates = [];
            for l=1:n
                i = list(1,l);
                j = list(2,l);
                k = list(3,l);
                if i>= 1 && i<= obj.countLayers
                    wLayer =obj.layers(i);
                	if (j >= 1  && j <= wLayer.horizontalCellCount && k >= 1  && k <= wLayer.verticalCellCount)
                        candidates=[candidates,wLayer.cells(j,k)];
                    end
                end
            end
            r=candidates;
        end
        % Get cell defined by euclidian position in LCF (Local coordinate frame)
        %   position - [x,y,z] position in LCF
        function r=getCellEuclidian(obj,position)
            pl = Cmnf.euc2plan(position(1),position(2),position(3));
            r = obj.getCellPlanar(pl);
        end
        
        % Get cell by planar coordinates - LLCMF
        %   position - [distance, theta,varhhi] - Cmnf gives apro. transf.
        function r=getCellPlanar(obj,position)
            distance =position(1);
            theta = position(2);
            phi = position(3);
            if distance >= obj.dEnd || distance < obj.dStart                 %check distance in grid
                r=0;
            else
                if (floor(distance/obj.stepLayer) ==distance)               %calculate layer index
                    iLayer = distance;
                else
                    iLayer = floor(distance/obj.stepLayer)+1;
                end
                if iLayer >= 1
                    iCell = obj.layers(iLayer).getCellIndexes(theta,phi);       %calculate cell index based on angles
                    if iCell == 0
                        r=0;
                    else
                        r=obj.layers(iLayer).cells(iCell(1),iCell(2));          %give the cell
                    end
                else
                    r=0;
                end
                
            end
        end
        
        % Holistic approach to get line vs Grid intersection, does not 
        % work very well ig cell count >> 800
        %   point - point of mass inside of grid 
        %   velocity - velocity 
        function r=getLineIntersection(obj,point,velocity)
            if length(obj.cellMap) == 0
                obj.calculateCellMap;
            end
            cellMap=obj.cellMap;
            normVelocity = velocity/norm(velocity,2);
            centeredCellMap=cellMap(1:3,:) - point * ones(1,length(cellMap));

            normedCenteredCellMap = centeredCellMap;
            for k=1:length(normedCenteredCellMap)
                vn=norm(normedCenteredCellMap(:,k),2);
                normedCenteredCellMap(:,k) = normedCenteredCellMap(:,k)/vn;
            end
            diffNormedCellMap = normedCenteredCellMap... 
                                - normVelocity*ones(1,length(normedCenteredCellMap));
            normVector = [1,1,1]*(diffNormedCellMap.^2);
            [v,in] = sort(normVector);

            % for one
            ccs = [];
            
            cCell = 0;
            while ~isempty(in)                
                cCell = 0;
                id=in(1);
                in(1)=[];
                wpt=centeredCellMap(:,id);
                time= norm(wpt,2)/norm(velocity,2);
                projPo = point + velocity*time;
                cCell=obj.getCellEuclidian(projPo);
                if cCell ~=0
                    tVal = cCell.center - cellMap(1:3,id);
                    if tVal == 0
                        ccs= [ccs, IntersectionGridCell(cCell,projPo,time)];
                    end
                end
            end
            r=ccs;
        end
        
        % Numeric approach for line intersection work well with greater
        % grids but its not stable as holistic approach
        %   adversaryPoint - Adversary point at time 0
        %   adversaryVelocity - Adversary velocity vector
        function r=getLineIntersectionNumeric(obj,adversaryPoint,...
                                                  adversaryVelocity)
            maximumGridRange= 2*(obj.dEnd -obj.dStart);
            maximumTime = maximumGridRange/norm(adversaryVelocity,2);
            stepTime = obj.stepLayer/norm(adversaryVelocity,2);
            time=0:stepTime:maximumTime;

            % initial search
            cells = [];
            sucTime = [];
            register = containers.Map;
            if obj.debug == 1
                points = [];
            end
            for t=time
                testPoint = adversaryPoint+adversaryVelocity*t;
                if obj.debug ==1
                    points=[points,testPoint];
                end
                cell=obj.getCellEuclidian(testPoint);
                if cell ~=0
                   key =  mat2str(cell.ijkIndex);
                   if ~register.isKey(key)
                        register(key)=cell;
                   end
                   cells = [cells,IntersectionGridCell(cell,testPoint,t)];
                   sucTime=[sucTime,t];
                end
            end

            % spanning range offset search
            for k=sucTime
                flag = 1;
                siz = 3;
                while flag 
                   flag=0;
                   for t=linspace(0,stepTime,siz)
                        testPoint = adversaryPoint+adversaryVelocity*(t+k);
                        cell=obj.getCellEuclidian(testPoint);
                        if cell ~=0
                            key =  mat2str(cell.ijkIndex);
                            if ~register.isKey(key)
                                register(key)=cell;
                                cells = [cells,IntersectionGridCell(cell,testPoint,t+k)];
                                sucTime=[sucTime,t];
                                flag=1;
                            end
                        end
                   end
                   siz=siz*2;
                end
                % and one more time but in reverse
                flag = 1;
                siz = 3;
                while flag 
                   flag=0;
                   for t=linspace(0,-stepTime,siz)
                        testPoint = adversaryPoint+adversaryVelocity*(t+k);
                        cell=obj.getCellEuclidian(testPoint);
                        if cell ~=0
                            key =  mat2str(cell.ijkIndex);
                            if ~register.isKey(key)
                                register(key)=cell;
                                cells = [cells,IntersectionGridCell(cell,testPoint,t+k)];
                                sucTime=[sucTime,t];
                                flag=1;
                            end
                        end
                   end
                   siz=siz*2;
                end
            end
            r=cells;
        end
        %% Adversarial behaviour
        % Adds adversarial objects created in AdversaryVehicle class
        %   adverarial - valid adversarial
        function r=putAdversarial(obj,adversarial)
            obj.adversarials= [obj.adversarials,adversarial];               % register adversarial for this runtime
            points = [];                                                    % interection points
            hits = 0;                                                       % count of one elipsoindal hits to grid range
            totalHits = 0;                                                  % count of total hits for all ranging attmpts   
            intersectionFlag = 0;                                           % enter/leave check flag ....
            for k=0:50 %add better sanity check                             % better numeric intersection algorithm TODO
                s=adversarial.calculateElipse(k,obj.stepLayer);
                [m,n] = size(s);
                for l=1:n
                    worker = s(:,l);
                    cc=obj.getCellEuclidian(worker(1:3));
                    if cc~=0
                        points = [points,worker(1:3),];
                        hits = hits + 1;
                        cc.pAdversary = [cc.pAdversary,worker];
                    end
                end
                if hits > 0  %raise flag
                    intersectionFlag = 1;
                end
                if intersectionFlag == 1 && hits == 0
                    break
                end
                totalHits = totalHits + hits;
                hits = 0;
            end
            r=points;
        end
        %
        function r=putTimedAdversarial(obj,adversarial)
            obj.adversarials= [obj.adversarials,adversarial];
            points=[];
            adversarial.findIntersection(obj);
            if ~adversarial.flagOnlySpread
                if (adversarial.flagTimeIntersection)
                    for ic=adversarial.timeMovementCells;
                        points = [points,ic.position];
                        av=Cmnf.getAdversaryVector(ic.cell.center,ic.probability,ic.time);
                        ic.cell.pAdversary=[ic.cell.pAdversary,av];
                    end
                else
                    for ic=adversarial.intersectionCells;
                        points = [points,ic.position];
                        av=Cmnf.getAdversaryVector(ic.cell.center,1,ic.time);
                        ic.cell.pAdversary=[ic.cell.pAdversary,av];
                    end
                end

                if adversarial.flagFutureMovements
                    for ic=adversarial.futureMovementCells;
                        points = [points,ic.position];
                        av=Cmnf.getAdversaryVector(ic.cell.center,ic.probability,ic.time);
                        ic.cell.pAdversary=[ic.cell.pAdversary,av];
                    end
                end

                if adversarial.flagBallIntersection
                    for brc = adversarial.ballRadiusCells
                        ic= brc.center;
                        points = [points,ic.position];
                        av=Cmnf.getAdversaryVector(ic.cell.center,ic.probability,ic.time);
                        ic.cell.pAdversary=[ic.cell.pAdversary,av];
                        for ic=brc.coating
                            points = [points,ic.position];
                            av=Cmnf.getAdversaryVector(ic.cell.center,ic.probability,ic.time);
                            ic.cell.pAdversary=[ic.cell.pAdversary,av];
                        end
                    end
                end
            end
            
            if adversarial.flagSpread
                adversaryLinearVelocity=norm(adversarial.velocity,2);                   
                adversaryTimeError=adversarial.distanceError;                               
                predictionError = adversaryLinearVelocity*adversaryTimeError;
                cells=adversarial.spreadCells;      
                for wCell=cells
                    x=[];y=[];z=[];p=[];t=[];
                    if (adversarial.flagTimeIntersection)
                        cellArrivalTime = wCell.cell.minimalEntryTime;
                        cellLeaveTime = wCell.cell.maximalLeaveTime;
                        cellIntersection = wCell.cell.toleratedDeviation/adversaryLinearVelocity;
                        minBoundary = cellArrivalTime - predictionError - cellIntersection;         % minimal cell entry boundary
                        maxBoundary = cellLeaveTime + predictionError + cellIntersection;           % maximal cell leave boundary
                        for i=1:length(wCell.time)
                            arrivalTime = wCell.time(i);
                            if arrivalTime >= minBoundary && maxBoundary>=arrivalTime
                                x=[x,wCell.position(1,i)];
                                y=[y,wCell.position(2,i)];
                                z=[z,wCell.position(3,i)];
                                p=[p,wCell.probability(i)];
                                t=[t,wCell.time(i)];
                            end
                        end
                    else
                        x=wCell.position(1,:);
                        y=wCell.position(2,:);
                        z=wCell.position(3,:);
                        p=wCell.probability;
                        t=wCell.time;
                    end
                    if ~isempty(p)
                        pos=[mean(x);mean(y);mean(z)];
                        points=[points,pos];
                        p=mean(p);
                        t=mean(t);
                        av=Cmnf.getAdversaryVector(pos,p,t);
                        wCell.cell.pAdversary=[wCell.cell.pAdversary,av];
                    end
                end
            end
            r= points;
        end
        
        %% Static obstacles
        function r=putObstacle(obj,cell,obstacle,probability)
            %if its detected obstacle
            if obstacle.type==ObstacleType.Detected
                %get frontal cells
                roll=obj.getCells(cell.layerIndex:obj.countLayers,cell.horizontalIndex,cell.verticalIndex);
                rollLength=length(roll);
                firstCell=roll(1);
                %Put obstacle into first cell
                %ID+OBSTACLE PROBABILITY = [obstacle.id; probability]
                firstCell.pDetectedObstacle=[firstCell.pDetectedObstacle,[obstacle.id; probability]];
                %Put hindered visibility into cell
                %ID+OBSTACLE PROBABILITY = [obstacle.id; probability]
                if rollLength >1
                    for k=2:rollLength
                        workCell=roll(k);
                        workCell.pHinderedVisibility = [workCell.pHinderedVisibility, [obstacle.id; probability]];
                    end
                end
            end
            if obstacle.type==ObstacleType.Map
               cell.pMapObstacle =[cell.pMapObstacle, [obstacle.id; probability]];
            end
            r=0;
        end
        %% Reachable set functions
        % Recreate associations NODE<-> cell after pruning or after reach
        % set change ...
        %   root - new reach set base implemented as PredictorNode class
        function r=recreateAssociations(obj,root)
            for i=1:length(obj.layers)                                      %Just clean old references 
                wLayer=obj.layers(i);
                for k=1:wLayer.horizontalCellCount*wLayer.verticalCellCount
                    wLayer.cells(k).trajectories=[]; 
                end 
            end
            r=root.registerSelf;                                            %Tree based function for initial registration
            root.calculateCost;                                             %Tree based calculation
            for i=1:length(obj.layers)                                      %Just clean old references 
                wLayer=obj.layers(i);
                for k=1:wLayer.horizontalCellCount*wLayer.verticalCellCount
                    wLayer.cells(k).calculateArrivalTime; 
                end
                obj.layers(i).calculateArrivalTime;
            end
        end
        % Sanity check function, returns count of assiociated nodes in
        % Avoidance grid, it should be same as reachibility Set - 1 (root)
        function r=countAssociations(obj)
            r=0;
            for i=1:length(obj.layers)
                wLayer=obj.layers(i);
                for k=1:wLayer.horizontalCellCount
                    for l = 1:wLayer.verticalCellCount
                       r=r+length(wLayer.cells(k,l).trajectories);
                    end
                end 
            end
        end
        
        %
        %testPredictionInGridSmartSplit03Best 
        function precalculateHarmonicReachSet(obj,lm)
            obj.linearModel = lm;
            %root build up
            root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero orientation
            root.root=root;
            root.normalExpand(obj,lm);
            for k=1:obj.countLayers
                candidates = obj.layers(k).expandRapid;
                for l=1:length(candidates)
                    candidates(l).normalExpand(obj,lm);
                end
            end
            root.reachFinalLayerTest(obj.countLayers);
            root.pruneBasedOnFinalLayer;
            root.pruneLastSpread(obj.countLayers);
            obj.recreateAssociations(root);
            obj.reachSet=root;
        end
        
        % testPredictorInGridLayerBased02 farcount = 8, nearcount =1
        function precalculateCellSpreadReachSet(obj,lm,farCount,nearCount)
            obj.linearModel=lm;
            %root build up
            root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero movement
            root.root=root;
            root.normalExpand(obj,lm);
            for k=1:obj.countLayers
                candidates = obj.layers(k).expand(farCount,nearCount);
                for l=1:length(candidates)
                    candidates(l).normalExpand(obj,lm);
                end
            end
            root.reachFinalLayerTest(obj.countLayers);
            root.pruneBasedOnFinalLayer;
            root.pruneLastSpread(obj.countLayers);
            obj.recreateAssociations(root);
            obj.reachSet=root;
        end
        
        %test 01
        function precalculateLimitedPassingReachSet(obj,lm,passRatio)
            obj.linearModel=lm;
            trajectoryRegister = containers.Map;
            %root build up
            root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero movement
            root.root=root;
            %build full map
            stack = [root];
            while ~length(stack)==0
                worker = stack(1);
                stack(1)=[];
                if worker.depth > obj.countLayers
                    worker.expanded = 1;
                    continue;
                end
                stack=[stack,worker.expand(obj,lm,trajectoryRegister,passRatio)];
            end
            root.reachFinalLayerTest(obj.countLayers);
            root.pruneBasedOnFinalLayer;
            root.pruneLastSpread(obj.countLayers);
            obj.recreateAssociations(root);
            obj.reachSet=root;
        end
        
        % tests
        %   testPredictorGridAcasReachSet.m
        %   testPredictorGridAcasReachSetFunction.m
        function precalculateACASReachSet(obj,lm,separations)
            obj.linearModel=lm;
            
            %root build up
            root=PredictorNode([0;0;0;0;0;0;0],0,0); %zero state, zero movement
            root.root=root;

            %Expand that root, you know you want it!
            root.expandACAS(obj,lm,separations);
            
            % Wavefront to the grid
            for k=1:obj.countLayers
                obj.layers(k).expandACAS(lm,separations);
            end

            % Test only final layer survival
            root.reachFinalLayerTest(obj.countLayers);

            % Obliterate
            root.pruneBasedOnFinalLayer;

            % Obliterate last sread
            root.pruneLastSpread(obj.countLayers);

            % Recreate associations
            obj.recreateAssociations(root);

            % Recaluclate trajectory cost
            root.calculateCost;
            obj.reachSet=root;
        end
        
        %% Global probability calculation
        function r=recalculate(obj)
            r=0;
            for i=1:obj.countLayers
                wLayer = obj.layers(i);
                for j =1:wLayer.horizontalCellCount
                    for k=1:wLayer.verticalCellCount
                        wCell = wLayer.cells(j,k);
                        wCell.recalculateProbabilities;
                        r=r+1;
                    end
                end
            end
            
            obj.reachSet.calculateProbability;
            obj.reclalculateReachibility;
        end
        
        function reclalculateReachibility(obj)
            for i=1:length(obj.layers)
                wLayer=obj.layers(i);
                for k=1:wLayer.horizontalCellCount*wLayer.verticalCellCount
                    wLayer.cells(k).recalculateReachibility;
                end 
            end
        end
        %% Plot functions
        function plotHorizontalSlice(obj,verticalIndex,statId)
            hc=obj.countHorizontal;
            obj.plotBaseSlice(1:obj.countLayers,1:hc,obj.countVertical-verticalIndex+1,0,1,statId)
            xlabel('x [m]')
            ylabel('y [m]')
        end
        function plotBaseSlice(obj,lrange,hrange,vrange,isVertical,isColored,statId)
            cells = obj.getCells(lrange,hrange,vrange);
            hold on
            for k = 1:length(cells)
                gObj=cells(k).getPlotData;
                worker = 0;
                if (isVertical)
                    worker = gObj.verticalCell;
                else
                    worker = gObj.horizontalCell;
                end
                if isColored == 1
                    colOpt = gObj.getcolorVectorRGB(statId);
                else
                    colOpt = gObj.getColorVectorGS(statId);
                end
                fill(worker(1,:),worker(2,:),colOpt)
            end
            hold off
        end
        
        function r=plotRasterRange(obj,offsets)
            r=[];
            hold on
            x=offsets(1);
            y=offsets(2);
            z=offsets(3);
            alpha=offsets(4);
            beta=offsets(5);
            gamma=offsets(6);
            line = [0,obj.dEnd;
                    0,0;
                    0,0;];
            for k = linspace(obj.thetaStart,obj.thetaEnd,3)
                for l = linspace(obj.phiStart,obj.phiEnd,3)
                    rline = Cmnf.rot3D(0,l,k,line);
                    rrline = Cmnf.rot3D(alpha,beta,gamma,rline);
                    %r=[r,rrline];
                    handle = plot3(rrline(1,:)+x,rrline(2,:)+y,rrline(3,:)+z,'--k');
                    r = [r,handle];
                end
            end
            m =obj.dEnd;
            point = [m;0;0];
            for k = linspace(obj.thetaStart,obj.thetaEnd,3)
                hline = zeros(3,50);
                hstep = linspace(obj.phiStart,obj.phiEnd,50);
                for l = 1:50
                    hline(:,l) =  Cmnf.rot3D(0,hstep(l),k,point);
                end
                rhline = Cmnf.rot3D(alpha,beta,gamma,hline);
                %r=[r,rhline];
                handle = plot3(rhline(1,:)+x,rhline(2,:)+y,rhline(3,:)+z,'--k');
                r = [r,handle];
            end
            for k = linspace(obj.phiStart,obj.phiEnd,3)
                vline = zeros(3,50);
                vstep = linspace(obj.thetaStart,obj.thetaEnd,50);
                for l = 1:50
                    vline(:,l) = Cmnf.rot3D(0,k,vstep(l),point);
                end
                rvline = Cmnf.rot3D(alpha,beta,gamma,vline);
                %r=[r,rvline];
                handle = plot3(rvline(1,:)+x,rvline(2,:)+y,rvline(3,:)+z,'--k');
                r = [r,handle];
            end
            
            hold off
        end
        
        function r=plotRaster(obj,fig,sx,sy,p)
            if nargin == 0
                figure(1);
            end
            if nargin >= 2
                figure(fig);
            end
            if nargin >= 5
                figure(fig);
                subplot(sx,sy,p);
            end
            hold on
            line = [0,obj.dEnd;
                    0,0;
                    0,0;];
            for k = linspace(obj.thetaStart,obj.thetaEnd,obj.countHorizontal+1)
                for l = linspace(obj.phiStart,obj.phiEnd,obj.countVertical+1)
                    rline = Cmnf.rot3Dvec(line,[0,l,k]);
                    plot3(rline(1,:),rline(2,:),rline(3,:),obj.rasterColor);
                end
            end

            for m =linspace(obj.stepLayer,obj.dEnd,obj.dEnd/obj.stepLayer);
                point = [m;0;0];
                for k = linspace(obj.thetaStart,obj.thetaEnd,obj.countHorizontal+1)
                    hline = zeros(3,50);
                    hstep = linspace(obj.phiStart,obj.phiEnd,50);
                    for l = 1:50
                        hline(:,l) = Cmnf.rot3Dvec(point,[0,hstep(l),k]);
                    end
                    plot3(hline(1,:),hline(2,:),hline(3,:),obj.rasterColor);
                end
                for k = linspace(obj.phiStart,obj.phiEnd,obj.countVertical+1)
                    vline = zeros(3,50);
                    vstep = linspace(obj.thetaStart,obj.thetaEnd,50);
                    for l = 1:50
                        vline(:,l) = Cmnf.rot3Dvec(point,[0,k,vstep(l)]);
                    end
                    plot3(vline(1,:),vline(2,:),vline(3,:),obj.rasterColor);
                end
            end  
            hold off
            r=0;
        end
        function plotReachSet(obj)
            root=obj.reachSet;
            nodesCount=root.countNodes(0);
            routeCount=root.countNodes(1);
            leafs=root.collectLeafs;
            obj.plotRasterRange([0,0,0,0,0,0])
            for k=1:length(leafs)
                hold on
                st=leafs(k).state;
                plot3(st(1,:),st(2,:),st(3,:))
                hold off
            end
            grid on
            xlabel('x[m]')
            ylabel('y[m]')
            zlabel('z[m]')
            title(['Nodes: ',mat2str(nodesCount),' Routes: ', mat2str(routeCount)])
        end
        
        function plotReachSetColored(obj, mode)
            if nargin == 1
                mode = StatisticType.Reachability;
            end
            traj=obj.reachSet.collectLeafs;
            grid on
            obj.plotRasterRange([0,0,0,0,0,0])
            for k=1:length(traj)
                hold on
                traj(k).plotTrajectoryStat(mode);
                hold off
            end
            title('Reach set colored (red-green)')
            xlabel('x [m]')
            ylabel('y [m]')
            zlabel('z [m]')
        end
        
        %% constraints application
        function r=applyHardConstraint(obj,mc,cc)
            obj.reachSet.applyHardConstraint(mc,cc);
            r=obj.reachSet.applyOperatibleSpace();
            obj.recalculate();
        end
        
        %% Rule engine create context (DO NOT CALL DIRECTLY)
        function r=createContextRuleEngine(obj)
            createContextRuleEngine@RullableObject(obj);
            obj.reContext('grid')=obj;
            r=obj.reContext;
        end
        
        %% Rule engine injection method
        function r=injectRuleEngine(obj,ruleEngine)
            masterFlag=injectRuleEngine@RullableObject(obj,ruleEngine);
            % TODO injection body
            r=masterFlag;
        end
    end
    
end

