classdef MissionControl<RullableObject
    %MISSIONCONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Mission data
        waypoints
        goal=[0;0;0];
        waypointID=0;
        maxWaypointID;
        waypointReachedFlag=0;
        waypointUnreachableFlag=0;
        finalWaypointReachedFlag=0;
        collisionFlag=0;
        ruleFlag=false;
        
        %Vehicle data
        vehicle
        movementBuffer
        vehicleName
        vehicleId=0;
        vehicleActualVelocity=1;  %set for 1 m/s for sake of simplicity TODO:mechanism of velocity change
        vehicleSafetyMargin=0; % set vehicleSafetyMargin
        vehicleCategory=VehicleCategory.UAVAutonomous;
        vehicleManCat =ManeuverabilityCategory.FullPropulsionGliding;
        
        %Obstacle data
        intruders=[];
        intruderId=1;
        staticObstacleID=1;
        staticObstacles=[];
        constraintID=1;
        constraints=[];
        activeSHCostraints = [];
        staticObstacleFlag=0;
        intruderFlag=0;
        obstacleFlag=0;
        hardConstraintFlag=false;
        softConstraintFlag=false;
        forcerReplaningFlag=0;
        
        %Navigation grid
        navigationGrid
        navigationType
        navigationParams
        
        %Avoidance grid
        avoidanceGrid
        avoidanceType
        avoidanceParams
        
        %Adversarial intersection model
        flagTimeIntersection = 0;    %Enforce time
        flagFutureMovements = 0;     %Enforce future movements
        flagBallIntersection = 0;    %Enforce ball intersection
        flagSpread = 0;              %Use spread
        flagOnlySpread = 0;          %DO not Use line intersection
        
        %Plot parameters
        trajectoryColor='b';
        plannedColor='r';
        lastTrajectoryPlotHandles=[];
        actualTrajectoryPlotHandles=[];
        
        %SimulationParameters
        simulationTime=0;
        missionLog=[];
    end
    
    methods
        %% Constructor
        function obj=MissionControl(waypoints,position,orientation,gridDistance,gridHorizontalRange,gridVerticalRange,gridHorizontalCount,gridVerticalCount,navigationType,navigationParams,avoidanceType,avoidanceParams)
            % Default values for reach set calculations and parameters
            if (nargin == 8)
                navigationType=ReachSetCalculation.Harmonic;
                navigationParams=containers.Map;
                avoidanceType=ReachSetCalculation.Harmonic;
                avoidanceParams=containers.Map;
            end
            %Start mission control creation
            Cmnf.logc(obj,'Creating mission control object');

            % Add waypoints
            obj.waypoints = waypoints;
            for k=1:length(obj.waypoints)
                str = ['Adding waypoint ',mat2str(k),'. with value ',mat2str(obj.waypoints(k).position)];
                Cmnf.logc(obj,str);
            end
            obj.maxWaypointID = length(obj.waypoints);
            %Create vehicle
            omega_alpha_0=orientation(1);
            omega_beta_0=orientation(2);
            omega_gamma_0=orientation(3);
            x_0=position(1);
            y_0=position(2);
            z_0=position(3);
            velocity=Cmnf.vehicleSpeed;

            Cmnf.logc(obj,'Creating state object')
            s = State(omega_alpha_0,omega_beta_0,omega_gamma_0,x_0,y_0,z_0, velocity);

            Cmnf.logc(obj,'Creating vehicle object')
            v = Vehicle(s);
            obj.vehicle=v;

            %Create avoidance grid
            ds=0;
            de=gridDistance;
            ts=-gridHorizontalRange;
            te=gridHorizontalRange;
            ps=-gridVerticalRange;
            pe=gridVerticalRange;
            lc=gridDistance/(Cmnf.vehicleSpeed*Cmnf.simStep);
            hc=gridHorizontalCount;
            vc=gridVerticalCount;
            Cmnf.logc(obj,'Creating navigation grid')
            navigationGrid=AvoidanceGrid(ds,de,ts,te,ps,pe,lc,hc,vc);
            obj.navigationGrid=navigationGrid;
            ReachSetCalculation.createReachSet(navigationGrid,navigationType,navigationParams);
            obj.navigationType=navigationType;
            obj.navigationParams=navigationParams;
            Cmnf.logc(obj,'Set initial values');
            navigationGrid.resetGrid;
            Cmnf.logc(obj,'Creating avoidance grid')
            avoidanceGrid=AvoidanceGrid(ds,de,ts,te,ps,pe,lc,hc,vc);
            obj.avoidanceGrid=avoidanceGrid;
            ReachSetCalculation.createReachSet(avoidanceGrid,avoidanceType,avoidanceParams);
            obj.avoidanceType=avoidanceType;
            obj.avoidanceParams=avoidanceParams;
            Cmnf.logc(obj,'Set initial values');
            avoidanceGrid.resetGrid;
        end
        
        %% Simulation function
        function runOnce(obj,showStatFlag)
            %Show status performance of navigation/avoidance grid for this
            %instance ?
            if(nargin==1)
                showStatFlag=false;
            end
            
            %% Waypoint check logic
            [obj.finalWaypointReachedFlag,obj.waypointReachedFlag]=obj.checkAndSetObjective;
            obj.obstacleFlag=0;
            obstacleHits=0;
            intruderHits=0;

            if obj.finalWaypointReachedFlag 
                return
            end

            %% Path planning logic
            if obj.waypointReachedFlag
                obj.movementBuffer=[];
            end
            obj.checkForcedPathReplaning;
            
            %% Intruder logic
            detectedIntruders=obj.getDetectedIntruders;
            intruderHits= obj.intersectIntrudersWithGrid(obj.avoidanceGrid,detectedIntruders);
            if intruderHits > 0
                obj.intruderFlag=1;
            else
                obj.intruderFlag=0;
            end


            %% static obstacles
            [intersections,collisions]=getIntersectionCollisionCandidates(obj);
            if ~isempty(intersections)
                obstacleHits=obj.intersectObstaclesWithGrid(obj.avoidanceGrid,intersections);
                if obstacleHits>0
                    obj.staticObstacleFlag=1;
                else
                    obj.staticObstacleFlag=0;
                end
            else
                obj.staticObstacleFlag=0;
            end
            %% Constraints application
            movingConstraints=obj.moveConstraints();
            [obj.hardConstraintFlag,obj.softConstraintFlag,hardConstraints,softConstraints] = obj.getConstraintsCandidates();
            
            %% Main  danger evaluation
            RuleEngine.invoke(obj,RuleJointPoint.MissionControlRunOnce);
            if obj.staticObstacleFlag || obj.intruderFlag || obj.ruleFlag
                % if there is any real danger then invoke recalculaitons
                obj.obstacleFlag = 1;
                RuleEngine.invoke(obj,RuleJointPoint.MissionControlCollisionCaseSolution);
                obj.avoidanceGrid.recalculate;
                if showStatFlag
                    obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Reachability,2)
                    obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Obstacle,3)
                    %obj.plotGridSlice(obj.avoidanceGrid,StatisticType.Visibility,4)
                end
            else
                obj.obstacleFlag = 0;
            end
            
            %% Invoke rule engine after run
            RuleEngine.invoke(obj,RuleJointPoint.MissionControlAfterRun);

            if isempty(obj.movementBuffer) || obj.forcerReplaningFlag || ~obj.staticObstacleFlag || obj.intruderFlag || obj.softConstraintFlag || obj.hardConstraintFlag
                if obj.obstacleFlag
                    % ## Avoidance mode is INTUITIVE ##                    
                    % Get trajectory if there is no constraints
                    [noConstraintsTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.avoidanceGrid);
                    
                    % Get trajectory if there are hard constraints
                    if obj.hardConstraintFlag
                        % apply Hard constraints
                        [hardConstraintTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.avoidanceGrid);
                    else
                        hardConstraintTrajectory = 0;
                    end
                    
                    % Get trajectory for soft constraints
                    if obj.softConstraintFlag
                        [softConstraintTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.avoidanceGrid);
                    else
                        softConstraintTrajectory = 0;
                    end
                    
                else
                    % Avoidance mode is COOPERATIVE
                    [noConstraintsTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.navigationGrid);
                    
                    % Get trajectory if there are hard constraints
                    if obj.hardConstraintFlag
                        % apply Hard constraints
                        [constrainedCount]=obj.applyHardConstraints(obj.navigationGrid);
                        [hardConstraintTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.navigationGrid);
                        if constrainedCount > 0
                            r=1
                        end
                    else
                        hardConstraintTrajectory = 0;
                    end
                    
                    % Get trajectory for soft constraints
                    if obj.softConstraintFlag
                        [softConstraintTrajectory,obj.waypointUnreachableFlag,obj.collisionFlag]=obj.findBestPath(obj.navigationGrid);
                    else
                        softConstraintTrajectory = 0;
                    end
                end
                
                % Collecting best possible trajecotry
                %  soft constraints - if exists - else try collect hard
                %  constraints
                %  hard constraints - if exist else try to collect no
                %  constraints
                %  no constraints trajectorty exists by default
                if obj.softConstraintFlag && softConstraintTrajectory ~= 0
                    bestTrajectory = softConstraintTrajectory;
                else
                    if obj.hardConstraintFlag && hardConstraintTrajectory~=0
                        bestTrajectory = hardConstraintTrajectory;
                    else
                        bestTrajectory = noConstraintsTrajectory;
                    end
                end
                
                % Grid cleanup
                if obj.softConstraintFlag || obj.hardConstraintFlag || obj.obstacleFlag
                    obj.navigationGrid.resetGrid
                    if obj.obstacleFlag
                        obj.avoidanceGrid.resetGrid
                    end
                end

                obj.movementBuffer=bestTrajectory.collectMovements;
                if obj.waypointUnreachableFlag
                    a=1
                end
                if obj.collisionFlag
                    b=1
                end
            end
            %% Movement execution
            %Log begin
                log=FlightLog;
                log.simulationTime=obj.simulationTime;
                log.vehiclePosOrBefore=obj.vehicle.getActualPositionOrientation;
                log.executedMovement=obj.movementBuffer(1);
                log.movementBuffer=obj.movementBuffer;
                log.waypointID=obj.waypointID;
                log.waypoint=obj.goal;
                log.intersections=intersections;
                log.obstacleHits=obstacleHits;
                log.staticObstacleFlag=obj.staticObstacleFlag;
                log.detectedIntruders=detectedIntruders;
                log.intruderHits=intruderHits;
                log.intruderFlag=obj.intruderFlag;
                log.waypointReachedFlag=obj.waypointReachedFlag;
                log.waypointUnreachableFlag=obj.waypointUnreachableFlag;
                log.finalWaypointReachedFlag=obj.finalWaypointReachedFlag;
                log.collisionFlag=obj.collisionFlag;
                log.obstacleFlag=obj.obstacleFlag;
                log.forcerReplaningFlag=obj.forcerReplaningFlag;
            % log interupt
            % fly movement
            obj.vehicle.fly(obj.movementBuffer(1));
            % log continue
                log.vehiclePosOrAfter=obj.vehicle.getActualPositionOrientation;
                obj.missionLog=[obj.missionLog,log];
                obj.simulationTime=obj.simulationTime+Cmnf.simStep;
                log.showMissionStatus;
            % log end
            if length(obj.movementBuffer)~=1
                obj.movementBuffer=obj.movementBuffer(2:length(obj.movementBuffer));
            else
                obj.movementBuffer=[];
            end
        end
        
        function runOnceWithPlot(obj,f,showStats)
            if nargin == 1
                f=1;
                showStats=false;
            end
            if nargin == 2
                showStats=false;
            end
            obj.runOnce(showStats);
            figure(f)
            obj.plotMissionTrajectory;
            obj.plotDetectedIntruders;
            if ~Cmnf.enableTracePlot
                obj.plotRemoveLastTrajectoryFromMissionFigure;
            end
        end
        
        %% waypoint reach methods
        function r=testWaypointReach(obj)
            vehicle=obj.vehicle;
            posOr=vehicle.getActualPositionOrientation();
            pos=posOr(1:3);
            or=posOr(4:6);
            realDistance=norm(obj.goal-pos,2);
            testDistance=Cmnf.vehicleSpeed*Cmnf.simStep*2;
            r=realDistance<=testDistance;
            Cmnf.logc(obj,['Testing waypoint ',mat2str(obj.goal),', vehicle position ',...
                            mat2str(pos),', check distance ', mat2str(testDistance),...
                            ', real distance ',mat2str(realDistance),...
                            ', result ',mat2str(r)]);
        end
        
        function [finalFlag,reachTest]=checkAndSetObjective(obj)
            finalFlag=false;
            if obj.waypointID ==0
                Cmnf.logc(obj,'Loading first waypoint');
                obj.waypointID=1;
                obj.goal=obj.waypoints(1).position;
            end

            while(1)
                reachTest=obj.testWaypointReach;
                if ~reachTest
                    break
                end
                if reachTest 
                    if obj.waypointID==length(obj.waypoints)
                        finalFlag=true;
                        Cmnf.logc(obj,'Final waypoint reached vehicle finished the mission')
                        break
                    end
                    obj.waypointID=obj.waypointID+1;
                    obj.goal=obj.waypoints(obj.waypointID).position;
                    Cmnf.logc(obj,['Waypoint reached, setting new goal ID:',...
                        mat2str(obj.waypointID),', Point:',mat2str(obj.goal)]);
                    break;
                end
            end
        end
        
        function r=checkForcedPathReplaning(obj)
            avoidanceGridEnd=obj.avoidanceGrid.dEnd;
            vehiclePosition = obj.vehicle.getActualPositionOrientation;
            vehiclePosition=vehiclePosition(1:3);
            goal=obj.goal;
            goalDistance=Cmnf.calculatePointDistance(vehiclePosition,goal);
            obj.forcerReplaningFlag = goalDistance <=avoidanceGridEnd;
            r=obj.forcerReplaningFlag;
        end
        %% Goal finding methods
        function [bestTrajectory,waypointUnreachableFlag,collisionFlag]=findBestPath(obj,ag)
            bestTrajectory=0;
            localGoal=obj.getLocalCoordiantes(obj.goal);
            intersectionCell=ag.getCellEuclidian(localGoal);
            waypointUnreachableFlag=false;
            collisionFlag=false;
            % Target is inside grid 
            if intersectionCell~=0
                Cmnf.logc(obj,['Mission Goal is inside Avoidance grid, cell ',mat2str(intersectionCell.ijkIndex)]);
                if intersectionCell.pReachability >= Cmnf.reachibilityTreshold
                    Cmnf.logc(obj,['Intersection cell ',mat2str(intersectionCell.ijkIndex),' is reachable']);
                    bestTrajectory=obj.findBestTrajectoryInCell(intersectionCell);
                    return
                else
                    Cmnf.logc(obj,['Intersection cell ',mat2str(intersectionCell.ijkIndex),' is NOT reachable']);
                    bestCell=obj.findBestCellOnLayer(ag,intersectionCell.layerIndex);
                    if bestCell~=0
                        Cmnf.logc(obj,['Nearest reachable cell ',mat2str(bestCell.ijkIndex),' found']);
                        bestTrajectory=obj.findBestTrajectoryInCell(bestCell);
                        return;
                    else
                        Cmnf.logc(obj,['No reachable cells at layer ',mat2str(intersectionCell.layerIndex),' waypoint is ',mat2str(obj.goal),' unreachable']);
                        waypointUnreachableFlag=true;
                        return
                    end
                end
            end

            % Waypoint is outside of Grid
            % 10,9...1
            Cmnf.logc(obj,['Target waypoint is outside of Avoidance grid trying to find closest advance']);
            testLayerIndexes=1:ag.countLayers;
            candidates=[];
            for testLayerIndex=testLayerIndexes
                candidateCell=obj.findBestCellOnLayer(ag,testLayerIndex,ag.countLayers);
                if candidateCell ~=0
                    candidates=[candidates,candidateCell];
                end
            end
            candidateCount = length(candidates);
            if candidateCount~=0
                Cmnf.logc(obj,[mat2str(candidateCount),' advance cell(s) found']);
                distances = zeros(1,candidateCount);
                for k=1:candidateCount
                    distances(k)=Cmnf.calculatePointDistance(localGoal,candidates(k).center);
                end
                [bestValue,bestIndex]=min(distances);
                bestCell=candidates(bestIndex);
                bestTrajectory=obj.findBestTrajectoryInCell(bestCell);
            else
                Cmnf.logc(obj,['No reachable cells COLLISION DETECTED!!']);
                collisionFlag=true;
                return
            end

        end
        function candidateCell=findBestCellOnLayer(obj,ag,layerIndex,maxLayerIndex)
            if nargin == 3
                maxLayerIndex=layerIndex;
            end
            candidateCell=0;
            Cmnf.logc(ag,['Looking for best cell on layer: ',mat2str(layerIndex)]);
            localGoal=obj.getLocalCoordiantes(obj.goal);
            layer=ag.layers(layerIndex);
            cellCount=layer.verticalCellCount*layer.horizontalCellCount;
            distance=zeros(1,cellCount);
            reachibility=zeros(1,cellCount);
            for k=1:cellCount
                distance(k) = norm(layer.cells(k).center - localGoal,2);
                if maxLayerIndex ~= layerIndex
                    ijkIndex=layer.cells(k).ijkIndex;
                    hIndex=ijkIndex(2);
                    vIndex=ijkIndex(3);
                    if hIndex == layer.horizontalCellCount || hIndex == 1 || vIndex == layer.verticalCellCount || vIndex == 1
                        reachibility(k) = layer.cells(k).pReachability;
                    else
                        reachibility(k) = 0;
                    end
                else
                    reachibility(k) = layer.cells(k).pReachability;
                end
            end
            candidateIndexes=find(reachibility>=Cmnf.reachibilityTreshold);
            Cmnf.logc(ag,['Cells over treshold:',mat2str(Cmnf.reachibilityTreshold),...
                ' found: ',mat2str(length(candidateIndexes))]);
            if ~isempty(candidateIndexes)
                candidateDistances = distance(candidateIndexes);
                [sorted,indexes]=min(candidateDistances);
                bestIndex = candidateIndexes(indexes);
                bestDistance = candidateDistances(indexes);
                candidateCell = layer.cells(bestIndex);
                Cmnf.logc(ag,['Best cell: ',mat2str(candidateCell.ijkIndex),...
                              ' with distance: ', mat2str(bestDistance),' m'] );   
            end
        end
        
        function bestTrajectory=findBestTrajectoryInCell(obj,candidateCell)
            trajectories=candidateCell.trajectories;
            Cmnf.logc(candidateCell,['Searching best trajectory in cell: ',mat2str(candidateCell.ijkIndex),...
                                     ' going trough ',mat2str(length(trajectories)),' trajector(y/ies)']);
            tCosts=zeros(1,length(trajectories));
            tReachibility=zeros(1,length(trajectories));
            for k=1:length(trajectories)
                tReachibility(k)=trajectories(k).pReachibility;
                tCosts(k)=trajectories(k).trajectoryCost;
            end
            feasibleIndex=find(tReachibility>=Cmnf.reachibilityTreshold);
            Cmnf.logc(candidateCell, ['Found ',mat2str(length(feasibleIndex)),' trajector(y/ies) ',...
                                      'with reachibility over ',mat2str(Cmnf.reachibilityTreshold)]);
            feasibleCosts=tCosts(feasibleIndex);
            [bestValue,localIndex]=min(feasibleCosts);
            bestIndex=feasibleIndex(localIndex);
            bestTrajectory=trajectories(bestIndex);
            Cmnf.logc(bestTrajectory,['Best trajectory id: ',mat2str(bestTrajectory.id),' with performace ',...
                                      mat2str(bestValue)]);
        end
        %% Obstacle functions
        % Put obstacle into obstacle register
        %   obstacles - list of AbstractObstacleObject
        function r=putObstacles(obj,obstacles)
            for k=1:length(obstacles)
                obstacles(k).id=obj.staticObstacleID;
                obj.staticObstacleID=obj.staticObstacleID+1;
                obj.staticObstacles=[obj.staticObstacles,obstacles(k)];
                s= ['[',class(obstacles(k)),']',' Putting obstacle id: ',...
                    mat2str(obstacles(k).id),', center: ',...
                    mat2str(obstacles(k).center),...
                    ' ,', mat2str(obstacles(k).type),...
                    ' ,' ,obstacles(k).getLogString];
                Cmnf.logc(obj,s);
            end
            r=obj.staticObstacles;
        end
        
        function [intersections,collisions]=getIntersectionCollisionCandidates(obj)
            intersections=[];
            collisions=[];
            posOr=obj.vehicle.getActualPositionOrientation;
            pos=posOr(1:3);
            for obs=obj.staticObstacles
               Cmnf.logc(obs,['Testing intersection/collision obstacle id: ',mat2str(obs.id),', center: ',mat2str(obs.center)]);
               if  obs.isIntersection(pos,obj.avoidanceGrid.dEnd)
                   Cmnf.logc(obs,['Intersection detected id: ',mat2str(obs.id)]);
                   intersections=[intersections,obs];
               end
               if obs.isCollision(pos)
                   Cmnf.logc(obs,['[!!] Collision detected id: ',mat2str(obs.id)]);
                   collisions=[collisions,obs];
               end
            end
        end
        
        function r=intersectObstaclesWithGrid(obj,ag,intersections)
            r=0;
            for intersection=intersections
                r=r+obj.intersectObstacleWithGrid(ag,intersection);
            end
        end
        
        function r=intersectObstacleWithGrid(obj,ag,intersection)
            r=0;        
            globPoints=intersection.getPoints;
            locPoints =obj.getLocalCoordiantes(globPoints);
            [m,n]=size(locPoints);
            for k=1:n
                cell = ag.getCellEuclidian(locPoints(1:3,k));
                if cell ~= 0
                    ag.putObstacle(cell,intersection,1);
                    r=r+1;
                end
            end
        end
        
        function r=calculateDistanceToObstacleStatistic(obj)
            recordCount=length(obj.missionLog);
            time =  zeros(1,recordCount);
            position = zeros(3,recordCount); 
            logs=obj.missionLog;
            for k=1:recordCount
                log=logs(k);
                time(k) = log.simulationTime;
                position(:,k)=log.vehiclePosOrBefore(1:3);
            end
            obstacles=obj.staticObstacles;
            obstaclesCount=length(obstacles);
            obstaclePoints=[];
            for k=1:obstaclesCount
                obstaclePoints=[obstaclePoints,obstacles(k).getPoints];
            end
            obstaclePointsCount=length(obstaclePoints);
            distance = zeros(1,recordCount);
            for k=1:recordCount
                shiftedObstaclePoints = obstaclePoints - position(:,k)*ones(1,obstaclePointsCount);
                distance(k)=min(sqrt([1,1,1]*(shiftedObstaclePoints.^2)));
            end
            r=[distance;time];
        end
        %% Intruder functions
        function r=addIntruder(obj,intruder)
            intruder.id=obj.intruderId;
            obj.intruders=[obj.intruders,intruder];
            Cmnf.logc(obj, ['Adding intruder id: ',mat2str(intruder.id),' position: ',mat2str(intruder.localPosition),' velocity: ',mat2str(intruder.localVelocity), ' TOD: ',mat2str(intruder.detectionTime),' s' ] );
            obj.intruderId=obj.intruderId+1;
            r=0;
        end
        
        function r=addIntruders(obj,intruders)
            for intruder=intruders
                obj.addIntruder(intruder);
            end
            r=0;
        end
        
        function r=testIntruderDetection(obj,intruder)
            if intruder.isFirstDetection(obj.simulationTime)
               lpos=intruder.localPosition;
               lvel=intruder.localPosition+intruder.localVelocity;
               gpos=obj.getGlobalCoordinates(lpos);
               gvel=obj.getGlobalCoordinates(lvel)-gpos;
               tStep=0:Cmnf.simStep:intruder.liveTime;
               posTimeMatrix=[(gpos*ones(1,length(tStep)) + gvel*tStep);tStep+intruder.detectionTime;gvel*ones(1,length(tStep))];
               intruder.posTime=posTimeMatrix;
               Cmnf.logc(intruder,['Intruder id: ',mat2str(intruder.id),' detected generating trajectory']);
            end
            if intruder.isDetected(obj.simulationTime)        
                Cmnf.logc(intruder,['Intruder id: ',mat2str(intruder.id),' active'])
            end
            r=intruder.isFirstDetection(obj.simulationTime)||intruder.isDetected(obj.simulationTime);
        end
        
        function r=getDetectedIntruders(obj)
            r=[];
            for intruder=obj.intruders
                if obj.testIntruderDetection(intruder)
                    r=[r,intruder];
                end
            end
        end
        
        function r=getIntruderPositionAndVelocityOnTime(obj,intruder)
            simTime=obj.simulationTime;
            pos=intruder.posTime(1:3,:);
            time=intruder.posTime(4,:);
            vel=intruder.posTime(5:7,:);
            index=find(time==simTime);
            gPos = pos(:,index);
            gVel = pos(:,index) + vel(:,index);
            lPos = obj.getLocalCoordiantes(gPos);
            lVel = obj.getLocalCoordiantes(gVel)-lPos;
            r=[lPos;lVel];
        end
        
        function r=getIntrudersPositionAndVelocityOnTime(obj,detectedIntruders)
            if nargin ==1
                detectedIntruders = obj.getDetectedIntruders;
            end
            r=[];
            for intruder=detectedIntruders
                r=[r,obj.getIntruderPositionAndVelocityOnTime(intruder)];
            end
        end
        
        function r=createLocalAdversarialObjects(obj,detectedIntruders)
            if nargin ==1
                detectedIntruders = obj.getDetectedIntruders;
            end
            posVel=obj.getIntrudersPositionAndVelocityOnTime(detectedIntruders);
            [m,n]=size(posVel);
            adversaries = [];
            for k=1:n
                wIntruder=detectedIntruders(k);
                intersectionConfig=wIntruder.intersectionConfig;
                pos = posVel(1:3,k);
                vel = posVel(4:6,k);
                adversaryPoint = pos;               %Adversary initial location
                adversaryVelocity =  vel;           %Adversary initial velocity
                adversaryTimeError = intersectionConfig.timeError;              %time error
                adversaryRadius = intersectionConfig.radius;                    %radius of adversary
                thetaSpread = intersectionConfig.thetaSpread;
                phiSpread = intersectionConfig.phiSpread;
                %Create adversary object
                ta=TimedAdversaryVehicle(adversaryPoint,adversaryVelocity,adversaryRadius,adversaryTimeError,thetaSpread,phiSpread);
                ta.flagTimeIntersection = intersectionConfig.flagTimeIntersection || obj.flagTimeIntersection;    %Enforce time
                ta.flagFutureMovements = intersectionConfig.flagFutureMovements || obj.flagFutureMovements;       %Enforce future movements
                ta.flagBallIntersection = intersectionConfig.flagBallIntersection || obj.flagBallIntersection;    %Enforce ball intersection
                ta.flagSpread = intersectionConfig.flagSpread || obj.flagSpread;                                  %Use spread
                ta.flagOnlySpread = intersectionConfig.flagOnlySpread || obj.flagOnlySpread;                      %DO not Use line intersection
                %Put out log
                Cmnf.logc(obj,  ['Intruder intersection for mission control: ',mat2str(obj.vehicleId),', commanding vehicle',obj.vehicleName]);
                Cmnf.logc(ta, [' -- Intruder parameters:']);
                Cmnf.logc(ta, ['    -- cooperative(1)/nonCooperative(0): ',mat2str(wIntruder.cooperative)]);
                Cmnf.logc(ta, ['    -- external(cooperative)/internal(noncooperative) id: ',mat2str(wIntruder.id)]);
                Cmnf.logc(ta, ['    -- position (glob):         ',mat2str(ta.position)]);
                Cmnf.logc(ta, ['    -- velocity (vect):         ',mat2str(ta.velocity)]);
                Cmnf.logc(ta, ['    -- body radius (m):         ',mat2str(ta.radius)]);
                Cmnf.logc(ta, ['    -- distance er.(m):         ',mat2str(ta.distanceError)]);
                Cmnf.logc(ta, ['    -- horizontal spread (rad): ',mat2str(ta.thetaSpread/pi)]);
                Cmnf.logc(ta, ['    -- vertical spread (rad):   ', mat2str(ta.phiSpread/pi)]);
                Cmnf.logc(ta, [' -- Intersection parameters:']);
                Cmnf.logc(ta, ['    -- time t(1)/f(0):          ', mat2str(ta.flagTimeIntersection)]);
                Cmnf.logc(ta, ['    -- future mov. t(1)/f(0):   ',mat2str(ta.flagFutureMovements)]);
                Cmnf.logc(ta, ['    -- body volume t(1)/f(0):   ',mat2str(ta.flagBallIntersection)]);
                Cmnf.logc(ta, ['    -- spread t(1)/f(0):        ',mat2str(ta.flagSpread)]);
                Cmnf.logc(ta, ['    -- only spread t(1)/f(0):   ',mat2str(ta.flagOnlySpread)]);
                Cmnf.logc(ta, ['    -- calculated t(1)/f(0):    ',mat2str(ta.flagCalculated)]);
                adversaries=[adversaries,ta];
            end
            r=adversaries;
        end
        
        function r=intersectIntrudersWithGrid(obj,ag,detectedIntruders)
            if nargin==1
                ag=obj.avoidanceGrid;
                detectedIntruders=obj.getDetectedIntruders;
            end
            if nargin==2
                detectedIntruders=obj.getDetectedIntruders;
            end
            adversaries=obj.createLocalAdversarialObjects(detectedIntruders);
            r=0;
            for adversary=adversaries
                p=ag.putTimedAdversarial(adversary);
                [m,n]=size(p);
                r=r+n;
            end
        end
        %% Plot functions
        function r=plotWaypoints(obj)
            Cmnf.logc(obj,'[PLOT] Starting waypoint plotting')
            k=1;
            for wp=obj.waypoints
                hold on
                    pos=wp.position;
                    t = ['  WP ',mat2str(k),'.'];
                    if obj.vehicleId > 0
                        t = ['  WP ',mat2str(obj.vehicleId),'-',mat2str(k),'.'];
                    end
                    plot3(pos(1),pos(2),pos(3),'Marker','square',...
                          'MarkerFaceColor','g','MarkerEdgeColor','r');
                    text(pos(1),pos(2),pos(3),t);
                    Cmnf.logc(obj,['[PLOT] ','putting WP ',t,...
                                   '. position: ',mat2str(pos)]);
                    k=k+1;
                hold off
            end
            Cmnf.logc(obj,'[PLOT] End of waypoint plotting')
            r=0;
        end
        
        function r=plotObstacles(obj)
            for stOb=obj.staticObstacles
                Cmnf.logc(obj,['[PLOT] putting obstacle id: ',...
                          mat2str(stOb.id),' ',mat2str(stOb.type),...
                          ' ',class(stOb),' center:',mat2str(stOb.center)]);
                stOb.plot;
            end
            r=0;
        end
        
        function r=plotRasterRange(obj)
            obj.avoidanceGrid.plotRasterRange(obj.vehicle.getActualPositionOrientation);
            r=0;
        end
        
        function r=plotMissionStaticContent(obj)
            obj.plotWaypoints;
            obj.plotObstacles;
            obj.plotStaticConstraints;
            xlabel('x[m]')
            ylabel('y[m]')
            zlabel('z[m]')
            grid on;
            r=0;
        end
        
        function r=plotMissionTrajectory(obj)
            hold on
            %plot flew trajectory
            handles=obj.vehicle.plotTrajectory(obj.trajectoryColor);
            hold off
            lm=obj.avoidanceGrid.linearModel;
            prediction=lm.predictBuffer(obj.vehicle.getActualPositionOrientation,obj.movementBuffer);
            hold on 
            %plot predicted trajectory
            predictHandle=plot3(prediction(1,:),prediction(2,:),prediction(3,:),obj.plannedColor);
            handles=[handles,predictHandle];
            % plot moving obstacles
            movingObstaclesHandles=obj.plotMovingConstraints;
            handles=[handles,movingObstaclesHandles];
            obj.lastTrajectoryPlotHandles=obj.actualTrajectoryPlotHandles;
            obj.actualTrajectoryPlotHandles=handles;
            hold off
            % Generate vehicle status line
            posOR=obj.vehicle.getActualPositionOrientation;
            pst=sprintf('[%0.2f,%0.2f,%0.2f]^T',posOR(1),posOR(2),posOR(3));
            ost=sprintf('\\alpha = %0.2f \\circ,\\beta = %0.2f \\circ,\\gamma = %0.2f \\circ,',rad2deg(posOR(4)),rad2deg(posOR(5)),rad2deg(posOR(6)));
            gst=sprintf('[%0.2f,%0.2f,%0.2f]^T',obj.goal(1),obj.goal(2),obj.goal(3));
            tst=['Position:',pst,', Orientation:',ost,' Goal:',gst];
            
            % If not part of UTM (UTM assigns non zero ID force title)
            if obj.vehicleId ==0
                title(tst);
            end
            
            %Log plot details
            Cmnf.logc(obj.vehicle,['[PLOT]',tst]);
            Cmnf.logc(obj.vehicle,['[PLOT] Movement buffer: ',Cmnf.movementBuffer2String(obj.movementBuffer)]);
            r=0;
        end
        
        function r=plotGridSlice(obj,ag,statType,fig)
            figure(fig)
            layCount=ag.countVertical;
            if layCount < 2
                plotwidht=layCount +1;
            else
                plotwidht=3;
            end
            rowCount=ceil((1+layCount)/plotwidht);
            
            subplot(rowCount,plotwidht,1)
            ag.plotReachSetColored(statType)

            for k=1:layCount
            subplot(rowCount,plotwidht,k+1)
            ag.plotHorizontalSlice(k,statType);
            title(['Layer ',mat2str(k),'.']);
            end
            r=0;
        end
        
        function r=plotObstacletoDistanceStatistic(obj)
            % retrieve statistical data;
            smd=obj.calculateDistanceToObstacleStatistic;
            wsmd=[];
            % create intersections of various levels
            for k=1:(length(smd)-1)
                margin1 = smd(1,k);
                margin2 = smd(1,k+1);
                cat1=0;
                cat2=0;
                if (margin1 <= Cmnf.plotRedDistande)
                    cat1=1;
                end
                if (margin1 >= Cmnf.plotRedDistande && margin1 < Cmnf.plotYellowDistance)
                    cat1=2;
                end
                if (margin1 >= Cmnf.plotYellowDistance)
                    cat1=3;
                end
                if(margin2 <= Cmnf.plotRedDistande)
                    cat2=1;
                end
                if (margin2 >= Cmnf.plotRedDistande && margin2 < Cmnf.plotYellowDistance)
                    cat2=2;
                end
                if (margin2 >= Cmnf.plotYellowDistance)
                    cat2=3;
                end
                if cat1~=cat2
                    lapse=0;
                    if cat1==1 && cat2==2 || cat1==2 && cat2==1
                        lapse = Cmnf.plotRedDistande;
                    end
                    if cat1==2 && cat2==3 || cat1==3 && cat2==2
                        lapse = Cmnf.plotYellowDistance;
                    end

                    velocity=(smd(1,k+1)-smd(1,k))/(smd(2,k+1)-smd(2,k));
                    t_add=abs(smd(1,k)-lapse)/abs(velocity);
                    intersection=[lapse;smd(2,k)+t_add];
                    intersection2=intersection;
                    intersection2(1)=intersection2(1)-0.00000000001;
                    if cat1==2 && cat2==3 || cat1==1 && cat2==2
                        wsmd=[wsmd,smd(:,k),intersection];
                    else
                        wsmd=[wsmd,smd(:,k),intersection2];
                    end
                else
                    wsmd=[wsmd,smd(:,k)];
                end
                if k == (length(smd)-1)
                    wsmd=[wsmd,smd(:,k+1)];
                end
            end
            smd=wsmd;
            % Plot fuction
            hold on
            grid on
            for k=1:(length(smd)-1)
                margin = smd(1,k);
                if(margin <= Cmnf.plotRedDistande)
                    plot([smd(2,k);smd(2,k+1)],[smd(1,k);smd(1,k+1)],'r','LineWidth',2)
                end
                if (margin >= Cmnf.plotRedDistande && margin < Cmnf.plotYellowDistance)
                    plot([smd(2,k);smd(2,k+1)],[smd(1,k);smd(1,k+1)],'y','LineWidth',2)
                end
                if (margin >= Cmnf.plotYellowDistance)
                    plot([smd(2,k);smd(2,k+1)],[smd(1,k);smd(1,k+1)],'g','LineWidth',2)
                end
            end
            axis([0, max(smd(2,:)),0,max(smd(1,:))])
            title('Crash distance to nearest obstacle evolution');
            xlabel('Flight time [s]');
            ylabel('Crash distance d_c [m]');
            plot([0,max(smd(2,:))],[Cmnf.plotCrashDistande Cmnf.plotCrashDistande],'r-.')
            plot([0,max(smd(2,:))],[Cmnf.plotRedDistande Cmnf.plotRedDistande],'y-.')
            plot([0,max(smd(2,:))],[Cmnf.plotYellowDistance Cmnf.plotYellowDistance],'g-.')
            hold off
            r=0;
        end
        
        function r=plotRealvsPlanTrajectoryStatistics(obj)
            x_pos=obj.vehicle.state.x_pos;
            y_pos=obj.vehicle.state.y_pos;
            z_pos=obj.vehicle.state.z_pos;
            startPoint=[x_pos(1);y_pos(1);z_pos(1)];
            time=obj.vehicle.state.time;
            logs=obj.missionLog;
            logsSize=length(logs);
            reachedWayPoints=[];
            intersections=[];
            for k=1:logsSize
                log = logs(k);
                if ~isempty(log.intersections)
                    for l=1:length(log.intersections)
                        inters=log.intersections(l);
                        intersections=[intersections,[inters.center;log.simulationTime;inters.id]];
                    end

                end
                if log.waypointReachedFlag
                    reachedWayPoints=[reachedWayPoints,[log.waypoint;log.simulationTime]];
                end
            end
            if ~isempty(reachedWayPoints)
                reachedWayPoints=[[logs(1).vehiclePosOrBefore(1:3);logs(1).simulationTime],reachedWayPoints];
            else
                ll=length(logs);
                reachedWayPoints=[[logs(ll).vehiclePosOrBefore(1:3);logs(ll).simulationTime]];
            end
            minTime=logs(1).simulationTime;
            maxTime=max(reachedWayPoints(4,:));
            [m,n]=size(reachedWayPoints);
            reachedWayPointsSize= n;
            endofSim=logs(logsSize).simulationTime;
            try
                reachedWayPoints(4,:)=[reachedWayPoints(4,2:reachedWayPointsSize) endofSim];
            catch E
                reachedWayPoints(4,1)=endofSim;
            end
            trajectoryWaypoints=[[startPoint;minTime],reachedWayPoints];
            [m,n]=size(trajectoryWaypoints);
            trajectoryWaypointsCount=n-1;
            intersectionsSize=length(intersections);
            subplot(3,1,1)
            title('Desired vs. real trajectory')
            grid on
            hold on
            plot(time,x_pos,'b','LineWidth',4);
            for k=1:trajectoryWaypointsCount
                plot([trajectoryWaypoints(4,k),trajectoryWaypoints(4,k+1)],[trajectoryWaypoints(1,k),trajectoryWaypoints(1,k+1)],'--g','LineWidth',4);
                %plot([[reachedWayPoints(4,k),reachedWayPoints(4,k)]],[reachedWayPoints(1,k),reachedWayPoints(1,k+1)],'-m');
            end
            %for k=1:intersectionsSize
                %plot(intersections(4,k),intersections(1,k),'Marker','o','MarkerFaceColor','r','MarkerEdgeColor','r');
            %end
            plot(minTime,startPoint(1),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
            text(minTime-0.5,startPoint(1),'S.');
            for k=1:reachedWayPointsSize
                plot(reachedWayPoints(4,k),reachedWayPoints(1,k),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
                text(reachedWayPoints(4,k)-0.5,reachedWayPoints(1,k),[mat2str(k),'.']);
            end
            hold off
            if min([reachedWayPoints(1,:),x_pos])< max([reachedWayPoints(1,:),x_pos])
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(1,:),x_pos]),max([reachedWayPoints(1,:),x_pos])]);
            else
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(1,:),x_pos])-1,max([reachedWayPoints(1,:),x_pos])+1]);
            end
            xlabel('Time [s]')
            ylabel('x [m]')

            subplot(3,1,2)
            grid on
            hold on
            plot(time,y_pos,'b','LineWidth',4);
            %for k=1:(reachedWayPointsSize-1)
            %    plot([reachedWayPoints(4,k),reachedWayPoints(4,k+1)],[reachedWayPoints(2,k),reachedWayPoints(2,k+1)],'--g','LineWidth',4);
            %    %plot([[reachedWayPoints(4,k),reachedWayPoints(4,k)]],[reachedWayPoints(2,k),reachedWayPoints(2,k+1)],'-m');
            %end
            for k=1:trajectoryWaypointsCount
                plot([trajectoryWaypoints(4,k),trajectoryWaypoints(4,k+1)],[trajectoryWaypoints(2,k),trajectoryWaypoints(2,k+1)],'--g','LineWidth',4);
            end
            %for k=1:intersectionsSize
            %    plot(intersections(4,k),intersections(2,k),'Marker','o','MarkerFaceColor','r','MarkerEdgeColor','r');
            %end
            plot(minTime,startPoint(2),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
            text(minTime-0.5,startPoint(2),'S.');
            for k=1:reachedWayPointsSize
                plot(reachedWayPoints(4,k),reachedWayPoints(2,k),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
                text(reachedWayPoints(4,k)-0.5,reachedWayPoints(2,k),[mat2str(k),'.']);
            end
            hold off
            if min([reachedWayPoints(2,:),y_pos])<max([reachedWayPoints(2,:),y_pos])
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(2,:),y_pos]),max([reachedWayPoints(2,:),y_pos])]);
            else
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(2,:),y_pos])-1,max([reachedWayPoints(2,:),y_pos])+1]);
            end
            xlabel('Time [s]')
            ylabel('y [m]')

            subplot(3,1,3)
            grid on
            hold on
            plot(time,z_pos,'b','LineWidth',4);
            %for k=1:(reachedWayPointsSize-1)
            %    plot([reachedWayPoints(4,k),reachedWayPoints(4,k+1)],[reachedWayPoints(3,k),reachedWayPoints(3,k+1)],'--g','LineWidth',4);
            %    %plot([[reachedWayPoints(4,k),reachedWayPoints(4,k)]],[reachedWayPoints(3,k),reachedWayPoints(3,k+1)],'-m');
            %end
            for k=1:trajectoryWaypointsCount
                plot([trajectoryWaypoints(4,k),trajectoryWaypoints(4,k+1)],[trajectoryWaypoints(3,k),trajectoryWaypoints(3,k+1)],'--g','LineWidth',4);
            end
            %for k=1:intersectionsSize
            %    plot(intersections(4,k),intersections(3,k),'Marker','o','MarkerFaceColor','r','MarkerEdgeColor','r');
            %end
            plot(minTime,startPoint(3),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
            text(minTime-0.5,startPoint(3),'S.');
            for k=1:reachedWayPointsSize
                plot(reachedWayPoints(4,k),reachedWayPoints(3,k),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r','MarkerSize',15);
                text(reachedWayPoints(4,k)-0.5,reachedWayPoints(3,k),[mat2str(k),'.']);
            end
            hold off
            %Treshold problem
            if min([reachedWayPoints(3,:),z_pos])< max([reachedWayPoints(3,:),z_pos])
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(3,:),z_pos]),max([reachedWayPoints(3,:),z_pos])]);
            else
                axis([min([reachedWayPoints(4,:),time]),max([reachedWayPoints(4,:),time]),min([reachedWayPoints(3,:),z_pos])-1,max([reachedWayPoints(3,:),z_pos])+1]);
            end
            
            xlabel('Time [s]')
            ylabel('z [m]')
        end
        
        function plotDetectedIntruders(obj,detectedIntruders)
            if nargin == 1 
                Cmnf.logc(obj,'[PLOT] No Intruders detected attempting intruder detection');
                detectedIntruders=obj.getDetectedIntruders;
            end
            for intruder=detectedIntruders
                %check if intruder is detected or cooperative
                if intruder.cooperative == false 
                    Cmnf.logc(intruder,['[PLOT] puting trajectory of intruder id: ',mat2str(intruder.id),' sim time: ',mat2str(obj.simulationTime),' s']);
                    intruder.plotTimedTrajectory(obj.simulationTime);
                end
            end
        end
        
        function plotRemoveLastTrajectoryFromMissionFigure(obj)
            if ~isempty(obj.lastTrajectoryPlotHandles)
                for handle=obj.lastTrajectoryPlotHandles
                    delete(handle);
                end
                obj.lastTrajectoryPlotHandles=[];
            end
        end
        %% Coordinate transformation methods
        function r=getLocalCoordiantes(obj,mat)
            posOr=obj.vehicle.getActualPositionOrientation;
            pos=posOr(1:3);
            rot=posOr(4:6);
            r=Cmnf.glob2loc(pos,rot,mat);
        end
        
        function r=getGlobalCoordinates(obj,mat)
            posOr=obj.vehicle.getActualPositionOrientation;
            pos=posOr(1:3);
            rot=posOr(4:6);
            r=Cmnf.loc2glob(pos,rot,mat);
        end
        
        %% ADS-B notification method corpus
        %   id of intruder at UTM
        %   intruderPosOR position and orientation at time of message
        %                 retrieval
        %   intruderVelocity - linearized intruder velocity vector in
        %   global coordiante system
        %   timeError - time offset of transition usually as 1s for
        %   standard condiguration
        %   radius - intruder body radius in meters in ADSB size class
        %   thetaSpread - horizontal maneuvaribility 0-pi/2
        %   phiSpread - vertical maneuvaribility 0-pi/2
        function r=notifyIntruder(obj,id,intruderPosOR,intruderVelocity,timeError,radius,thetaSpread,phiSpread)
            if nargin <= 4
                timeError = 1;               
            end
            if nargin <= 5 
                radius = 1;                  
            end
            if nargin <=6
                thetaSpread = pi/8;          
            end
            if nargin <=7
                phiSpread = pi/12;
            end
            % intruderRelativePosition
            localIntruderPosition=obj.getLocalCoordiantes(intruderPosOR(1:3));
            posOr=obj.vehicle.getActualPositionOrientation;
            pos=posOr(1:3);
            or=posOr(4:6);
            globalIntruderVelocity=Cmnf.loc2glob([0;0;0],intruderPosOR(4:6),[intruderVelocity;0;0]);
            localIntruderVelocity=Cmnf.glob2loc([0;0;0],or,globalIntruderVelocity);
            %r=[localIntruderPosition;localIntruderVelocity];
            simulationTime=obj.simulationTime;
            Cmnf.logc(obj, ['[Notification] Intruder id: ',mat2str(id),'  local position: ',mat2str(localIntruderPosition),' local velocity', mat2str(localIntruderVelocity),' simualtion time: ',mat2str(simulationTime)]);
            intruder=Intruder(localIntruderPosition,localIntruderVelocity,simulationTime);
            % Create cooperative intersection config with parameters
            % retrieved from ADS-B
            intruder.intersectionConfig = IntruderIntersectionType.getIntersectionConfig(IntruderIntersectionType.Cooperative);
            intruder.intersectionConfig.timeError=timeError;
            intruder.intersectionConfig.radius=radius;
            intruder.intersectionConfig.thetaSpread=thetaSpread;
            intruder.intersectionConfig.phiSpread=phiSpread;
            
            % set live time to 20 seconds (smart live time depending on grid should be implemented)
            intruder.liveTime=20; 
            % set intruder id
            intruder.id=id;
            % set cooperative flag - triggers additional intersection
            % logic, at least I hope so :D
            intruder.cooperative=true;
            cIntruders=length(obj.intruders); 
            rCandidate=0;
            for k=1:cIntruders
                if id == obj.intruders(k).id
                    rCandidate=k;
                end
            end
            if rCandidate~=0
                obj.intruders(rCandidate) = [];
                 Cmnf.logc(obj, ['[Notification] Replacing old Intruder record, id: ',mat2str(id),]);
            end
            obj.intruders = [obj.intruders,intruder];
            r=obj.intruders;
        end
        
        function r=getVehiclePositionNotification(obj)
            posOr=obj.vehicle.getActualPositionOrientation;
            pos=posOr(1:3);
            or=posOr(4:6);
            vel=Cmnf.loc2glob([0;0;0],posOr(4:6),[obj.vehicleActualVelocity;0;0]);
            sm=obj.vehicleSafetyMargin;
            r=VehiclePositionNotification(pos,or,vel,sm);
        end
        
        function r=isActive(obj)
            if (obj.finalWaypointReachedFlag ==0)
                r=true;
            else
                r=false;
            end
        end
        %% Predictor funtcions
        function r=predictTrajectory(obj)
            lastState=obj.vehicle.state.getLastState;
            movementBuffer=obj.movementBuffer;
            stepCount=length(obj.avoidanceGrid.layers); %length of layer = length of step => works only in assumption holds
            if (stepCount>length(movementBuffer))
                diff= stepCount-length(movementBuffer);
                for k=1:diff
                    movementBuffer=[movementBuffer,MovementType.Straight];
                end
            end
            state=obj.vehicle.state.getLastStatePredictor;
            lm=LinearizedModel;
            prediction=lm.predictBuffer(state,movementBuffer);
            r=prediction([1:3],:);
        end
        
        %% Constraint methods
        % Put obstacle into obstacle register
        %   obstacles - list of AbstractObstacleObject
        function r=putConstraints(obj,constraints)
            for k=1:length(constraints)
                constraints(k).id=obj.constraintID;
                obj.constraintID=obj.constraintID+1;
                obj.constraints=[obj.constraints,constraints(k)];
                s= ['[',class(constraints(k)),']',' Putting constraints id: ',...
                    mat2str(constraints(k).id),', center: ',...
                    mat2str(constraints(k).center),...
                    ' ,', mat2str(constraints(k).type),...
                    ' ,' ,constraints(k).getLogString];
                Cmnf.logc(obj,s);
            end
            r=obj.staticObstacles;
        end
        
        %plot constraints
        function r=plotStaticConstraints(obj)
            for c = obj.constraints
                if c.type == CostraintType.Static
                    c.plot();
                end
            end
        end
        
        function r=plotMovingConstraints(obj)
            r=[];
            for c = obj.constraints
                if c.type == CostraintType.Moving
                    h=c.plot();
                    r=[r,h];
                end
            end
        end
        
        function [hcf,scf,hc,sc] = getConstraintsCandidates(obj);
            % Initial data structure preparation
            obj.activeSHCostraints=[];
            obj.hardConstraintFlag=false;
            obj.softConstraintFlag=false;
            hc=[];
            sc=[];
            
            % check hard Constraints()
            for cc=obj.constraints
                tf = cc.breachibility == CostraintType.Hard;
                tc = cc.inRange(obj);
                if tf && tc
                    hc = [hc,cc]; 
                end
            end
            obj.hardConstraintFlag = ~isempty(hc);
            
            %check softConstraint (TBI)
            obj.softConstraintFlag = false;
            
            
            %return structure preparation
            obj.activeSHCostraints=[hc,sc];
            hcf=obj.hardConstraintFlag;
            scf=obj.softConstraintFlag;
        end
        
        % application of hard constraints one by one
        function r=applyHardConstraints(obj,ag)
            r=0;
            mc=obj;
            for cc = obj.activeSHCostraints
                r=r+ag.applyHardConstraint(mc,cc);
            end
        end
        
        % moving constraints
        function movingConstraints=moveConstraints(obj)
            mc=obj; 
            movingConstraints=[];
            for cc=obj.constraints
                if cc.type == CostraintType.Moving
                    movingConstraints=[movingConstraints,cc];
                    cc.applyMovement(mc);
                end
            end
        end
        
        
        %% Rule engine create context (DO NOT CALL DIRECTLY)
        function r=createContextRuleEngine(obj)
            createContextRuleEngine@RullableObject(obj);
            obj.reContext('missionControl')=obj;
            %constraints
            obj.reContext('constraints') =  obj.constraints;
            
            %Navigation grid
            obj.reContext('navigationGrid')=obj.navigationGrid;
            obj.reContext('navigationType')=obj.navigationType;
            obj.reContext('navigationParams')=obj.navigationParams;
        
            %Avoidance grid
            obj.reContext('avoidanceGrid')=obj.avoidanceGrid;
            obj.reContext('avoidanceType')=obj.avoidanceType;
            obj.reContext('avoidanceParams')=obj.avoidanceParams;
            r=obj.reContext;
        end
        
        %% Rule engine injection method
        function r=injectRuleEngine(obj,ruleEngine)
            masterFlag=injectRuleEngine@RullableObject(obj,ruleEngine);
            % TODO injection body
            obj.navigationGrid.injectRuleEngine(ruleEngine);
            obj.navigationGrid.appendContextRuleEngine(obj);
            obj.avoidanceGrid.injectRuleEngine(ruleEngine);
            obj.avoidanceGrid.appendContextRuleEngine(obj);
            r=masterFlag;
        end
    end
    
end

