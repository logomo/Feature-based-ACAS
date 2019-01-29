classdef PredictorNode<LoggableObject
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Global properties
        counter=0;      % Static counter for ID assignement
        id=0;           % Real ID root =0
        mycell = '';    % cell hash - belonging cell
        hash = '';      % trajectory hash - list of passed cells
        depth=0;        % tree depth 0 - root ex - 7 -> 7 parrent nodes exist
        root=0;         % root reference 
        parrent=0;      % parrent reference
        referencedCell=0;   %referenced cell in blended avoidance grid
        state;          % Linear model simulated state of trajectory
        movement;       % Movement executed to get into this state
        leafs=[];       % List of leafs - childs, followers, or whatever is correct name ...
        cellDistance=0; % Distance to cell center ||x0-x_c_i,j,k||
        expanded = 0;   % Indication if node vas expanded - unsuprisingly 0
        reachFinalLayer=0; % Indication if node reached final layer - used in pruning
        reachTime = 0;  % Reach time of cell;
        cellEntryTime=0; % Entry time to cell
        cellLeaveTime=0; % Leave time from cell
        movementCost=0;  % Unitary movement cost 
        trajectoryCost=0; % trajectory cost
        directTrajectoryFlag=true;
        movementGroups %[] for ACAS trees only
        isACASPreffered=false;
        
        %Statistical properties
        pFeasibility=1;     %trajectory probability of feasibility 
        pObstacle = 0;      %trajectory probability of obstacle
        pVisibility = 1;    %trajectory probability of visibility
        pReachibility = 1;  %trajectory probability of reachibility
        cFeasibility = 1;   %movement probability of feasibility
        cObstacle = 0;      %movement probability of obstacle
        cVisibility = 1;    %movement probability of visibility
        cReachibility = 1;  %movement probability of reachibility 
        rFeasibility = 1;   %rule ratio of feasibility (100% unchanged by rule)
        rObstacle = 1;      %rule ratio of obstacle    (100% unchanged by rule)
        rVisibility = 1;    %rule ratio of visibility  (100% unchanged by rule)
        rReachibility = 1;  %rule ratio of reachibility  (100% unchanged by rule)
        
        %Rule engine parameters
        flagDistanceFeasibility = true;
        flagHeadingFeasibility =true;
    end
    
    methods
        function obj=PredictorNode(state,id,movementType)
            obj.state=state;
            obj.id=id;
            obj.movement=MovementType(movementType);
        end
        %% Standard expansion
        % Plain expansion, used for normal expansion of selected node
        %  r - list of newly created leafs
        %  obj - parrent reference
        %  grid - avoidance grid object handle
        %  linearModel - system linearized model for working point
        function r=normalExpand(obj,grid,linearModel)                                 
            if obj.expanded == 0                                            % Check if node has been expanded, if not continue for expansion
                for k=0:8                                                   % For each movement from movement bank -k=0:8
                    newstate=linearModel.predict(obj.state,k);              % Create new state based on parrent state, 
                    [m,n] = size(newstate);                                 
                    pos = newstate(1:3,n);                                  % Fetch position from new state
                    cell=grid.getCellEuclidian(pos);                        % Fetch cell based on new position
                    if cell~=0                                              % If there exists cell where trajectory belongs, continue, else skip 
                        obj.root.counter = obj.root.counter + 1;            % Create unique ID for trajectory
                        newId = obj.root.counter;                           
                        worker = PredictorNode(newstate,newId,k);           % Create node object, with state, id, movementType
                        worker.depth=obj.depth+1;                           % calculate node tree depth based on parrent node
                        worker.root=obj.root;                               % Set root
                        worker.parrent=obj;                                 % set parrent as obj handle
                        worker.referencedCell=cell;                         % set cell reference as fetched cell
                        worker.leafs=[];                                    % initialize leaf set
                        worker.hash = [obj.hash,mat2str(cell.ijkIndex)];    % calculate hash as trajectory footprint trough root->worker (full hash)
                        worker.mycell = mat2str(cell.ijkIndex);             % create cell hash
                        worker.reachTime = newstate(7,n);                   % Set cell reach time
                        obj.leafs=[obj.leafs,worker];                       % append parrent leaf set 
                        worker.cellDistance=sqrt(sum(cell.center(2:3)...    % calculate cell distance (timed adversary parameter)
                            -pos(2:3)).^2);
                        if worker.parrent.directTrajectoryFlag              % indication if trajectory is direct (build only from straight movements)
                            worker.directTrajectoryFlag = ...
                                worker.movement == MovementType.Straight;
                        else
                            worker.directTrajectoryFlag=false;              
                        end
                        cell.registerTrajectory(worker);                    % self registration of trajectory in cell register
                        if Cmnf.debugTree
                            Cmnf.logc(obj,['--DEBUG Node: ',mat2str(worker.id),' Layer: ',mat2str(worker.depth)]);
                        end
                    end
                end
                r = obj.leafs;
            else
                r = [];
            end
            obj.expanded = 1;                                               % mark node as expanded
        end
        %% Chaotic expansion
        % obj object reference
        % grid avoidance grid reference
        % linearModel - linearized model for predictor function
        % tReg - trajectory register
        % tMax - trajectory maximum - expansion ratio - default 9
        function r=expand(obj,grid,linearModel,tReg,tMax)
            if obj.expanded == 0                                             % Initialize movement set
                if obj.movement== 0                                          % If parrent.movement == Straight
                    movementSet = 0:8;                                       % Use full movement set 0:8
                else
                    movementSet = [0,obj.movement];                          % Else use constrained movement set Straight + Actual Movement type
                end
                for k=movementSet                                            % For each movement in movement set create leaf
                    newstate=linearModel.predict(obj.state,k);               % Predict new state, based on parrent state + movement type -k
                    [m,n] = size(newstate);
                    pos = newstate(1:3,n);                                   % Extract new position from leaf
                    cell=grid.getCellEuclidian(pos);                         % Try to fetch belonging cell
                    if cell~=0                                               % If cell was fetched
                        obj.root.counter = obj.root.counter + 1;             % Create unique ID for node
                        newId = obj.root.counter;                            
                        worker = PredictorNode(newstate,newId,k);            % Create new leaf node with state, id, movement type
                        worker.depth=obj.depth+1;                            % We went deeper by one step (tree depth~= grid layer!)
                        worker.root=obj.root;                                % set root reference
                        worker.parrent=obj;                                  % set parrent == obj reference
                        worker.referencedCell=cell;                          % set referenced cell
                        worker.leafs=[];                                     % No leafs
                        worker.hash = [obj.hash,mat2str(cell.ijkIndex)];     % Create hash- append cell footprint - for variations
                        worker.mycell = mat2str(cell.ijkIndex);              % Create cell hash footprint - for referencing purposes 
                        worker.reachTime =  newstate(7,n);                   % calculate cell reach time
                        %check expandabilibity
                        thash= [obj.mycell,worker.mycell];
                        if (worker.depth>2)                                  % If depth > 2 steps, check last 3 passing cells, that is sufficient for variability
                            thash= [obj.parrent.mycell,obj.mycell,worker.mycell];
                        end
                        if isKey(tReg,thash)                                 % Variability recording
                            tcount = tReg(thash);                            % check if cell footprint existsd
                            tReg(thash)= tcount +1;                          % increase uniqunes footprint
                            if tcount >= tMax                                % Check if treshold is passed, if there is more paths leading with same footprint, do not mark them as candidates
                                worker.expanded=1;
                            else
                                worker.expanded=0;
                            end
                        else
                            tReg(thash)=1;
                            worker.expanded = 0;
                        end
                        if worker.expanded == 0                             % if worker is expandable add it to main tree
                            obj.leafs=[obj.leafs,worker];
                            if Cmnf.debugTree
                                Cmnf.logc(obj,['--DEBUG Node: ',mat2str(worker.id),' Layer: ',mat2str(worker.depth)]);
                            end
                        end
                    end
                end
                r = obj.leafs;                                              % return set of leafs 
            else
                r = [];
            end
            obj.expanded = 1;                                               % set parrent node expansion flag when finished
        end
        %% Cell expansion function for ACAS-XU like tree
        %   obj - object refe3rence
        %   grid - AvoidanceGrid reference
        %   linearModel - lineralized model for predictions
        %   separations - availible separations
        function r=expandACAS(obj,grid,linearModel,separations)
            % check if path was expanded
            if obj.expanded == 0
                %Cmnf.logc(obj,['Expanding node ', mat2str(obj.id)]);
                %Generate available movements from trajectory
                %charasteristics and available separations
                possibleGroups=MovementGroup.getTrajectoryGroups(obj);
                obj.movementGroups=possibleGroups;
                avialableGroups=intersect(possibleGroups,separations);
                %Cmnf.logc(obj,['  -- For groups ', mat2str(avialableGroups)]);
                %Cmnf.logc(obj,['  -- Layer      ', mat2str(obj.depth)]);
                
                %Generate movement set from available groups
                movementSet=[];
                for k=1:length(avialableGroups)
                    movementSet=[MovementGroup.getMovementGroupMembers(...
                        avialableGroups(k)),movementSet];
                end
                movementSet=unique(movementSet);
                movementSet=sort(movementSet);
                % apply each movement in movementSet to current trajectory
                for k=movementSet
                    newstate=linearModel.predict(obj.state,k);              % Generate new state in Lineraized model
                    [m,n] = size(newstate);                                 
                    pos = newstate(1:3,n);                                  % Extract position from new state after movement execution
                    cell=grid.getCellEuclidian(pos);                        % Try to fetch cell where the predictor will land after movement execution 
                    if cell~=0                                              % Check if cell exist, we do not care about trajectories going out of constrained area ...
                        obj.root.counter = obj.root.counter + 1;            %   Generate new ID
                        newId = obj.root.counter;                           %   Create new predictor Node for after movement application
                        worker = PredictorNode(newstate,newId,k);
                        worker.depth=obj.depth+1;
                        worker.root=obj.root;
                        worker.parrent=obj;
                        worker.referencedCell=cell;
                        worker.leafs=[];
                        worker.hash = [obj.hash,mat2str(cell.ijkIndex)];
                        worker.mycell = mat2str(cell.ijkIndex);
                        worker.reachTime =  newstate(7,n);
                        worker.expanded=0;
                        cell.registerTrajectory(worker);
                        if length(possibleGroups) > 1 ...
                           && worker.movement == MovementType.Straight
                           worker.isACASPreffered=true;
                        end
                        thash= [obj.mycell,worker.mycell];                  % Hash generation 
                        if (worker.depth>2)
                            thash= [obj.parrent.mycell,obj.mycell,worker.mycell];
                            %thash=[obj.hash,worker.mycell];                %This does not work well
                        end
                        worker.hash=thash;                           
                        obj.leafs=[obj.leafs,worker];                       % add to collection
                    end
                end
                r = obj.leafs;                                              % result is a collection of expanded leaflets in trajectory treee
            else
                r = [];
            end
            obj.expanded = 1;
        end
        
        function r=reachFinalLayerTest(obj,finalLayer)
            if (obj.depth == finalLayer)
                obj.reachFinalLayer=1;
            else
                if isempty(obj.leafs)
                    obj.reachFinalLayer=0;
                else
                    results=[];
                    for k=1:length(obj.leafs)
                        results=[results,obj.leafs(k).reachFinalLayerTest(finalLayer)];
                    end
                    obj.reachFinalLayer = sum(results);
                end
            end
            r=[obj.reachFinalLayer];
        end
        
        function r=pruneBasedOnFinalLayer(obj)
            if ~isempty(obj.leafs)
                rating = [];
                for k = 1: length(obj.leafs)
                    rating = [rating, obj.leafs(k).reachFinalLayer];
                end
                removalCandidates= rating==0;
                obj.leafs(removalCandidates)=[];
                if ~isempty(obj.leafs)
                    for k = 1: length(obj.leafs)
                       obj.leafs(k).pruneBasedOnFinalLayer; 
                    end
                end
            end
        end
        
        function r=pruneLastSpread(obj,lastLayer)
            if ~isempty(obj.leafs)
                if (obj.depth == (lastLayer-1))
                    candidate= [];
                    for k = 1:length(obj.leafs)
                        if obj.leafs(k).movement == MovementType.Straight
                           candidate =  obj.leafs(k);
                        end    
                    end
                    obj.leafs = candidate;
                    r=1;
                else
                    r=0;
                    for k = 1:length(obj.leafs)
                       r=r+ obj.leafs(k).pruneLastSpread(lastLayer);
                    end
                end
            end
        end
           
        function r=collectLeafs(obj)
            l=length(obj.leafs);
            if (l==0)
                r=[obj];
            else
                r=[];
                for k=1:l
                    r=[r,obj.leafs(k).collectLeafs];
                end
            end
        end
        
        function r=countNodes(obj,leafsOnly)
            r=0;
            if isempty(obj.leafs)
                r=1;
            else
                for k = 1:length(obj.leafs)
                    r=r+obj.leafs(k).countNodes(leafsOnly);
                end
                if leafsOnly == 0
                    r=r+1;
                end
            end
        end
        
        function r=registerSelf(obj)
            r=0;
            if obj.root ~= obj
                cell=obj.referencedCell;
                %cell.registerTrajectory(obj);
                cell.trajectories = [cell.trajectories,obj];
                r=1;
            end
            if ~isempty(obj.leafs)
                for k=1: length(obj.leafs)
                    r=r+obj.leafs(k).registerSelf();
                end
            end
        end
        
        function calculateCost(obj)            
            obj.calculateCostNode;
            if ~isempty(obj.leafs)
                for k=1: length(obj.leafs)
                    obj.leafs(k).calculateCost();
                end
            end
        end
        
        function calculateCostNode(obj)
            [m,n]=size(obj.state);
            if obj.depth == 0
                obj.movementCost=1;
                obj.trajectoryCost=1;
            else
                if obj.movement == MovementType.Straight || obj.movement == MovementType.Left ||  obj.movement == MovementType.Right
                    obj.movementCost = 1;
                else
                    obj.movementCost = 1.2;
                end
                obj.trajectoryCost = obj.movementCost*obj.parrent.trajectoryCost;
            end
        end
        
        function calculateProbability(obj)
            obj.calculateNodeProbability;
            if ~isempty(obj.leafs)
                for k=1: length(obj.leafs)
                    obj.leafs(k).calculateProbability;
                end
            end
        end
        
        function calculateNodeProbability(obj)
            if obj ~= obj.root
               %calculate movement distribution
               obj.cObstacle = (obj.referencedCell.pObstacle)*obj.rObstacle;
               obj.cVisibility = (obj.referencedCell.pVisibility)*obj.rVisibility;
               obj.cReachibility = ((1-obj.cObstacle)*obj.cVisibility)*obj.rReachibility;
               obj.cFeasibility = (obj.cReachibility * obj.cVisibility)*obj.rFeasibility;
               
               %calculate trajectory distribution
               obj.pObstacle = obj.parrent.pObstacle * obj.cObstacle;
               obj.pVisibility = obj.parrent.pVisibility * obj.cVisibility;
               obj.pReachibility = obj.parrent.pReachibility * obj.cReachibility;
               obj.pFeasibility = obj.parrent.pFeasibility * obj.cFeasibility;   
            end
        end
        
        function r=calculateTrajectoryEntryLeave(obj)
            cell = obj.referencedCell;                                  %Fetch referenced  cell
            worker=obj;                                                 %Reference transition obj == worker
            [m,n]=size(worker.state);                                   %extract position from worker
            wpos=worker.state(1:3,n);                                   
            parent=worker.parrent;                                      % extract position from previously applied  movement
            ppos=parent.state(1:3,n-1);
            vehicleVelocity = wpos-ppos./...
                             (worker.reachTime-parent.reachTime);
            vehicleVelocityNorm = norm(vehicleVelocity);
            startPointNorm = norm(ppos);                                % calculate velocity and local entry leave
            d_enter = cell.dStart - startPointNorm;                     % calculate entry/leave distance from vehicle
            d_leave = cell.dEnd - startPointNorm;                       % 
            t_enter = d_enter/vehicleVelocityNorm +parent.reachTime;    % calculate entry/leave time based on distance velocity ...
            t_leave = d_leave/vehicleVelocityNorm +parent.reachTime;
            obj.cellEntryTime = t_enter;                                % Set cell entry leave time based on calc values
            obj.cellLeaveTime = t_leave;
            r=[t_enter;t_leave];                                        % Return entry/leave time as touple
        end
        
        function resetNodeParameters(obj)
            obj.pFeasibility=1;     %trajectory probability of feasibility 
            obj.pObstacle = 0;      %trajectory probability of obstacle
            obj.pVisibility = 1;    %trajectory probability of visibility
            obj.pReachibility = 1;  %trajectory probability of reachibility
            obj.cFeasibility = 1;   %movement probability of feasibility
            obj.cObstacle = 0;      %movement probability of obstacle
            obj.cVisibility = 1;    %movement probability of visibility
            obj.cReachibility = 1;  %movement probability of reachibility 
            obj.rFeasibility = 1;   %rule ratio of feasibility
            obj.rObstacle = 1;      %rule ratio of obstacle
            obj.rVisibility = 1;    %rule ratio of visibility
            obj.rReachibility = 1;  %rule ratio of reachibility
            obj.flagDistanceFeasibility = true; % By default this is true
            obj.flagHeadingFeasibility = true; %  By default this is true
        end
        
        function r=resetParameters(obj)
            r=1;
            obj.resetNodeParameters;
            if ~isempty(obj.leafs)
                for k=1: length(obj.leafs)
                    r=r+obj.leafs(k).resetParameters;
                end
            end
        end
        
        function r=collectMovements(obj)
            if obj.parrent~=0
                r=[obj.parrent.collectMovements,obj.movement];
            else
                r=[];
            end
        end
        
        function r=checkTrajectoryRange(obj,hCellRange,vCellRange)
            if obj.root == obj
                r = true;
            else
                cell = obj.referencedCell;
                cond = (cell.horizontalIndex >= hCellRange(1)) && ...
                       (cell.horizontalIndex <= hCellRange(2)) && ...
                       (cell.verticalIndex   >= vCellRange(1)) && ...
                       (cell.verticalIndex   <= vCellRange(2));
                r= cond && obj.parrent.checkTrajectoryRange(hCellRange,vCellRange);
            end
            
        end
        function r=selectTrajectories(obj,hCellRange,vCellRange)
            r=[];
            if obj.root == obj
                for k=1:length(obj.leafs)
                    r=[r, obj.leafs(k).selectTrajectories(hCellRange,vCellRange)];
                end
            else
                cell = obj.referencedCell;
                cond = (cell.horizontalIndex >= hCellRange(1)) && ...
                       (cell.horizontalIndex <= hCellRange(2)) && ...
                       (cell.verticalIndex   >= vCellRange(1)) && ...
                       (cell.verticalIndex   <= vCellRange(2));
                if cond
                    if ~isempty(obj.leafs)
                        for k=1:length(obj.leafs)
                        	r=[r, obj.leafs(k).selectTrajectories(hCellRange,vCellRange)];
                        end
                    else
                        r=obj;
                    end
                else
                    if obj.root ~= obj
                        r=[obj.parrent];
                    end
                end
            end
        end
        
        function node=generateGraph(obj,register)
            % determine hash code
            if (obj ~= obj.root)
                h=obj.mycell;
            else
                h='root';
            end
            
            if ~register.isKey(h)
                regid=register.Count+1;
                node = GraphNode(regid,obj.referencedCell,obj.referencedCell.center);
                register(h)=node;
            end
            node = register(h);
            for k=1:length(obj.leafs)
                leaf=obj.leafs(k);
                leafNode=leaf.generateGraph(register);
                node.registerLink(leafNode);
            end
        end
        
        function register=getTrajectoryRegister(obj,register)
            if (nargin==1)
                register=containers.Map;
            end
            if(~register.isKey(obj.hash))
                register(obj.hash)=[obj];
            else
                list = register(obj.hash);
                list = [list,obj];
                register(obj.hash)=list;
            end
            for leaf=obj.leafs
                leaf.getTrajectoryRegister(register);
            end
        end
     
        function plotTrajectoryStat(obj, mode)
            if obj ~= obj.root
                if mode == StatisticType.Reachability
                    pre=obj.pReachibility; 
                    %Hotfix of saturated rating
                    if pre >1
                        pre=1;
                    end
                    if pre <0
                        pre=0;
                    end
                    col = [1-pre,pre,0];
                end
                if mode == StatisticType.Visibility
                    pre=obj.pVisibility;
                    %Hotfix of saturated rating
                    if pre >1
                        pre=1;
                    end
                    col = [0,pre,1-pre];
                end
                if mode == StatisticType.Obstacle
                    pre=obj.cObstacle;
                    %Hotfix of saturated rating
                    if pre >1
                        pre=1;
                    end
                    col = [pre,1-pre,0];
                end
                if mode == StatisticType.Feasibility
                    pre=obj.pFeasibility;
                    %Hotfix of saturated rating
                    if pre >1
                        pre=1;
                    end
                    col = [1-pre,0,pre];
                end
                sta=obj.state;
                [m,n] = size(sta);
                cc= sta(1:3,(n-1):n);
                if (mode ~= StatisticType.TrimmedReach)
                    plot3(cc(1,:),cc(2,:),cc(3,:),'Color',col);
                else
                    if obj.pReachibility == 1
                        plot3(cc(1,:),cc(2,:),cc(3,:),'Color','g','Linewidth',2);
                    end
                end
                obj.parrent.plotTrajectoryStat(mode);
            end
        end
        function plotTrajectoryWide(obj, color)
            if nargin ==1
                color = 'c';
            end
            if obj ~= obj.root
                sta=obj.state;
                [m,n] = size(sta);
                cc= sta(1:3,(n-1):n);
                plot3(cc(1,:),cc(2,:),cc(3,:),'Linewidth',4,'Color',color);
                obj.parrent.plotTrajectoryWide(color);
            end
        end
        
        function r=getLocalPositionOrientation(obj)
            [m,n]=size(obj.state);
            r=obj.state(1:6,n);
        end
        %% Rule engine
        % side feasibility node
        function [flagDistanceFeasibility,flagHeadingFeasibility]= ...
                calculateOperatibleSpaceNode(obj,missionControl,linearVelocity,...
                vehiclePosition,vehicleOrientation,collisionPoint,...
                ruleSafetyMargin)
            % Grab node local position orientation
            nodeLocPosOr=obj.getLocalPositionOrientation;
            % 1st triplet in state is position (local)
            nodeLocPos=nodeLocPosOr(1:3);
            % 2nd triplet in state is velocity (local)
            nodeLocOr=nodeLocPosOr(4:6);
            % Global just rotate the local position of node to vehicle orientation
            nodeGlobPos=missionControl.getGlobalCoordinates(nodeLocPos);
            % Well velocity is not that easy
            % X - right hand rotation base 
            nodeLocVelocity=[linearVelocity;0;0];
            % Apply vehicle rotation
            vehicleVel=Cmnf.rot3D(vehicleOrientation(1),vehicleOrientation(2),vehicleOrientation(3),nodeLocVelocity);
            % Apply after movement rotation to get true heading
            trajectoryVel=Cmnf.rot3D(nodeLocOr(1),nodeLocOr(2),nodeLocOr(3),vehicleVel);
            % Add global position to get second point of vector
            nodeGlobVelocity=nodeGlobPos+trajectoryVel;

            % comparable safety margin
            comSafetyMargin = norm(nodeGlobPos-collisionPoint);


            % compare heading plane
            compCol=collisionPoint - nodeGlobPos;
            compVel=trajectoryVel; %=nodeGlobVelocity-nodeGlobPos;

            % Normalize to vehicle heading,
            compColNorm=Cmnf.rot3D(-nodeLocOr(1),-nodeLocOr(2),-nodeLocOr(3),compCol);
            compColLoc=Cmnf.rot3D(-vehicleOrientation(1),-vehicleOrientation(2),-vehicleOrientation(3),compColNorm);
            % Create comparison
            %   [x;y;z] y+ and y0 good for us,

            flagDistanceFeasibility = comSafetyMargin >= ruleSafetyMargin;
            flagHeadingFeasibility = compColLoc(2) >=0;
            obj.flagDistanceFeasibility=flagDistanceFeasibility;
            obj.flagHeadingFeasibility=flagHeadingFeasibility;
        end
        % planarFeasibility
        function r=calculateOperatibleSpace(obj,missionControl,linearVelocity,...
                vehiclePosition,vehicleOrientation,collisionPoint,...
                ruleSafetyMargin)
            r=0;
            [flagDistanceFeasibility,flagHeadingFeasibility]= ...
                obj.calculateOperatibleSpaceNode(missionControl,linearVelocity,...
                vehiclePosition,vehicleOrientation,collisionPoint,...
                ruleSafetyMargin);
            for leaf =obj.leafs
                r= r + leaf.calculateOperatibleSpace(missionControl,linearVelocity,...
                vehiclePosition,vehicleOrientation,collisionPoint,...
                ruleSafetyMargin);
            end
            if isempty(obj.leafs)
                r=1;
            end 
        end
        
        
        function r=applyOperatibleSpaceNode(obj)
            if ~(obj.flagDistanceFeasibility && obj.flagHeadingFeasibility)
                obj.rFeasibility = 0;   % Impact on feasibility of trajectory ...
                obj.rObstacle = 1;      % No impact on obstacle
                obj.rVisibility = 1;    % No impact on visibility
                obj.rReachibility = 0;  % Impact on reachibility of trajectory from this point 
                r=1;
            else
                r=0;
            end
        end
        
        function r=applyOperatibleSpace(obj)
            r=applyOperatibleSpaceNode(obj);
            for leaf = obj.leafs
                r=r+applyOperatibleSpace(leaf);
            end
        end
        
        %% Hard constraints
        function r=applyHardConstraint(obj,mc,cc)
            r=0;
            posor=obj.getLocalPositionOrientation();
            if cc.isIntersection(posor,mc,obj)
                obj.flagDistanceFeasibility = false;
            end
            for lf=obj.leafs
                r=r+lf.applyHardConstraint(mc,cc);
            end
        end
    end
    
end

