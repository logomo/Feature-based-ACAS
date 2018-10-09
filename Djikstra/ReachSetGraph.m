classdef ReachSetGraph<LoggableObject
    %REACHSETGRAPH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        avoidanceGrid
        root            %entry point (it should be tree so root ?)
        register
        nodeList
        trajectories
    end
    
    methods
        function obj=ReachSetGraph(ag)
            root = ag.reachSet;
            register=containers.Map;
            graphRoot=GraphNode(1,0,[0;0;0]);
            register('root')=graphRoot;
            root.generateGraph(register);
            % generate ordered set of nodes;
            nodeList(register.Count)=GraphNode();
            val=(values(register));
            for k=1:register.Count
                node=val{k};
                nodeList(node.id)=node;
            end
            %data setup
            obj.avoidanceGrid=ag;
            obj.root=graphRoot;
            obj.register=register;
            obj.nodeList=nodeList;
        end
        
        function putObstacle(obj,os)
            [m,n]=size(os.points);
            for k=1:n
                cell = obj.avoidanceGrid.getCellEuclidian(os.points(1:3,k));
                if cell ~= 0
                    obj.avoidanceGrid.putObstacle(cell,os,1);
                end
            end
            obj.avoidanceGrid.recalculate;
        end
        
        function [costs,paths]=findPaths(obj,statId)
            % Adjacent matrix if I->J then A(I,J)=1
            nodeCount=obj.register.Count;
            A=zeros(nodeCount);
            for node =obj.nodeList
                i=node.id;
                for connection = node.ref
                    j = connection.id;
                    A(i,j)=1;
                end
            end

            % Cost matrix reflect good/bad cost
            if nargin == 1
                statId=StatisticType.Reachability;
            end
            C=zeros(nodeCount);
            for node =obj.nodeList
                i=node.id;
                for connection = node.ref
                    j = connection.id;
                    base=1;
                    cell=connection.cell;
                    if statId == StatisticType.Reachability
                        base=cell.pReachability; % 1 Good
                    end
                    if statId == StatisticType.Visibility
                        base=cell.pVisibility; % 1 good;
                    end
                    if statId == StatisticType.Obstacle || statId == StatisticType.GraphObstacle
                        base=1-cell.pObstacle; % must be reversed 0- good
                    end
                    C(i,j)=1/base; % cost increases with decreasing base !!
                end
            end

            %Source nodes == 1 => ROOT  NODE
            SID = 1;

            %Final Nodes == all nodes ending on final layer of avoidance grid
            FID = [];
            for node =obj.nodeList
                cell = node.cell;
                if cell~=0 && obj.avoidanceGrid.dEnd == node.cell.dEnd
                    FID=[FID,node.id];
                end
            end

            [costs,paths] = dijkstra(A,C,SID,FID,1);
        end
        
        function trajectories=generateTrajectories(obj, costs,paths)
            trajectories=[];
            for k=1:length(costs)
                trajectories=[trajectories,DijkstraTrajectory(k,costs(k),paths{k},obj.nodeList)];
            end
            obj.trajectories=trajectories;
        end
        
        function [unfeasible,feasible,feasibilityMap]=assessExistance(obj,trajectoryRegister)
            feasible = [];
            unfeasible = [];
            feasibilityMap=containers.Map('KeyType','double','ValueType','any');
            for dijTraj=obj.trajectories
                key=dijTraj.footprint;
                if(trajectoryRegister.isKey(key))
                    candidates = trajectoryRegister(key);
                    feasibilityMap(dijTraj.id)=candidates;
                    feasible = [feasible,dijTraj.id];
                else
                    dijTraj.id;
                    unfeasible = [unfeasible,dijTraj.id];
                end
            end
        end
        
        function plot(obj,statId)
            if nargin==1
                statId=StatisticType.Reachability;
            end
            if statId == StatisticType.Obstacle
                statId = StatisticType.GraphObstacle;
            end
            obj.root.plotColoredGraph(statId);
            if statId == StatisticType.Obstacle || statId == StatisticType.GraphObstacle
                title('Obstacle rating (blue-red)');
            end
            if statId == StatisticType.Reachability
               title('Reachibility rating (green-red)');
            end
            if statId == StatisticType.Visibility
               title('Visibility rating (blue-black)');
            end
            grid on;
            xlabel('x [m]');
            ylabel('y [m]');
            zlabel('z [m]');
        end
    end
    
end

