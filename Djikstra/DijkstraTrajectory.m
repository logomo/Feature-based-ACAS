classdef DijkstraTrajectory<LoggableObject
    %DIJKSTRATRAJECTORY Helper object for Djikstra trajectory search
    %   The object nodes are parsed to create simple matrix, 
    %       -start node - there is root node (1) where we always start search of least costly trajectory (best reachibility)
    %       -final node - there is at least one goal node in cell where we want to get
    
    properties
        id          %Unique identifier
        cost        %Overall trajectory cost to flew it
        points      %List of points XYZ LCF along trajectories
        cells       %List of passing cells along the trajectory in start-end order
        graphNodes  %List of composite nodes in start-end order
        goalCell    %The goal cell where trajectory ends
        goalPoint   %The last point XYZ LCF where trajectory lands
        footprint   %The string representing trajectory hash footprint (coverage)
    end
    
    methods
        function obj=DijkstraTrajectory(id,cost,path,nodeList)
            %Contructor, creating data envelope for one Graph node represented trajectory 
            %   id - the unique trajectory ID          
            %   cost -overall trajectory cost       
            %   path -list of passing cells       
            %   nodeList - list of passing nodes    
            Cmnf.logc(obj,['Creating Dijkstra trajectory ',mat2str(id),' with associated cost ',mat2str(cost)]);
            obj.id=id;
            obj.cost=cost;
            obj.points=[];
            obj.cells=[];
            obj.graphNodes=[];
            for k=1:length(path)
                nodeId=path(k);
                node=nodeList(nodeId);
                if (node.id ~= nodeId)
                    Cmnf.logc(obj,['[Error] Disparity in mapping selected node ', mat2str(nodeId) , 'do not match']);
                    nodeId
                    node.id
                    exit(-1);
                else
                    if nodeId~=1 %check root 
                        obj.cells=[obj.cells,node.cell];
                    end
                    obj.points=[obj.points,node.position];
                    obj.graphNodes=[obj.graphNodes,node];
                end
            end
            obj.goalCell=obj.cells(length(obj.cells));
            obj.goalPoint=obj.points(:,length(obj.points));
            obj.footprint='';
            for k=1:length(obj.cells)
                obj.footprint=[obj.footprint,mat2str(obj.cells(k).ijkIndex)];
            end
            Cmnf.logc(obj,['   - footprint:',obj.footprint]);
        end
        
        function plot(obj,col)
            %Plots the trajectory in LCF avoidance grid space
            %   col - color of lines connecting nodes, default - magenta
            if nargin==1
                col = 'm';
            end
            plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),'Linewidth',4,'Color',col);
        end
    end
end

