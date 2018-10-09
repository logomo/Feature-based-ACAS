classdef DijkstraTrajectory<LoggableObject
    %DIJKSTRATRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        cost
        points
        cells
        graphNodes
        goalCell
        goalPoint
        footprint
    end
    
    methods
        function obj=DijkstraTrajectory(id,cost,path,nodeList)
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
            if nargin==1
                col = 'm';
            end
            plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),'Linewidth',4,'Color',col);
        end
    end
end

