classdef GraphNode < LoggableObject
    %GRAPHNODE Represents one reach set node (See AvoidanceGrid.PredictorNode)
    
    
    properties
        id;             %Unique identifier of the node 
        hash;           %String key to identify (cell hash)
        cell;           %Parrent cell reference
        position;       %XYZ euclidean LCF position (see Avoidance Grid)
        links;          %List of directed connections from other nodes(in)
        ref;            %List of directed connections to other nodes (out)
    end
    
    methods
        function obj=GraphNode(id,cell,position)
            % Contructor creating node based on uq id, referenced cell and position
            %   id - uq key
            %   cell - referenced cell where node lies in LCF
            %   position - node XYZ LCF position
            if nargin ~= 0 
                obj.id=id;
                if cell == 0 
                   obj.hash = 'root'; 
                else
                   obj.hash = mat2str(cell.ijkIndex);
                end
                obj.cell=cell;
                obj.position=position;
                obj.links=containers.Map;
                obj.ref=[];
            end
        end
        
        function r=registerLink(obj,node)
            % Registers link to the node checking references to disable circularity
            %   node - next node 
            if ~obj.links.isKey(node.hash)
                obj.links(node.hash)=node;
                obj.ref=[obj.ref,node];
                r=true;
            else
                r=false;
            end
        end
        
        function r=plot(obj)
            %Plots the node in 3D enviroment
            hold on
            %plot node point
            pos=obj.position;
            plot3(pos(1),pos(2),pos(3),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r');
            t=[mat2str(obj.id)];
            %text(pos(1),pos(2),pos(3),t);
            %plot links
            for k=1:length(obj.ref)
                wRef=obj.ref(k);
                rPos=wRef.position;
                plot3([pos(1),rPos(1)],[pos(2),rPos(2)],[pos(3),rPos(3)]);
            end
            
            %plot followers
            for k=1:length(obj.ref)
                wRef=obj.ref(k);
                wRef.plot;
            end
            hold off
        end
        %robust plot function (if there are cycles in reach set)
        function plot2(obj,visited)
            %Plots the node in 3D enviroment,robust plot function (if there are cycles in reach set)
            if nargin==1
                visited = containers.Map;
            end
            if ~(visited.isKey(obj.hash))
                obj.hash;
                visited(obj.hash)=1;
                hold on
                %plot node point
                pos=obj.position;
                plot3(pos(1),pos(2),pos(3),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r');
                t=[mat2str(obj.id)];
                %text(pos(1),pos(2),pos(3),t);
                %plot links
                for k=1:length(obj.ref)
                    wRef=obj.ref(k);
                    rPos=wRef.position;
                    plot3([pos(1),rPos(1)],[pos(2),rPos(2)],[pos(3),rPos(3)]);
                end

                %plot followers
                for k=1:length(obj.ref)
                    wRef=obj.ref(k);
                    wRef.plot2(visited);
                end
                hold off
            end
        end
        
        function plotColoredGraph(obj,statId)
            %Recursive function to plot statistics in colored node graph, 
            %   statId - the enumeration Statistic
            if nargin==1
                
                statId =  StatisticType.Reachability;
            end
            visited = containers.Map;
            obj.plotColoredGraphRecursion(statId,visited);
        end
        
        function plotColoredGraphRecursion(obj,statId,visited)
            %[HELPER] the recursive call of plotColoredGraph function
            if ~(visited.isKey(obj.hash))
                obj.hash;
                visited(obj.hash)=1;
                hold on
                %plot node point
                pos=obj.position;
                plot3(pos(1),pos(2),pos(3),'Marker','square','MarkerFaceColor','g','MarkerEdgeColor','r');
                t=[mat2str(obj.id)];
                %text(pos(1),pos(2),pos(3),t);
                %plot links
                for k=1:length(obj.ref)
                    wRef=obj.ref(k);
                    rPos=wRef.position;
                    pd=wRef.cell.getPlotData;
                    cool=pd.getcolorVectorRGB(statId);
                    plot3([pos(1),rPos(1)],[pos(2),rPos(2)],[pos(3),rPos(3)],'Color',cool);
                end

                %plot followers
                for k=1:length(obj.ref)
                    wRef=obj.ref(k);
                    wRef.plotColoredGraphRecursion(statId,visited)
                end
                hold off
            end
        end
    end
    
end

