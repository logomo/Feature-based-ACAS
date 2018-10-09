classdef GraphNode < LoggableObject
    %GRAPHNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id;
        hash;
        cell;
        position;
        links;
        ref;
    end
    
    methods
        function obj=GraphNode(id,cell,position)
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
            if ~obj.links.isKey(node.hash)
                obj.links(node.hash)=node;
                obj.ref=[obj.ref,node];
                r=true;
            else
                r=false;
            end
        end
        function r=plot(obj)
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
            if nargin==1
                
                statId =  StatisticType.Reachability;
            end
            visited = containers.Map;
            obj.plotColoredGraphRecursion(statId,visited);
        end
        
        function plotColoredGraphRecursion(obj,statId,visited)
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

