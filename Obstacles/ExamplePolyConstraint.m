classdef ExamplePolyConstraint<PolyConstraint
    % PolyConstraint Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        polygonType
        scale
        rotation
        forcedColorFlag=true; % Maybe change in future or use base class PolyConstraints
        forcedColor='c';
    end
    
    methods
        function obj = ExamplePolyConstraint(type,center,scale,rotation, sh,eh,psh,peh)
            if nargin < 3
                scale=1;
            end
            
            if nargin < 4
                rotation = 0;
            end
            
            if nargin < 5
                sh = -5;
            end
            
            if nargin < 6
                eh = 5;
            end
            
            if nargin < 7
                psh = -5;
            end
            
            if nargin < 8
                peh = 5;
            end
            
            base=ExamplePolygonType.getPolygonData(type);
            scaled = base.*scale;
            rotated = Cmnf.rot2D(rotation,scaled);
            shifted = rotated + center(1:2);
            
            obj=obj@PolyConstraint(shifted, sh,eh,psh,peh);
            obj.polygonType=type;
            obj.scale=scale;
            obj.rotation=rotation;
        end
        
        function handles=plot(obj) 
            cl=obj.getColor;
            % prepare working polygon
            [m,~] = size(obj.polygon);
            if m ~= 2
                wpol= obj.polygon';
            else
                wpol = obj.polygon;
            end
            [~,n] = size(wpol);
            lower = [wpol;ones(1,n)*obj.plotStartHeight];
            upper = [wpol;ones(1,n)*obj.plotEndHeight];
            P=[lower,upper]';
            %k=boundary(P,1);

            
            hold on
            k=[1:n-1;n+1:2*n-1;n+2:2*n]';
            l=[1:n-1;2:n;n+2:2*n]';
            h1=trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor',cl,'FaceAlpha',1,'EdgeColor','k');
            h2=trisurf(l,P(:,1),P(:,2),P(:,3),'Facecolor',cl,'FaceAlpha',1,'EdgeColor','k');
            h3=fill3(upper(1,:),upper(2,:),upper(3,:),cl);
            h4=fill3(lower(1,:),lower(2,:),lower(3,:),cl);
            hold off
            handles=[h1,h2,h3,h4];
        end
        
        function r=getColor(obj)
           if obj.forcedColorFlag
               r=obj.forcedColor;
           else
               r=getColor@PolyConstraint(obj);
           end
        end
        
        function r=forceColor(obj,col)
            obj.forcedColor=col;
            obj.forcedColorFlag=true;
            r=obj.forcedColor;
        end
        
        function r=getLogString(obj)
            r=[', polynome: ',mat2str(obj.polygon),'radius: ',mat2str(obj.radius),', start height: ',mat2str(obj.startHeight),', end height: ',mat2str(obj.endHeight)];
        end
    end
end

