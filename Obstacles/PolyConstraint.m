classdef PolyConstraint<BarrelConstraint
    % PolyConstraint Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        polygon =[]    %polynomial
    end
    
    methods
        function obj = PolyConstraint(polygon, sh,eh,psh,peh)
            % 2D polygon with dimensions N:2 is expected 
            if length(polygon)~= 2
                wpol = polygon';
            else
                wpol = polygon;
            end
                
            %calculate center
            [radius,center]=Cmnf.waltzCircleFit(polygon');
            if nargin < 2
                sh = -5;
            end
            if nargin < 3
                eh = 5;
            end
            if nargin < 4
                psh = -5;
            end
            if nargin < 5
                peh = 5;
            end
            obj=obj@BarrelConstraint(center,radius, sh,eh,psh,peh);
            obj.polygon=polygon;
        end
        
        function r=applyMovement(obj,mc)
            offset=obj.velocityVector;
            xyof=offset(1:2);
            % Move polygon
            obj.polygon = obj.polygon+xyof;
            
            %change heigh boundaries
            r=applyMovement@AbstractConstraint(obj,mc);
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
        
        function r=getLogString(obj)
            r=[', polynome: ',mat2str(obj.polygon),'radius: ',mat2str(obj.radius),', start height: ',mat2str(obj.startHeight),', end height: ',mat2str(obj.endHeight)];
        end
    end
end

