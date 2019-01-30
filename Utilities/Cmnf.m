classdef Cmnf
    %Cmnf Common functionality class
    %   Covers all from configs to the stuff
    
    properties(Constant)
        %% Adversary intersection constants
        precision = 0.2;                % minimal intersection precision in meters
        obstaclePrecision = 0.5;        % maximal point cloud dispersion in meters
        steps = 3;                      % linear backtracking intersection model minimum
        eliSteps = 5;                   % elipsioidal step minimum
        adversaryCounter  = 0;          % default adversary counter
        
        %% Debug constants
        debugTree=0;                   % show possible tree when makind decision
        debugAdversary=0;              % show adversary intersection process in verbose and graphic
        enabledTrace=false;            % trace functionality on loggable objects - turn off for demonstration
        enabledForcedTrace=false;      % second level trace functionality on loggable objects - turn off for demonstration
        enableRolling=true;            % enable rolling in console -- turn on for debugging
        enableNonlinearVehicle=false;  % enable nonlinear vehicle for more precise trajectories
        enableTracePlot=false;         % FALSE by default, TRUE for performance test - plane with no trace (false) or lines with decision points (true)    
        enableTrajectoryTracePlot=true; % show possible trace plot
        enableFigureExports=false;     % this should be by default disabled
        pngExportPath='c:\\Sukromne\\'; % export path for figure to png
        
        
        %% Plot constants
        plotYellowDistance=5;           % UAS conservative performance[m]
        plotRedDistande=1.5;            % UAS adaptive performance[m]
        plotCrashDistande=0.6;          % UAS crash distance [m]

        %% Simulation setup
        vehicleSpeed=1;             % Vehicle default speed 1m/s
        simStep=1;                  % simulation step 1(s)
        reachibilityTreshold=0.98;  % stability treshold 98% is standard for TCAS/ACAS
        acasSplitRatio = 4;         % Bulgarian constant
    end
    
    methods(Static)
        function r=euc2plan(x,y,z)
            %3D euclidian to planar transformation
            d = (x^2 + y^2 + z^2)^(1/2);
            dxy= (x^2 + y^2)^(1/2);
            theta = atan2(y,x);
            psi = atan2(z,dxy);
            r = [d;theta;psi];
        end
        function r=euc2planV(v)
            %3D planar to euclidian transformation
            r=Cmnf.euc2plan(v(1),v(2),v(3));
        end
        function r= plan2euc(d,theta,phi)
            %3D planar to euclidean parameters only
            dxy = cos(phi)*d;
            z = sin(phi)*d;
            y = sin(theta)*dxy;
            x = cos(theta)*dxy;
            r=[x;y;z];
        end
        
        function r=rot2D(theta,mat)
            %2D rotation matrix
            %   theta - rotation angle 
            %   matrix in row format
            ROT = [cos(theta), -sin(theta);
                   sin(theta), cos(theta)];
            r=ROT*mat;
        end
        
        function r=rot3D(alpha,beta,gamma,mat)
            %Rodriguez rotation matrix
            ROT = [ cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta);
                    cos(beta)*sin(gamma), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha);
                    -sin(beta),                                    cos(beta)*sin(alpha),                                    cos(alpha)*cos(beta)];
            r= ROT*mat;    
        end
        
        function ROT=rot3DMatrix(alpha,beta,gamma)
            %Rodriguez rotation matrix for given angles
            ROT = [ cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta);
                    cos(beta)*sin(gamma), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha);
                    -sin(beta),                                    cos(beta)*sin(alpha),                                    cos(alpha)*cos(beta)];
             
        end
        
        function r=rot3Dvec(mat,vec)
            %Rodriguez rotation matrix application on matrix
            %   mat - 3 row matrix
            %   vec - alpha/beta/gamma
            r=Cmnf.rot3D(vec(1),vec(2),vec(3),mat);
        end
        
        function r=align3Dvec(vec,mat)
            %Align the vector on the same directive
            %   mat - 3 row matrix
            %   vec - the direction vector
            pla =Cmnf.euc2plan(vec(1),vec(2),vec(3));
            %alpha=atan2(z,y);
            %beta=-atan2(z,x); = -\varhphi
            %gamma=atan2(y,z); = \theata
            r=Cmnf.rot3D(0,-pla(3),pla(2),mat);
        end
        function r=ofst(vec,mat)
            %Offset matrix to 3D point
            %   vec - Offset point
            %   mat - 3D point line matrix
            [m,n]=size(mat);
            r = mat + vec*ones(1,n);
        end
        
        function r=getMovement(mt)
            %Get movement based on Movement type
            %   mt - MovementType enumeration member
            velocity = 1;
            time = 1;
            omega_alpha=0;
            omega_beta=0;
            omega_gamma=0;
            %dole
            if(mt == MovementType.Down)
               omega_beta = pi/12; 
            end
            %hore
            if(mt == MovementType.Up)
               omega_beta = -pi/12; 
            end
            %dolava
            if(mt == MovementType.Left)
               omega_gamma = pi/12; 
            end
            %doprava
            if(mt == MovementType.Right)
               omega_gamma = -pi/12; 
            end
            
            if(mt == MovementType.DownLeft)
                omega_beta = pi/12;
                omega_gamma = pi/12;
            end
            if(mt == MovementType.DownRight)
                omega_beta = pi/12;
                omega_gamma = -pi/12;
            end
            if(mt == MovementType.UpLeft)
                omega_beta = -pi/12;
                omega_gamma = pi/12;
            end
            if(mt == MovementType.UpRight)
                omega_beta = -pi/12;
                omega_gamma = -pi/12;
            end
            r=[omega_alpha;omega_beta;omega_gamma;velocity;time];
        end
        
        function r=getCubicOffsets(vec)
            %Calculates cubbic offsets for vector
            cnt=1;
            ofsts=zeros(3,27);
            for k = [-1,0,1]
                for l = [-1,0,1]
                    for m = [-1,0,1]
                        ofsts(:,cnt)=[k;l;m];
                        cnt=cnt+1;
                    end
                end
            end
            r= ofsts + vec*ones(1,27);
        end
        
        function r=filterUnusedCells(map,list)
            %Filter unused cells
            cnt =  length(list);
            r = [];
            for k=1:cnt
                key=mat2str(list(k).ijkIndex);
                if ~map.isKey(key)
                    map(key)=1;
                    r=[r,list(k)];
                end
            end
        end
        
        function r=isInRange(point,cell,range)
            %Check if point is in range of the cell
            %   point - point to check
            %   cell - base cell
            %   range - layer/horizontal/vertical range
            distance=norm(cell.center-point,2);
            distanceWithDeviation=distance-cell.toleratedDeviation;
            flagD= distance<=range;
            flagDWD = distanceWithDeviation<=range;
            if flagD || flagDWD
                r= DistanceGridCell(cell,range,distance,flagD,flagDWD);
            else
                r=0;
            end
        end
         
        function r=findCellsInRange(ag,initialCell,point,range)
            %Find cells in distance range of the body around the initial intersection cell
            %   ag - active grid
            %   initialCell - initial intersection cell
            %   point - reference point
            %   range - the range around in [m]
            results = [DistanceGridCell(initialCell,range,0,1,1)];
            register = containers.Map;
            register(mat2str(initialCell.ijkIndex))=1;
            stack = [initialCell];
            while ~isempty(stack)
                worker = stack(1);
                stack(1) = [];
                list=ag.getCellsListedSanity(Cmnf.getCubicOffsets(worker.ijkIndex));
                list=Cmnf.filterUnusedCells(register,list);
                for k=1:length(list)
                    test=Cmnf.isInRange(point,list(k),range);
                    if isa(test,'DistanceGridCell')
                        stack=[stack,list(k)];
                        results = [results, test];
                    end
                end
            end
            r=results;
        end
        
        function probCells=intersectPointCloud(ag,candidates)
            %[Deprecated] probabilistic distribution has been replaced
            probCells=[];
            register=containers.Map;
            [m,n]=size(candidates);
            for k=1:n
                worker=candidates(:,k);
                wCell = ag.getCellEuclidian(worker(1:3));
                if wCell~=0
                    key=mat2str(wCell.ijkIndex);
                    if ~register.isKey(key)
                        value=IntersectionGridCell(wCell,worker(1:3),worker(5));
                        value.probability=worker(4);
                        register(key)=value;
                        probCells=[probCells,value];
                    else
                        value=register(key);
                        value.position=[value.position,worker(1:3)];
                        value.probability=[value.probability,worker(4)];
                        value.time=[value.time,worker(5)];
                    end
                end
            end
        end
        
        
        function r=getAdversaryVector(position,probability,timeofArrival)
            %Creates adversarial column vector adv(5)
            %   x=1 x-position of adversarial
            %   y=2 y-position of adversarial
            %   z=3 z-position of adversarial
            %   p=4 probability of adversarial arrival
            %   t=4 time of adversarial arrival
            r=[position;probability;timeofArrival];
        end
        
                    
        function r=harmonicDistribution(fmin,fmax,val)
            % Harmonic distribution calculation using tangensial sigmoid function
            %   input \in <-1,1>
            %               1
            %   f(i) = 2 - --- e^(-10*i)
            %               2
            range = fmax-fmin;
            center = val-fmin -1/2*range;
            input = center/(1/2*range);
            r=2/(1+exp(-10*input))/2;
        end
        
        function r=log(msg)
            % Log a message to console
            r=[datestr(datetime('now'),'YYYY-MM-DD '),datestr(datetime('now'),'HH:MM:SS.FFF'),' ', msg];
            if Cmnf.enableRolling
                disp(r);
            end
        end
        
        function r=logc(obj,msg)
            %Log message to console and use logable object ledger to keep action track
            nMsg=['[',class(obj),']',' ',msg];
            lc=Cmnf.log(nMsg);
            if Cmnf.enabledTrace && (isa(obj,'LoggableObject')) 
                obj.traceLog=[obj.traceLog,lc,'\n'];
            end
        end
        
        function r=logcft(obj,msg)
            %Log message to console and use logable object ledger to keep action track
            nMsg=['[',class(obj),']',' ',msg];
            lc=Cmnf.log(nMsg);
            if Cmnf.enabledForcedTrace&&(isa(obj,'LoggableObject')) 
                obj.traceLog=[obj.traceLog,lc,'\n'];
            end
        end
        
        function r=glob2loc(pos,rot,mat)
            %GCF->LCF transformation
            %   pos - UAS position GCF
            %   rot - UAS rotation GCF
            %   mat - The point set to be transformed
            [m,n]=size(mat);
            pLoc=mat-pos*ones(1,n);
            ROT = Cmnf.rot3DMatrix(rot(1),rot(2),rot(3));
            r=inv(ROT)*pLoc;
        end
        
        function r=loc2glob(pos,rot,mat)
            %LCF->GCF transformation
            %   pos - UAS position GCF
            %   rot - UAS rotation GCF
            %   mat - The point set to be transformed
            [m,n]=size(mat);
            pGlob=Cmnf.rot3D(rot(1),rot(2),rot(3),mat);
            r=pGlob+pos*ones(1,n);
        end
        
        function r=calculatePointDistance(a,b)
            %Calculate distance of two 2D/3D points
            r=norm(a-b,2);
        end
        
        function r=movementBuffer2String(mb)
            %[Helper]Get movement buffer as string representation 
            r='[';
            mbl=length(mb);
            for k=1:mbl
                r=[r,char(mb(k))];
                if k~=mbl
                    r=[r,', '];
                end
            end
            r=[r,']'];
        end
        
        function r=bolleanString(b)
            %[Helper]Transform boolen to string
            if b
                r='true';
            else
                r='false';
            end
        end
        
        function [collisionFlag,mdest,time,mdist]=calculateLinearClash(p1,v1,p2,v2)
            %Calculates linear clash
            %   Common function based on proceedings from: http://sections.maa.org/lams/proceedings/spring2001/bard.himel.pdf
            %   p1 - position vector for UAS1
            %   p2 - position vector for UAS2
            %   v1 - velocity LCF for UAS1
            %   v2 - velocity LCF for UAS2
            
            
            % calculate coeficients
            A=norm(v1)^2;
            B=2*(v1'*p1-v1'*p2);
            C=2*v1'*v2;
            D=2*(v2'*p2 - v2'*p1);
            E=norm(v2)^2;
            F=norm(p1)^2+norm(p2)^2;
            a=A-C+E;
            b=B+D;
            c=F;
            %times=roots([a,b,c]); olny works in case of direct intersection
            %minima as derivation of (A-C+E)t^2+(B+D)t+F
            time=-(b)/(2*a);

            collisionFlag= time > 0;
            dest1=p1+v1*time;
            dest2=p2+v2*time;
            mdest=(dest1+dest2)./2;
            mdist= (norm(dest1-dest2))^(1/2);
        end
        
        function theta=minimalAnglesBetweenVectors(a, b)
            %calculates minimal planar angle between two vectors on main planes
            
            %a=a(1:2);b=b(1:2);
            theta = atan2(norm(cross(a,b)),dot(a,b));
            theta = rad2deg(abs(theta));
        end
        
        function a=minimalAnglesBetweenVectorsOld(v1, v2)
            %[DEPRECATED] Calculates minimal planar angle between two vectors on main planes
            %   Origin of both vectors is at 0
            %   Magnitude of vector is its norm
            m(1) = norm(v1);
            m(2) = norm(v2); 

            % Min. angles from vector 1 to axes
            a(1 : 3) = acosd(v1 / m(1));
            % Min. angles from vector 2 to axes
            a(4 : 6) = acosd(v2 / m(2)); 

            % Dot product dot(v1, v2) equals sum(v1 .* v2)
            a(7) = acosd(dot(v1, v2) / m(1) / m(2)); 

            %disp(' ')
            %disp(['Angle between vectors: ' num2str(a(7))]) 

            %disp(' ')
            %disp('Vector 1')
            %disp(['Magnitude :' num2str(m(1))])
            %disp(['Min. angle with x-axis: ' num2str(a(1))])
            %disp(['Min. angle with y-axis: ' num2str(a(2))])
            %disp(['Min. angle with z-axis: ' num2str(a(3))]) 

            %disp(' ')
            %disp('Vector 2')
            %disp(['Magnitude :' num2str(m(2))])
        end
        
        function r=copyMap(oldMap)
            %[Helper] copy map content to new intance
            r=containers.Map;
            keys=oldMap.keys;
            for k=1:length(keys)
                key=keys{k};
                value=oldMap(key);
                r(key)=value;
            end
        end
        
        function [R,C,Xb]=waltzCircleFit(X)
        % Compute exact minimum bounding circle of a 2D point cloud using Welzl's 
        % algorithm. 
        %
        %   - X     : M-by-2 list of point co-ordinates, where M is the total
        %             number of points.
        %   - R     : radius of the circle.
        %   - C     : 1-by-2 vector specifying the centroid of the circle.
        %   - Xb    : subset of X, listing K-by-2 list of point coordinates from 
        %             which R and C were computed. See function titled 
        %             'FitCirc2Points' for more info.
        %
        % REREFERENCES:
        % [1] Welzl, E. (1991), 'Smallest enclosing disks (balls and ellipsoids)',
        %     Lecture Notes in Computer Science, Vol. 555, pp. 359-370
        %
        % AUTHOR: Anton Semechko (a.semechko@gmail.com)
        % DATE: Dec.2014
        %


        if isobject(X), X=X.X; end
        if size(X,2)~=2
            error('This function only works for 2D data')
        end
        if sum(isnan(X(:)) | isinf(X(:)))>0
            error('Point data contains NaN or Inf entries. Remove them and try again.')
        end

        % Get the convex hull of the point set
        F=convhulln(X);
        F=unique(F(:));
        X=X(F,:);

        % Randomly permute the point set
        idx=randperm(size(X,1));
        X=X(idx(:),:);

        % Get the minimum bounding circle
        if size(X,1)<=3
            [R,C]=FitCirc2Points(X); 
            Xb=X;
            return
        end

        if size(X,1)<1E3
            try

                % Center and radius of the circle
                [R,C]=B_MinCircle(X,[]);

                % Coordiantes of the points used to compute parameters of the 
                % minimum bounding circle
                D=sum(bsxfun(@minus,X,C).^2,2);
                [D,idx]=sort(abs(D-R^2));
                Xb=X(idx(1:4),:);
                D=D(1:4);
                Xb=Xb(D<1E-6,:);
                [~,idx]=sort(Xb(:,1));
                Xb=Xb(idx,:);
                return
            catch
            end
        end

        % If we got to this point, then the recursion depth limit was reached. So 
        % need to break-up the the data into smaller sets and then recombine the 
        % results.
        M=size(X,1);
        dM=min(floor(M/4),300);
        res=mod(M,dM);
        n=ceil(M/dM);  
        idx=dM*ones(1,n);
        if res>0
            idx(end)=res;
        end

        if res<=0.25*dM 
            idx(n-1)=idx(n-1)+idx(n);
            idx(n)=[];
            n=n-1;
        end

        X=mat2cell(X,idx,2);
        Xb=[];
        for i=1:n

            % Center and radius of the circle
            [R,C,Xi]=B_MinCircle([Xb;X{i}],[]);    

            % 40 points closest to the circle
            if i<1
                D=abs(sum(bsxfun(@minus,Xi,C).^2,2)-R^2);
            else
                D=abs(sqrt(sum(bsxfun(@minus,Xi,C).^2,2))-R);
            end
            [D,idx]=sort(D);
            Xb=Xi(idx(1:40),:);

        end
        D=D(1:3);
        Xb=Xb(D/R*100<1E-3,:);
        [~,idx]=sort(Xb(:,1));
        Xb=Xb(idx,:);
            function [R,C,P]=B_MinCircle(P,B)

                if size(B,1)==3 || isempty(P)
                    [R,C]=Cmnf.FitCircle2Points(B); % fit circle to boundary points
                    return
                end

                % Remove the last (i.e., end) point, p, from the list
                P_new=P;
                P_new(end,:)=[];
                p=P(end,:);

                % Check if p is on or inside the bounding circle. If not, it must be
                % part of the new boundary.
                [R,C,P_new]=B_MinCircle(P_new,B); 
                if isnan(R) || isinf(R) || R<=eps
                    chk=true;
                else
                    chk=norm(p-C)>(R+eps);
                end

                if chk
                    B=[p;B];
                    [R,C]=B_MinCircle(P_new,B);
                    P=[p;P_new];
                end

            end
        end
        function [R,C]=FitCircle2Points(X)
            % Fit a circle to a set of 2 or at most 3 points in 3D space. Note that
            % point configurations with 3 collinear points do not have well-defined 
            % solutions (i.e., they lie on circles with infinite radius).
            %
            %   - X     : M-by-2 array of point coordinates, where M<=3.
            %   - R     : radius of the circle. R=Inf when the circle is undefined, as 
            %             specified above.
            %   - C     : 1-by-2 vector specifying the centroid of the circle. 
            %             C=nan(1,2) when the circle is undefined, as specified above.
            %
            % AUTHOR: Anton Semechko (a.semechko@gmail.com)
            % DATE: Dec.2014
            %


            N=size(X,1);
            if N>3
                error('Input must a N-by-2 array of point coordinates, with N<=3')
            end

            % Empty set
            if isempty(X)
                C=nan(1,2);
                R=nan; 
                return
            end

            % A single point
            if N==1
                C=X;
                R=0;
                return
            end

            % Line segment
            if N==2
                C=mean(X,1);
                R=norm(X(2,:)-X(1,:))/2;
                return
            end

            % Remove duplicate vertices, if there are any
            D=bsxfun(@minus,permute(X,[1 3 2]),permute(X,[3 1 2]));
            D=sqrt(sum(D.^2,3));
            D(1:(N+1):end)=Inf;
            chk=D<=1E-12;
            if sum(chk(:))>0
                for i=1:(N-1)
                    if size(X,1)<=i, break; end
                    idx=chk(i,:);
                    idx(1:i)=false;
                    idx=find(idx);
                    chk(idx,:)=[];
                    chk(:,idx)=[];
                    X(idx,:)=[];
                end
                [R,C]=FitCirc2Points(X);
                return
            end


            % Three unique, though possibly collinear points
            tol=1E-2; % collinearity/co-planarity threshold (in degrees)

            % Check for collinearity
            D12=X(2,:)-X(1,:); D12=D12/norm(D12);
            D13=X(3,:)-X(1,:); D13=D13/norm(D13);

            chk=abs(D12*D13(:));
            chk(chk>1)=1;
            if acos(chk)/pi*180<tol
                R=inf;
                C=nan(1,2);
                return
            end

            % Circle centroid
            A=2*bsxfun(@minus,X(2:3,:),X(1,:));
            b=sum(bsxfun(@minus,X(2:3,:).^2,X(1,:).^2),2);
            C=(A\b)';

            % Circle radius
            R=sqrt(sum((X(1,:)-C).^2,2));
        end
        
        function exportFigure(name,id)
            %Export figure to png file wiht 300 dpi
            %   - check if the export is preffered
            if Cmnf.enableFigureExports
                if nargin == 1
                    nm=name;
                else
                    nm=[name,'-',sprintf('%05d',id)];
                end
                print([Cmnf.pngExportPath,nm],'-r300','-dpng')
            end
        end
        
        function exportFigureForced(name,id)
            % Forced Export figure to png file wiht 300 dpi
            if nargin == 1
                nm=name;
            else
                nm=[name,'-',sprintf('%05d',id)];
            end
            print([Cmnf.pngExportPath,nm],'-r300','-dpng')
        end
        
        function [newX,newY]=preparefillData(xx,yy,useZero)
            %[Helper] - pretty plot data
            if nargin < 3
                useZero = true;
            end
            if useZero
                miny = 0;
            else
                miny = min(yy);
            end
            x0 = xx(1);
            xl = xx(length(xx));
            
            newX=[x0,xx,xl];
            newY=[miny,yy,miny];
        end
        
        function r=fill(xx,yy,color,lineStyle,alpha,edgecolor)
            %[Helper] - pretty plot
            if nargin < 4
                lineStyle='-';
            end
            if nargin < 5
                alpha=0.2;
            end
            if nargin < 6
                edgecolor = color;
            end
            a=fill(xx,yy,color);
            a.FaceAlpha=alpha;
            a.EdgeColor=edgecolor;
            a.LineStyle=lineStyle;
            r=a;
        end
    end
    
end

