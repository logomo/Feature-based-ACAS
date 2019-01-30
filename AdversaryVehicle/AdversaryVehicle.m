classdef AdversaryVehicle<AbstractAdversaryVehicle
    %ADVERSARYVEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        positionVector              %[x,y,z] global position
        velocityVector              %[\alpha,\beta,\gamma] RPY angles, see Cmnf.rot3D
        thetaSpread                 %Horizontal adversary spread - symetrical +- theta
        phiSpread                   %Vertical adversary spread - symetrical +- varphi
        debug = 0;                  %Debug indicatior - keep it 0, for sanity checks only
    end
    
    methods
        
        function obj=AdversaryVehicle(pv,vv,ts,ps)
            % Constructor method
            %   pv - position vector
            %   vv - velocity vector
            %   ts - horizontal spread - positive radians
            %   vs - vertical spread - positive radians
            obj.positionVector=pv;
            obj.velocityVector=vv;
            obj.thetaSpread=ts;
            obj.phiSpread=ps;
        end
        
        
        function r=calculateElipse(obj,distance,stepSize)
            % calculate ecliplse - numeric approximation of intersection point
            % stepSize should be same as grid size, its standard numeric
            % approximation  guaranting uuniform distribution within given
            % conditions
            %   distance - distance from point of origin at time of flight
            %   stepSize - avoidanceGrid.stepSize or lesser (performance tuning)
            % Generate covariant elyptical slice on  direct distance
            a=sin(obj.thetaSpread)*distance;                                %calculate horizontal spread in meters
            b=sin(obj.phiSpread)*distance;                                  %calculate vertical spread
            flag = 1;r=[];                                                  % use standard elypse shrinking algorithm
            while flag == 1 || a >= stepSize || b >= stepSize
                cira = 2*pi*a;
                cirb = 2*pi*b;
                cir = (cira+cirb)/2;
                stepCount = round(cir/stepSize);                            %calculate step count according to numeric aproximator
                if stepCount < Cmnf.eliSteps
                    stepCount = Cmnf.eliSteps;
                end
                if a==0 && b ==0
                    stepCount = 1;
                end
                flag = 0;
                t=linspace(0,2*pi,stepCount);
                elip = [distance*ones(1,stepCount);a*cos(t);b*sin(t)];      %generate proto elypse for given a,b params
                if obj.debug == 1
                    hold on
                        figure(1)
                        plot3(elip(1,:),elip(2,:),elip(3,:),'b');
                    hold off
                end
                r=[r, elip];
                if (a-stepSize) >= stepSize && (b-stepSize) >= stepSize
                    a=a-stepSize;
                    b=b-stepSize;
                else if (a-stepSize) >= stepSize
                        a=a-stepSize;
                    else if (b-stepSize) >= stepSize
                            b=b-stepSize;
                        else break;
                        end
                    end    
                end
            end
            % calculate probabilistic distribution with appeal on
            % centristic approach
            n=r(2:3,:);                                                     %standard random distributuin initialization
            covh=n(1,:)*(n(1,:)')/(length(n)-1);                            %covariance of x parameters
            covv=n(2,:)*(n(2,:)')/(length(n)-1);                            %covariance of y parameters
            pdfh=normpdf(n(1,:),0,sqrt(covh));                              %probability density function horizontal
            pdfv=normpdf(n(2,:),0,sqrt(covv));                              %probability density function vertical
            %sanity check for NaN if zero distribution
            for k = 1:length(pdfh)                                          %pdf for zero covariance == inf
                if (isnan(pdfh(k)))
                    pdfh(k) = 1;
                end
                if (isnan(pdfv(k)))
                    pdfv(k) = 1;
                end
            end
            pdf = (pdfh + pdfv)/2;                                          %use mean for horizontal+vertical pdf
            r=[r;pdf];
            if obj.debug==1
                hold on
                    figure(2);
                    plot3(r(2,:),r(3,:),r(4,:));
                hold off 
            end
            % calculate time of arival for each point
            velocity = obj.velocityVector(1);                               % time of arival for each point at given velocity
            [m,n]=size(r');
            tim = zeros(1,m);
            for k= 1:m
                tim(k)= norm(r(1:3,k))/velocity;
            end
            r=[r;tim];
            if obj.debug ==1
                hold on
                    figure(3)
                    plot3(r(2,:),r(3,:),r(5,:));
                hold off
            end
            % rotation 
            mat = r(1:3,:);                                                 %standard rotation and offseting for global coordinates
            angl=obj.velocityVector(2:4);                                   %adversary is using x-axis as main - same as vehicle and avoidance grid
            mat = Cmnf.rot3D(angl(1),angl(2),angl(3),mat);
            % offsetting
            vec = obj.positionVector;
            mat = Cmnf.ofst(vec,mat);
            % merginf final result
            r(1:3,:)=mat;                                                  
            if obj.debug ==1
                hold on
                    figure(4)
                    plot3(r(1,:),r(2,:),r(3,:));
                hold off
            end
        end
    end
end

