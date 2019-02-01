%% 2D eclipse intersection with Avoidance grid, used in Spread intersection intruder model
clear;
position = [6;8;0];
velocity=[0;-1;0];
stepSize=1;
distance=20;
thetaSpread=pi/4;
phiSpread=pi/6;
debug=1

% Generate covariant elyptical slice on  direct distance
a=sin(thetaSpread)*distance;                                
b=sin(phiSpread)*distance;                                  
flag = 1;
r=[];                                                  
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
    if debug == 1
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
r= [r,[distance;0;0]];
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
%Normalize the results
r(4,:)=r(4,:)./sum(r(4,:));
if debug==1
    hold on
        figure(2);
        plot3(r(2,:),r(3,:),r(4,:));
    hold off 
end
% calculate time of arival for each point
velocityNormalized = norm(velocity,2);                               % time of arival for each point at given velocity
[m,n]=size(r');
tim = zeros(1,m);
for k= 1:m
    tim(k)= norm(r(1:3,k))/velocityNormalized;
end
r=[r;tim];
if debug ==1
    hold on
        figure(3)
        plot3(r(2,:),r(3,:),r(5,:));
    hold off
end
% rotation 
mat = r(1:3,:);                                                 %standard rotation and offseting for global coordinates
mat = Cmnf.align3Dvec(velocity,mat);
% offsetting
vec = position;
mat = Cmnf.ofst(vec,mat);
% merginf final result
r(1:3,:)=mat;                                                  
if debug ==1
    hold on
        figure(4)
        plot3(r(1,:),r(2,:),r(3,:));
    hold off
end