%% Adversary probability distribution function test example
clear
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);                        % Initialize standard grid
adv=AdversaryVehicle([5;5;5],[1;0;0;0],pi/4,pi/6);                          % Create adversary at global position [5,5,5], with velocity 1 and uncertainities horizontal pi/4, verticalpi/6


r=adv.calculateElipse(50,1);                                                %Create figures

s=[r(2,:);r(3,:);r(4,:)];

%coat=s';
%    k = boundary(coat);
%    hold on
%    h=trisurf(k,coat(:,1),coat(:,2),coat(:,3),'Facecolor','red','FaceAlpha',0.5)
%k=boundary(s');
f=figure(1)
[xq,yq] = meshgrid(min(s(1,:)):.2:max(s(1,:)), min(s(2,:)):.2:max(s(2,:)));
vq = griddata(s(1,:),s(2,:),s(3,:),xq,yq);
mesh(xq,yq,vq)
zlabel('P(x(t), d_\theta , d_\phi)')
ylabel('d_\phi [m]')
xlabel('d_\theta [m]')
set(gca, 'xtick', [min(s(1,:)),0,max(s(1,:))], 'ytick', [min(s(2,:)),0,max(s(2,:))]) ;
set(gca, 'xticklabels', {'-\sigma_\theta','0','\sigma_\theta'}) ;
set(gca, 'yticklabels', {'-\sigma_\phi','0','\sigma_\phi'}) ;
set(gca, 'zlim', [min(s(3,:)),max(s(3,:))]) ;
set(gca, 'ztick',[0.0100    0.0150    0.0200    0.0250]);
set(gca, 'zticklabels', {'','','',''}) ;