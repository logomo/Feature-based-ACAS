figure(1)
grid on
hold on 
plot3([0,15],[0,0],[0,0],'LineWidth',2,'Color','b');
plot3([0,10],[0,4],[0,2],'LineWidth',2,'Color','g');
plot3(0,0,0,'Marker','o','MarkerFaceColor','r','MarkerEdgeColor','r','MarkerSize',8)
plot3(10,0,0,'Marker','o','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',8)
plot3(10,4,2,'Marker','o','MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',8)
thetaSpread=pi/6;
phiSpread=pi/8;
distance = 10;
stepCount= 50;
a=sin(thetaSpread)*distance;                                
b=sin(phiSpread)*distance;
t=linspace(0,2*pi,stepCount);
elip = [distance*ones(1,stepCount);a*cos(t);b*sin(t)];
%plot3(elip(1,:),elip(2,:),elip(3,:),'r)
h=fill3(elip(1,:),elip(2,:),elip(3,:),'r','FaceAlpha',0.1,'EdgeColor', [1 0 0]);
plot3(10,a,0,'*k')
text(10,a+0.3,0,'d_{\theta_{max}}')
plot3(10,0,b,'*k')
text(10,0.3,b+0.5,'d_{\phi_{max}}')
plot3(10,-a,0,'*k')
text(10,-a-0.7,0,'-d_{\theta_{max}}')
plot3(10,0,-b,'*k')
text(10,0.3,-b-0.5,'-d_{\phi_{max}}')
plot3([10,10],[0,a],[0,0],'k')
plot3([10,10],[0,-a],[0,0],'k')
plot3([10,10],[0,0],[0,b],'k')
plot3([10,10],[0,0],[0,-b],'k')
plot3([10,10],[4,4],[0,2],'k')
plot3([10,10],[0,4],[2,2],'k')
text(10,4.3,2,'[d_d,d_\theta,d_\phi]')
text(-0.5,-0.5,0,'x_s')
text(10,-1.5,-0.5,'x(10)')
axis([-2 17 -6 6 -6 6])
grid off
set(gca,'visible','off')