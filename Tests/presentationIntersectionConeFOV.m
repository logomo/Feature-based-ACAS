%% Test adversarial behaviour with different spread
clear all

%adv1=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],pi/4,pi/6);     % big spread adversarial
adv2=AdversaryVehicle([8;8;0],[1;0;0;-pi/2],pi/8,pi/12);    %Medium spread adversarial
%adv3=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],pi/12,pi/16);   %Small spread adversarial
%adv4=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],0,0);           %Zero spread adversarial
adp = [adv2];
for k = 0                                                 %plot results
    adv = adp(k+1);
    ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);
    figure(k+1)
    rr=ag.plotRasterRange([0,0,0,0,0,0]);
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    title('Intersection view')
    coat = [];
    
    for l=0:15
        s=adv.calculateElipse(l,1);
        coat=[coat,s];
    end
    coat=coat(1:3,:);
    coat=coat';
    k = boundary(coat);
    hold on
    h=trisurf(k,coat(:,1),coat(:,2),coat(:,3),'Facecolor','red','FaceAlpha',0.5)
    set(h, 'edgecolor','none')
    hold off
    rr=rr'
    k = boundary(rr);
    hold on
    h=trisurf(k,rr(:,1),rr(:,2),rr(:,3),'Facecolor','blue','FaceAlpha',1)
    set(h, 'edgecolor','none')
    hold off
end

