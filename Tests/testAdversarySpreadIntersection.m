%% Test adversarial behaviour with different spread
clear all

adv1=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],pi/4,pi/6);     % big spread adversarial
adv2=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],pi/8,pi/12);    %Medium spread adversarial
adv3=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],pi/12,pi/16);   %Small spread adversarial
adv4=AdversaryVehicle([8;-8;0],[1;0;0;pi/2],0,0);           %Zero spread adversarial
adp = [adv1,adv2,adv3,adv4];
for k = 0:3                                                 %plot results
    adv = adp(k+1);
    ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,6,4);
    figure(k+1)
    subplot(2,2,1)
    ag.plotRasterRange([0,0,0,0,0,0])
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    title('Intersection view')
    hold on
        for l=0:15
            s=adv.calculateElipse(l,1);
            plot3(s(1,:),s(2,:),s(3,:),'r');
        end
    hold off

    points = ag.putAdversarial(adv);
    hold on
    plot3(points(1,:),points(2,:),points(3,:),'r*')
    hold off

   
    figure(k+1)

    subplot(2,2,2)
    ag.plotBaseSlice(1:10,1:6,2,0,1,3);
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    title('Close layer')
    
    subplot(2,2,3)
    ag.plotBaseSlice(1:10,1:6,3,0,1,3);
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    title('Direct intersection layer')
    
    subplot(2,2,4)    
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    ag.plotBaseSlice(1:10,1:6,4,0,1,3);
    title('Bottom layer')
end

