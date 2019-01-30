x=linspace(0,5,300);
y= [ 0*ones(1,60), tanh(10*linspace(0,1,60)),(tanh(-10*linspace(0,1,60))+1),(tanh(-10*linspace(0,1,60))),(tanh(10*linspace(0,1,60))-1)]*(pi/12);
y2=[zeros(1,120),tanh(10*linspace(0,1,60)),(tanh(-10*linspace(0,1,60))+1),(tanh(-10*linspace(0,1,60)))]*(pi/12);
figure(1);
subplot(2,1,1);
hold on;
grid on;
set(gca,'ytick',linspace(-pi/12,pi/12,3))
set(gca,'yticklabel',{'-\pi/12','0','\pi/12'});
title('Example of Movement automaton [pitch(t),yaw(t)]')
ylabel('Pitch [rad]')
xlabel('Time [s]')
for k=0:4
    rectanglePlot(k,k+1,pi/12+(pi/12/8),-pi/12-(pi/12/8),'k')
end
for k=0:4
    rectanglePlot(k,k+.237,pi/12+(pi/12/20),-pi/12-(pi/12/20),'r')
    rectanglePlot(k+.237,k+1,pi/12+(pi/12/20),-pi/12-(pi/12/20),'m')
end
plot(x,y,'b','linewidth',2);
hold off;

subplot(2,1,2);
hold on;
grid on;
ylabel('Yaw [rad]')
xlabel('Time [s]')
for k=0:4
    rectanglePlot(k,k+1,pi/12+(pi/12/8),-pi/12-(pi/12/8),'k')
end
for k=0:4
    rectanglePlot(k,k+.237,pi/12+(pi/12/20),-pi/12-(pi/12/20),'r')
    rectanglePlot(k+.237,k+1,pi/12+(pi/12/20),-pi/12-(pi/12/20),'m')
end
k=0;
rectanglePlot(k,k+1,pi/12+(pi/12/8),-pi/12-(pi/12/8),'k',2)
k=2;
rectanglePlot(k,k+.237,pi/12+(pi/12/20),-pi/12-(pi/12/20),'r',2)
k=4;
rectanglePlot(k+.237,k+1,pi/12+(pi/12/20),-pi/12-(pi/12/20),'m',2)
plot(x,y2,'b','linewidth',2);
set(gca,'ytick',linspace(-pi/12,pi/12,3))
set(gca,'yticklabel',{'-\pi/12','0','\pi/12'});
hold off;