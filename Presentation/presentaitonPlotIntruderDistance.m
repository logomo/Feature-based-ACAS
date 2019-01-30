intruders=obj.intruders;
posTimId = [];
for intruder=intruders;
    posTimOr=intruder.posTime;
    [m,n]=size(posTimOr);
    dat=[posTimOr(1:4,1:n);intruder.id*ones(1,n)];
    posTimId=[posTimId,dat];
end

logs=obj.missionLog;
dist=[];
for log=logs
    posOr=log.vehiclePosOrBefore;
    pos=posOr(1:3);
    simTime=log.simulationTime;
    indexes=find(posTimId(4,:)==simTime);
    if ~isempty(indexes)
        records=posTimId(:,indexes);
        [m,n]=size(records);
        criterion=sqrt([1 1 1]*((records(1:3,:)-pos*ones(1,n)).^2));
        [v,i]=min(criterion);
        dist=[dist,[v;records(:,i)]];
    end
end


for k =1:length(dist)
    hold on 
    if dist(1,k) <=1.2
        c='r';
    else
        c='g';
    end
    plot(dist(5,k),dist(1,k),'Marker','o','MarkerFaceColor',c,'MarkerEdgeColor',c)
    text(dist(5,k)+.25,dist(1,k),[mat2str(dist(6,k)),'.'])
    hold off
end
hold on 
plot(dist(5,:),dist(1,:),'g')
plot([min(dist(5,:)),max(dist(5,:))],[1.2,1.2],'--r')
grid on

xmax=max(dist(5,:));
xmin=min(dist(5,:));
ymin=0;
ymax=ceil(max(dist(1,:)));
axis([xmin,xmax,ymin,ymax]);
 xlabel('Collision time [s]')
ylabel('Collision distance [m]')
