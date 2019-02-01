%% Line intersection test implementaiton prototype
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
cellMap=[];                                                     
for k= 1:length(ag.layers)
    for l=1:ag.layers(k).horizontalCellCount
        for m = 1:ag.layers(k).verticalCellCount
            cell = ag.layers(k).cells(l,m);
            center = cell.center;
            position = [k;l;m];
            cellMap=[cellMap,[center;position]];
        end
    end
end

point = [8;-8;0];
velocity = [0;2;0];
normVelocity = velocity/norm(velocity,2);
centeredCellMap=cellMap(1:3,:) - point * ones(1,length(cellMap));

normedCenteredCellMap = centeredCellMap;
for k=1:length(normedCenteredCellMap)
    vn=norm(normedCenteredCellMap(:,k),2);
    normedCenteredCellMap(:,k) = normedCenteredCellMap(:,k)/vn;
end
diffNormedCellMap = normedCenteredCellMap... 
                    - normVelocity*ones(1,length(normedCenteredCellMap));
normVector = [1,1,1]*(diffNormedCellMap.^2);
[v,in] = sort(normVector);

% for one
ccs = [];
firstRun = 1;
cCell = 0;
while firstRun ||  cCell~=0
    firstRun = 0;
    cCell = 0;
    id=in(1);
    in(1)=[];
    wpt=centeredCellMap(:,id);
    time= norm(wpt,2)/norm(velocity,2);
    projPo = point + velocity*time;
    cCell=ag.getCellEuclidian(projPo);
    if cCell ~=0
        tVal = cCell.center - cellMap(1:3,id);
        if tVal == 0
            ccs= [ccs, cCell];
        end
    end
end

for k = 1:length(ccs)
    ccs(k).pObstacle = 1;
end

figure(1)
ag.plotHorizontalSlice(3,StatisticType.Obstacle)


%
%t= 0:0.1:15;

%x = [-5;-3;-4]*t + [4;6;8]*ones(1,length(t));
%figure(1)
%plot3(x(1,:),x(2,:),x(3,:));
%xp = [];
%for l =1:length(x)
%    xp = [xp,Cmnf.euc2plan(x(1,l),x(2,l),x(3,l))];
%end
%xp = xp -Cmnf.euc2plan(4,6,6)*ones(1,length(t));
%diff = xp(:,2:151)-xp(:,1:150);
%slope = [mean(diff(1,:));mean(diff(2,:));mean(diff(3,:))];
%mse = sum(sum(((diff - slope*ones(1,length(diff))).^2)))