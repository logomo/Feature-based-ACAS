clear;
debug=1;
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
ag.linearModel=LinearizedModel;

adversaryPoint = [10;0;0];
adversaryVelocity = [-1;0;0];
vehicleVelocity=ag.linearModel.getVelocity;
maximumGridRange= 2*(ag.dEnd -ag.dStart);
maximumTime = maximumGridRange/norm(adversaryVelocity,2);
stepTime = ag.stepLayer/norm(adversaryVelocity,2);
time=0:stepTime:maximumTime;

% initial search
cells = [];
sucTime = [];
register = containers.Map;
if debug == 1
    points = [];
end
for t=time
    testPoint = adversaryPoint+adversaryVelocity*t;
    if debug ==1
        points=[points,testPoint];
    end
    cell=ag.getCellEuclidian(testPoint);
    if cell ~=0
       key =  mat2str(cell.ijkIndex);
       if ~register.isKey(key)
            register(key)=cell;
            cells = [cells,cell];
            sucTime=[sucTime,t];
       end
    end
end

% spanning range offset search
for k=sucTime
    flag = 1;
    siz = 3;
    while flag 
       flag=0;
       for t=linspace(0,stepTime,siz)
            testPoint = adversaryPoint+adversaryVelocity*(t+k);
            cell=ag.getCellEuclidian(testPoint);
            if cell ~=0
                key =  mat2str(cell.ijkIndex);
                if ~register.isKey(key)
                    register(key)=cell;
                    cells = [cells,cell];
                    sucTime=[sucTime,t];
                    flag=1;
                end
            end
       end
       siz=siz*2;
    end
    % and one more time but in reverse
    flag = 1;
    siz = 3;
    while flag 
       flag=0;
       for t=linspace(0,-stepTime,siz)
            testPoint = adversaryPoint+adversaryVelocity*(t+k);
            cell=ag.getCellEuclidian(testPoint);
            if cell ~=0
                key =  mat2str(cell.ijkIndex);
                if ~register.isKey(key)
                    register(key)=cell;
                    cells = [cells,cell];
                    sucTime=[sucTime,t];
                    flag=1;
                end
            end
       end
       siz=siz*2;
    end
end

for k = 1:length(cells)
    cells(k).pObstacle = 1;
end
ag.plotHorizontalSlice(3,StatisticType.Obstacle)
