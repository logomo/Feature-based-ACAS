farCount = 8; % spread 8 trajectories far away from center
nearCount = 1; % spread 1 trajectory close to cell center
ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
ag.precalculateCellSpreadReachSet(LinearizedModel(),farCount,nearCount);

figure(1)
for k=1:10
    layer=ag.layers(k);
    x=1:layer.horizontalCellCount;
    y=1:layer.verticalCellCount;
    zmin=zeros(layer.horizontalCellCount,layer.verticalCellCount);
    zmax=zmin;
    for i=1:layer.horizontalCellCount
        for j = 1:layer.verticalCellCount
            zmin(i,j)=layer.cells(i,j).minimalEntryTime;
            zmax(i,j)=layer.cells(i,j).maximalLeaveTime;
        end
    end
    subplot(2,5,k)
    hold on
    stem3(x,y,zmax','--o r')
    stem3(x,y,zmin','--o b')
    hold off
    grid on
    xlabel('h_i[index]')
    ylabel('v_i[index]')
    zlabel('t[s]')
    title(['Arrival/Leave times at ',mat2str(k),'^{th} layer'])
end



