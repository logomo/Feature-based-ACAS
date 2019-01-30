%We just need plain avoidance grid
ag=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,1,1);
ag1=AvoidanceGrid(0,10,-0.1683,0.1683,-pi/6,pi/6,10,1,1);
ag2=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,4,1);


figure(1)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    ag1.plotBaseSlice(5:10,1,1,0,1,StatisticType.Visibility);
    cell=ag1.layers(4).cells(1,1);
    cell.pReachability=0;
    ag1.plotBaseSlice(4,1,1,0,1,StatisticType.Reachability);
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    cell=ag1.layers(4).cells(1,1);
    text(cell.center(1)-0.3,cell.center(2),'O_1');
    for k=5:10
        cell=ag1.layers(k).cells(1,1);
        text(cell.center(1)-0.3,cell.center(2),'60%','Color','red')
    end
    
    cell=ag.layers(2).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(3).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag2.layers(4).cells(4,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag2.layers(5).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag2.layers(6).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag2.layers(7).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag2.layers(8).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag2.layers(9).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag2.layers(10).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag.layers(9).cells(1,1);
    hold on
    plot(cell.center(1),cell.center(2),'Marker','o','MarkerSize',26,'MarkerFaceColor','b','MarkerEdgeColor',[0 0 0]);
    hold off
    text(8.2,0.65,'O_M');
    cell=ag.layers(9).cells(1,1);
    text(cell.center(1)-0.3,cell.center(2),'60%','Color','white')
    
figure(2)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    
    
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])

    
    cell=ag.layers(2).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(3).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(4).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(5).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(6).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(7).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(8).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag2.layers(9).cells(4,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(10).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(9).cells(1,1);
    hold on
    plot(cell.center(1),cell.center(2),'Marker','o','MarkerSize',26,'MarkerFaceColor','b','MarkerEdgeColor',[0 0 0]);
    hold off
    text(8.2,0.65,'O_M');
    cell=ag.layers(9).cells(1,1);
    text(cell.center(1)-0.3,cell.center(2),'0%','Color','white')
    
figure(3)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    ag1.plotBaseSlice(10,1,1,0,1,StatisticType.Visibility);
    cell=ag1.layers(9).cells(1,1);
    cell.pReachability=0;
    ag1.plotBaseSlice(9,1,1,0,1,StatisticType.Reachability);
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    text(8.2,-0.65,'O_1');
    for k=10
        cell=ag1.layers(k).cells(1,1);
        text(cell.center(1)-0.3,cell.center(2),'60%','Color','red')
    end
    
    cell=ag.layers(2).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(3).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(4).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(5).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(6).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(7).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag.layers(8).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag2.layers(9).cells(4,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag2.layers(10).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag.layers(9).cells(1,1);
    hold on
    plot(cell.center(1),cell.center(2),'Marker','o','MarkerSize',26,'MarkerFaceColor','b','MarkerEdgeColor',[0 0 0]);
    hold off
    text(8.2,0.65,'O_M');
    cell=ag.layers(9).cells(1,1);
    text(cell.center(1)-0.4,cell.center(2),'100%','Color','white')
    
for k=1:3
    figure(k);
    set(gca,'visible','off');
    print(gcf,['fig',mat2str(k),'.png'],'-dpng','-r300');
end
    