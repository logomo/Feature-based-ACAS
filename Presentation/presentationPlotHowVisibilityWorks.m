%Mission we just need plain avoidance grid
ag=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,1,1);
ag2=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,4,1);
ag3=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,6,1);
ag4=AvoidanceGrid(0,10,-0.3366,0.3366,-pi/6,pi/6,10,5,1);

figure(1)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    
    cell=ag4.layers(2).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(3).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(3).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(4).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(5).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(6).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(7).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(8).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(9).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(10).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
figure(2)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    ag2.plotBaseSlice(4:10,1,1,0,1,StatisticType.Visibility);
    cell1=ag2.layers(3).cells(1,1);
    cell1.pReachability=0;
    ag2.plotBaseSlice(3,1,1,0,1,StatisticType.Reachability);
    cell2=ag3.layers(5).cells(6,1);
    cell2.pReachability=0;
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    text(cell1.center(1)-0.3,cell1.center(2),'O_1')
    

    for k=4:10
        cell=ag2.layers(k).cells(1,1);
        text(cell.center(1)-0.3,cell.center(2),'25%','Color','red')
    end


    cell=ag4.layers(2).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(3).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(4).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(5).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(6).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(7).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(8).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(9).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(10).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    
figure(3)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    ag2.plotBaseSlice(4:10,1,1,0,1,StatisticType.Visibility);
    ag3.plotBaseSlice(6:10,6,1,0,1,StatisticType.Visibility);
    cell1=ag2.layers(3).cells(1,1);
    cell1.pReachability=0;
    ag2.plotBaseSlice(3,1,1,0,1,StatisticType.Reachability);
    cell2=ag3.layers(5).cells(6,1);
    cell2.pReachability=0;
    ag3.plotBaseSlice(5,6,1,0,1,StatisticType.Reachability);
    cell3=ag4.layers(7).cells(3,1);
    cell3.pReachability=0;

    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    text(cell1.center(1)-0.3,cell1.center(2),'O_1')
    text(cell2.center(1)-0.3,cell2.center(2),'O_2')

    for k=4:10
        cell=ag2.layers(k).cells(1,1);
        text(cell.center(1)-0.3,cell.center(2),'25%','Color','red')
    end

    for k=6:10
        cell=ag3.layers(k).cells(6,1);
        text(cell.center(1)-0.3,cell.center(2),'15%','Color','red')
    end


    cell=ag4.layers(2).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(3).cells(3,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(4).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(5).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(6).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(7).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(8).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(9).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(10).cells(3,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    
figure(4)
    ag.plotBaseSlice(1:10,1,1,0,1,StatisticType.Reachability);
    ag2.plotBaseSlice(4:10,1,1,0,1,StatisticType.Visibility);
    ag3.plotBaseSlice(6:10,6,1,0,1,StatisticType.Visibility);
    ag4.plotBaseSlice(8:10,3,1,0,1,StatisticType.Visibility);
    cell1=ag2.layers(3).cells(1,1);
    cell1.pReachability=0;
    ag2.plotBaseSlice(3,1,1,0,1,StatisticType.Reachability);
    cell2=ag3.layers(5).cells(6,1);
    cell2.pReachability=0;
    ag3.plotBaseSlice(5,6,1,0,1,StatisticType.Reachability);
    cell3=ag4.layers(7).cells(3,1);
    cell3.pReachability=0;
    ag4.plotBaseSlice(7,3,1,0,1,StatisticType.Reachability);
    hold on
    plot(0,0,'Marker','o','MarkerSize',26,'MarkerFaceColor','k','MarkerEdgeColor',[0 0 0]);
    text(-0.5,1,'Vehicle')
    hold off
    axis([-1,10,-4,4])
    text(cell1.center(1)-0.3,cell1.center(2),'O_1')
    text(cell2.center(1)-0.3,cell2.center(2),'O_2')
    text(cell3.center(1)-0.3,cell3.center(2),'O_3')

    for k=4:10
        cell=ag2.layers(k).cells(1,1);
        text(cell.center(1)-0.3,cell.center(2),'25%','Color','red')
    end

    for k=6:10
        cell=ag3.layers(k).cells(6,1);
        text(cell.center(1)-0.3,cell.center(2),'15%','Color','red')
    end

    for k=8:10
        cell=ag4.layers(k).cells(3,1);
        text(cell.center(1)-0.3,cell.center(2),'20%','Color','red')
    end

    cell=ag4.layers(2).cells(4,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(3).cells(4,1);
    text(cell.center(1)-0.4,cell.center(2),'100%')
    cell=ag4.layers(4).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(5).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'75%')
    cell=ag4.layers(6).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(7).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'60%')
    cell=ag4.layers(8).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag4.layers(9).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
    cell=ag4.layers(10).cells(4,1);
    text(cell.center(1)-0.3,cell.center(2),'40%')
 
    
for k=1:4
    figure(k);
    set(gca,'visible','off');
    print(gcf,['fig',mat2str(k),'.png'],'-dpng','-r300');
end