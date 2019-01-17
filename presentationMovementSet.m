orig =Vehicle(State(0,0,0,0,0,0,1));
movel = [];
for k=1:9
    vor=Vehicle(State(0,0,0,0,0,0,1))
    movel = [movel,vor];
    vor.fly(k-1);
    vor.fly(0);
    vor.fly(k-1);
    vor.fly(0);
    vor.fly(k-1);
    vor.fly(0);
    vor.fly(k-1);
    vor.fly(0);
    
end

hold on
    orig.plotTrajectory('r')
    movel(1).plotTrajectory('b')
    for k=2:5
        movel(k).plotTrajectory('c')
    end
    for k=6:9
        movel(k).plotTrajectory('m')
    end
    grid on
    daspect([1 1 1]);
    view(-75,20);
    Cmnf.exportFigure('MovementSet');
hold off

