shakyVehicle = Vehicle(State(0,0,0,0,0,0, 1));
shakyVehicle1 = Vehicle(State(0,0,0,0,0,0, 1));
shakyVehicle2 = Vehicle(State(0,0,0,0,0,0, 1));
shakyVehicle3 = Vehicle(State(0,0,0,0,0,0, 1));
shakyVehicle4 = Vehicle(State(0,0,0,0,0,0, 1));

shakyVehicle.forceSimulinkModel = true;
shakyVehicle1.forceSimulinkModel = true;
shakyVehicle2.forceSimulinkModel = true;
shakyVehicle3.forceSimulinkModel = true;
shakyVehicle4.forceSimulinkModel = true;

shakyVehicle.flyBuffer([MovementType.Straight,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,...
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                       ]);
                   
shakyVehicle1.flyBuffer([MovementType.Left,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,...
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                       ]);
shakyVehicle2.flyBuffer([MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,...
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                       ]);
shakyVehicle3.flyBuffer([MovementType.Left,...
                        MovementType.Left,... 
                        MovementType.Straight,... 
                        MovementType.Straight,... 
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,...
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                       ]);
shakyVehicle4.flyBuffer([MovementType.Right,...
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,...
                        MovementType.Left,...
                        MovementType.Right,...
                        MovementType.Left,... 
                        MovementType.Right,... 
                        MovementType.Right,... 
                        MovementType.Left,...
                       ]);
straightVehicle = Vehicle(State(0,0,0,0,0,0, 1));
for k=1:13
    straightVehicle.fly(MovementType.Straight);
end

%hold on
%shakyVehicle3.plotTrajectoryWide('g')
%shakyVehicle4.plotTrajectoryWide('g')
%shakyVehicle.plotTrajectoryWide('b')
%shakyVehicle2.plotTrajectoryWide('b')
%shakyVehicle1.plotTrajectoryWide('b')
%straightVehicle.plotTrajectoryWide('r')
%hold off


ag= AvoidanceGrid(0,13,-(2*pi)/6,(2*pi)/6,-0.001,0.001,5,3,1);
%ag.plotRaster


%legend([tt2,tt1],'Shaky trajectories','Straigth trajectory')
figure(1)
hold on
tt1 = plot(NaN,NaN,'LineStyle','-','Color','r','LineWidth',4);
tt2 = plot(NaN,NaN,'LineStyle','-','Color','b','LineWidth',4);
tt3 = plot(NaN,NaN,'LineStyle','-','Color','g','LineWidth',4);
h=[tt2,tt1];
hold off
hold on
shakyVehicle.plotTrajectoryWide('b')
shakyVehicle2.plotTrajectoryWide('b')
shakyVehicle1.plotTrajectoryWide('b')
straightVehicle.plotTrajectoryWide('r')
ag.plotRaster
daspect([1,1,1])
legend([tt2,tt1],'Shaky trajectories','Straigth trajectory')
axis off

figure(2)
hold on
tt1 = plot(NaN,NaN,'LineStyle','-','Color','r','LineWidth',4);
tt2 = plot(NaN,NaN,'LineStyle','-','Color','b','LineWidth',4);
tt3 = plot(NaN,NaN,'LineStyle','-','Color','g','LineWidth',4);
h=[tt2,tt1];
hold off
hold on
shakyVehicle3.plotTrajectoryWide('r')
shakyVehicle4.plotTrajectoryWide('g')
shakyVehicle.plotTrajectoryWide('b')
shakyVehicle2.plotTrajectoryWide('b')
shakyVehicle1.plotTrajectoryWide('b')
straightVehicle.plotTrajectoryWide('b')
ag.plotRaster
daspect([1,1,1])
legend([tt1,tt2,tt3],'Left footprint','Center footprint','Right footprint')
axis off

figure(3)
hold on
tt1 = plot(NaN,NaN,'LineStyle','-','Color','r','LineWidth',4);
tt2 = plot(NaN,NaN,'LineStyle','-','Color','b','LineWidth',4);
tt3 = plot(NaN,NaN,'LineStyle','-','Color','g','LineWidth',4);
h=[tt2,tt1];
hold off
hold on
shakyVehicle3.plotTrajectoryWide('r')
shakyVehicle4.plotTrajectoryWide('g')
shakyVehicle.plotTrajectoryWide('b')
%shakyVehicle2.plotTrajectoryWide('b')
%shakyVehicle1.plotTrajectoryWide('b')
%straightVehicle.plotTrajectoryWide('b')
ag.plotRaster
daspect([1,1,1])
legend([tt1,tt2,tt3],'Left candidate','Center candidate','Right candidate')
axis off

figure(1)
Cmnf.exportFigureForced('Turn-minimizing')
figure(2)
Cmnf.exportFigureForced('Coverage-ratio')
figure(3)
Cmnf.exportFigureForced('Minimal-Coverage-set')