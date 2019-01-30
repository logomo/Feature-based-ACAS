ag = AvoidanceGrid(0,4,-pi/4,pi/4,-pi/6,pi/6,4,4,3);
ag2 = AvoidanceGrid(0,4,-pi,pi,-pi/6,pi/6,4,16,3);
ag.rasterColor = '.-r';
ag2.rasterColor = '.-c';
figure(1)

c130(0,-0.25,0,...
                'roll',0,...
                'pitch',0,...
                'yaw',-90,...
                'color','b',...
                'linecolor','b',...
                'scale',1/40);
                %  c130(...,'wing',WingColor)
                %  c130(...,'tailwing',TailwingColor)
                %  c130(...,'fin',FinColor)
                %  c130(...,'prop',PropellerColor)
                %  c130(...,'scale',SizeScaleFactor)
                %  c130(...,'z',ZScaleFactor)
                %  c130(...,'linestyle','LineStyle')
                %  c130(...,'linecolor',LineColor)
hold on
ag2.plotRaster
ag.plotRaster

hold off
daspect([1,1,1])
view(124.4,38)
grid on
Cmnf.exportFigure('LiDARRasterRange')