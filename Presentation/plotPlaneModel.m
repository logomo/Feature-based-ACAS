function handles = plotPlaneModel(vehicle,radius,farben)
    posOr=vehicle.actualPositionOrientation;
    position=posOr(1:3);
    orientation=rad2deg(posOr(4:6));
    scale =radius/40;
    handles=...
    c130(position(1),position(2),position(3),...
        'roll',orientation(1),...
        'pitch',-orientation(2),...
        'yaw',orientation(3)-90,...
        'color',farben,...
        'scale',scale);
    %c130(...,'fuselage',FuseLageColor)
    %c130(...,'wing',WingColor)
    %c130(...,'tailwing',TailwingColor)
    %c130(...,'fin',FinColor)
    %c130(...,'prop',PropellerColor)
    %c130(...,'scale',SizeScaleFactor)
end

