%obj=BarrelConstraint([0,0],5);
%obj.plot;

%unusual building r=5
%pg1=[-4,3;4,3;4,-3;2,-3;,2,0;0,0;0,-3;-4,-3;-4,-1;-2,-1;-2,2;-4,2;-4,3]';
%plot(pg1(1,:),pg1(2,:))

%penta trap r=5
%pg2=[-4,-3;-4,3;0,5;4,3;4,-3;3,-3;3,2;0,3;-3,2;-3,-3;-4,-3]';
%plot(pg2(1,:),pg2(2,:))

%H building r=5
%pg3=[-4,3; -4,-3; -2,-3; -2,-1; 2,-1; 2,-3;4,-3; 4,3; 2,3; 2,1; -2,1; -2,3;-4,3]';
%plot(pg3(1,:),pg3(2,:))

% square r=5
%pg4=[2.5,2.5;2.5,-2.5;-2.5,-2.5;-2.5,2.5;2.5,2.5]';
%plot(pg4(1,:),pg4(2,:))

% poly5 r=5
%pg5=[-3.99,3.02;0.88,4.92;4.92,0.89;1.9,-4.63;-4.87,-1.11;-3.99,3.02]';
%plot(pg5(1,:),pg5(2,:))
type=ExamplePolygonType.PentaTrap;
scale= 25.0;
rotation = 0;
center = [12;14;-5];
base=ExamplePolygonType.getPolygonData(type);

scaled = base.*scale;
rotated = Cmnf.rot2D(rotation,scaled);
shifted = rotated + center(1:2);
plot(shifted(1,:),shifted(2,:))

pgn= PolyConstraint(shifted);
figure(2)
pgn.plot