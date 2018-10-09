ag=AvoidanceGrid(0,10,-pi/4,pi/4,-pi/6,pi/6,10,7,5);
s=0;
for k=1:7
    for l=1:5
        c=ag.layers(10).cells(k,l);
        s=s+ c.calculateSurface2(c.dEnd);
    end
end

s

ballSurface = 4*pi*c.dEnd^2;

ag=AvoidanceGrid(0,10,-pi,pi,-pi/2,pi/2,10,7,5);
s=0;
for k=1:7
    for l=1:5
        c=ag.layers(10).cells(k,l);
        s=s+ c.calculateSurface2(c.dEnd);
    end
end

s
s/ballSurface


