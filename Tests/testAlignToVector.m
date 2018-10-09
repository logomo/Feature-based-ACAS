clear;



vec=100*rand(3,1)-50;
n=norm(vec,2);
pla =Cmnf.euc2plan(vec(1),vec(2),vec(3));
%=atan2(z,y);
%beta=-atan2(z,x);
%gamma=atan2(y,z);
rvec=Cmnf.rot3D(0,-pla(3),pla(2),[n;0;0]);
comp=norm(rvec-vec)

raster=[];
for y=-10:0.5:10
    for z=-10:0.5:10
        raster=[raster,[n;y;z]];
    end
end

rraster=Cmnf.align3Dvec(vec,raster);
figure(1)
hold on
grid on 
plot3(rraster(1,:),rraster(2,:),rraster(3,:),'ob')
plot3(rvec(1),rvec(2),rvec(3),'*r')
hold off