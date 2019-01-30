%Common function based on proceedings from:
%http://sections.maa.org/lams/proceedings/spring2001/bard.himel.pdf
p1=[20;0;0];
p2=[0;20;0];
v1=[0;1;0];
v2=[1;0;0];
[collisionFlag,mdest,time]=Cmnf.calculateLinearClash(p1,v1,p2,v2)
