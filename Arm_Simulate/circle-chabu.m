function numlist=chabu(p_1,p_2,p_3,n)
% clear 
% clc
% p_1=[250 0 300];
% p_2=[250 0 100];
% p_3=[250 -100 200];

u1=p_2-p_1;
w1=cross((p_3-p_1),u1);
u=u1/sqrt(dot(u1,u1));
w=w1/sqrt(dot(w1,w1));
v=cross(w,u);

bx=dot((p_2-p_1),u);
cx=dot((p_3-p_1),u);
cy=dot((p_3-p_1),v);
h=((cx-bx/2)^2+cy^2-(bx/2)^2)/(2*cy);
p0=p_1+(bx/2)*u+h*v;

x0=p0(1);y0=p0(2);z0=p0(3);
x1=p_1(1);x2=p_2(1);x3=p_3(1);
y1=p_1(2);y2=p_2(2);y3=p_3(2);
z1=p_1(3);z2=p_2(3);z3=p_3(3);

A=(y2-y1)*(z3-z2)-(z2-z1)*(y3-y2);
B=(z2-z1)*(x3-x2)-(x2-x1)*(z3-z2);
C=(x2-x1)*(y3-y2)-(y2-y1)*(x3-x2);
k=sqrt(A^2+B^2+C^2);
ax=A/k;
ay=B/k;
az=C/k;
r=sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2);
nx=(x1-x0)/r;
ny=(y1-y0)/r;
nz=(z1-z0)/r;
ox=ay*nz-az*ny;
oy=az*nx-ax*nz;
oz=ax*ny-ay*nx;
T=[nx ox ax x0;ny oy ay y0;nz oz az z0;0 0 0 1];
T1=inv(T);
p1=T1*[x1 y1 z1 1]';
p2=T1*[x2 y2 z2 1]';
p3=T1*[x3 y3 z3 1]';
x31=p3(1); y31=p3(2);

if y31<0
    theta13=atan2(y31,x31)+2*pi;
else
    theta13=atan2(y31,x31);
end
dotsnum=zeros(n,4);
step=0;
for i=1:n
    
    d=[r*cos(step),r*sin(step),0,1];
    dotsnum(i,:)=T*d';
    step=step+theta13/(n-1);
end

dotsnum_f=dotsnum(:,1:3);
numlist=dotsnum_f';
end
