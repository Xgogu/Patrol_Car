%陈勇杭 20213511
a=input('how much is a');
b=input('how much is b');
c=input('how much is c');
Rz=[cosd(a) -sind(a) 0; sind(a) cosd(a) 0;0 0 1];
Ry=[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)];
Rx=[1 0 0; 0 cosd(c) -sind(c);0 sind(c) cosd(c)];
T=Rz*Ry*Rx;
disp('T本身');
disp(T);
%求每一列向量
disp('T的每一列向量')
disp(norm(T(:,1)));
disp(norm(T(:,2)));
disp(norm(T(:,1)));
disp('T的第一列和第二列乘积')
disp(dot(T(:,1),T(:,2)))
disp('T的第二列和第三列乘积')
disp(dot(T(:,2),T(:,3)))
disp('T的第一列和第三列乘积')
disp(dot(T(:,1),T(:,3)))
T1=T'; %转置
T2=inv(T); %逆
disp('T的转置')
disp(T1)
disp('T的逆')
disp(T2)
if roundn(T1,10)==roundn(T2,10)
    disp('yes');
end
