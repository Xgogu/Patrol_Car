clear;clc;       
theta1 =0; 
theta2 = -90; 
theta3 = 0; 
theta4 = 0; 
theta5 = 0;


T_01 = tmat(0,0 , 100, theta1);
T_12 = tmat(-90, 0, 0, theta2);
T_23 = tmat(0, 105, 0, theta3);
T_34 = tmat(0, 98, 0, theta4);
T_45 = tmat(-90, 150, 0, theta5);
T_02 = T_01*T_12;
T_03 = T_02*T_23;
T_04 = T_03*T_34;
T_05 = T_04*T_45;
disp('转换矩阵');
disp(T_05);



p_x = T_05(1,4);
p_y = T_05(2,4);
p_z = T_05(3,4);
%角度转换
angle=pi/180;  %度
%D-H参数表
alpha1 = 0;    A1 = 0;     D1 = 0.1;  offset1 = 0;
alpha2 = -pi/2;A2 = 0;     D2 = 0;    offset2 = 0;
alpha3 = 0;    A3 = 0.105; D3 = 0;    offset3 = 0;
alpha4 = 0;    A4 = 0.098; D4 = 0;    offset4 = 0;
alpha5 = -pi/2;A5 = 0.15;  D5 = 0;    offset5 = 0;

L(1) = Link([theta1,D1,A1 , alpha1, offset1], 'modified');
L(2) = Link([theta2, D2,A2, alpha2, offset2], 'modified');
L(3) = Link([theta3, D3,A3, alpha3, offset3], 'modified');
L(4) = Link([theta4,D4, A4, alpha4, offset4], 'modified');
L(5) = Link([theta5,D5, A5, alpha5, offset5], 'modified');
disp("D-H表")
L(1).display();
L(2).display();
L(3).display();
L(4).display();
L(5).display();

% 定义关节范围
L(1).qlim =[-180*angle, 180*angle];
L(2).qlim =[-180*angle, 180*angle];
L(3).qlim =[-180*angle, 180*angle];
L(4).qlim =[-180*angle, 180*angle];
L(5).qlim =[-180*angle, 180*angle];


%% 显示机械臂
robot0 = SerialLink(L,'name','six');
f = 1 ;									%画在第1张图上
theta = [0 -pi/2 0 0 0];				%初始关节角度
figure(f);
robot0.plot(theta);
title('机械臂初始模型');
robot1 = SerialLink(L,'name','five');
f = 2;
figure(f);
robot1.plot(theta);
robot1.teach
title('可调节机械臂模型');

function T = tmat(alpha, a, d, theta)
    % tmat(alpha, a, d, theta) (T-Matrix used in Robotics)
    % The homogeneous transformation called the "T-MATRIX"
    % as used in the Kinematic Equations for robotic type
    % systems (or equivalent).
    %
    % This is equation 3.6 in Craig's "Introduction to Robotics."
    % alpha, a, d, theta are the Denavit-Hartenberg parameters.
    %
    % (NOTE: ALL ANGLES MUST BE IN DEGREES.)
    %
    alpha = alpha*pi/180;    %Note: alpha is in radians.
    theta = theta*pi/180;    %Note: theta is in radians.
    c = cos(theta);
    s = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    T = [c -s 0 a; s*ca c*ca -sa -sa*d; s*sa c*sa ca ca*d; 0 0 0 1];
end

 