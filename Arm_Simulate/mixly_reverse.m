clear;clc;

L_1 = 100;
L_2 = 105;
L_3 = 98;
L_4 = 245;                   % 根据末端执行器长度修改L4

% poi_num = 11;                % 插补点数
% p_1=[250 0 250];             % 直线插补起点
% p_3=[250 0 100];             % 直线插补终点
% inter_poi = line_interpolation(p_1,p_3,poi_num);  % 求解直线插补点

poi_num = 10;              % 插补点数
p_1=[250 0 300];           % 圆弧插补点1
p_2=[250 0 100];           % 圆弧插补点2
p_3=[250 -100 200];        % 圆弧插补点3
inter_poi = chabu(p_1,p_2,p_3,poi_num); % 求解圆弧插补点

ang_mat = zeros(poi_num,4);              % ang_mat 中存放四个舵机角度的逆解理论值
ang_matR = zeros(poi_num,4);             % ang_matR 中存放四个舵机角度的mixly输入值，可能有偏差

for poi = 1:poi_num
    
    px = inter_poi(1,poi);
    py = inter_poi(2,poi);
    pz = inter_poi(3,poi);

    if px < 0
        disp('机械结构限制，无解');
    else
% 	以3号舵机与水平角度最大作为最佳角度，尝试求解关节角度
        flag = 0;
        for alpha_i=0:-1:-135
            angsol_i = RK_analysis(px,py,pz,alpha_i,L_1,L_2,L_3,L_4);
            if angsol_i(5) == 0
                Alpha = alpha_i;
                flag = 1;
            end
        end
% 	如果解出，赋值给输出矩阵
        if flag == 1
                angsol=RK_analysis(px,py,pz,Alpha,L_1,L_2,L_3,L_4);
                ang_mat(poi,1) = angsol(1);
                ang_mat(poi,2) = angsol(2);
                ang_mat(poi,3) = angsol(3);
                ang_mat(poi,4) = angsol(4);
                ang_matR(poi,1) = 90-angsol(1)*7/10;
                ang_matR(poi,2) = 90+angsol(2)*7/9+8;
                ang_matR(poi,3) = 90+angsol(3)*7/9+3;
                ang_matR(poi,4) = 90-angsol(4)*7/9;
        else
                disp('无解');
        end  
    end
    
end

function   T = RK_analysis(x,y,z,alpha,L1,L2,L3,L4)

% 	求解theta1
    if y==0
        theta1 = 0;
    else
        theta1=atan(y/x)*180/pi;
    end

    ya = sqrt(x^2+y^2);
    yL = ya-L4*cos(alpha*pi/180);
    zL = z-L1-L4*sin(alpha*pi/180);
% 	求解位置低于基座，无解
    if zL < -L1
        T = [0,0,0,0,1];
        return;
    end

% 	腕部（关节4）位置远于臂L2、L3长度，无解
    if (sqrt(yL^2+zL^2)) > (L2+L3)
        T = [0,0,0,0,2];                                                                                                                                                                                                                                                                                                                                                                                                                     
        return;
    end

    phi1 = acos(yL/(sqrt(yL^2+zL^2)));
% 	余弦定理无法解出，无解
    if (yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)) < -1 || (yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)) > 1
        T = [0,0,0,0,3];
        return;
    end
    phi2 = acos((yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)));
% 	通过腕部位置高低，确定L2、L3位姿
    if zL < 0
        zf_flag = -1;
    else
        zf_flag = 1;
    end
% 	通过L2、L3位姿求解theta2
    theta2 = (zf_flag*phi1 + phi2)*180/pi;
% 	theta2超出机械范围，无解
    if theta2 > 180 || theta2 < 0
        T = [0,0,0,0,4];
        return;
    end

% 	余弦定理无法解出，无解
    if -(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3)>1 || -(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3)<-1
        T = [0,0,0,0,5];
        return;
    end
% 	求解theta3
    theta3 = acos(-(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3));
    theta3 = 180-(theta3)*180/pi;
% 	theta3超出机械范围，无解
    if theta3 > 135 || theta3 < -135
        T = [0,0,0,0,6];
        return;
    end

% 	求解theta4
    theta4 = alpha - theta2 + theta3;
% 	theta4超出机械范围，无解
    if theta4 > 90 || theta4 < -90
        T = [0,0,0,0,7];
        return;
    end

    T = [theta1,theta2-90,theta3,theta4,0];
end