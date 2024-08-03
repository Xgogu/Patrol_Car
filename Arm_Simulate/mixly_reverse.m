clear;clc;

L_1 = 100;
L_2 = 105;
L_3 = 98;
L_4 = 245;                   % ����ĩ��ִ���������޸�L4

% poi_num = 11;                % �岹����
% p_1=[250 0 250];             % ֱ�߲岹���
% p_3=[250 0 100];             % ֱ�߲岹�յ�
% inter_poi = line_interpolation(p_1,p_3,poi_num);  % ���ֱ�߲岹��

poi_num = 10;              % �岹����
p_1=[250 0 300];           % Բ���岹��1
p_2=[250 0 100];           % Բ���岹��2
p_3=[250 -100 200];        % Բ���岹��3
inter_poi = chabu(p_1,p_2,p_3,poi_num); % ���Բ���岹��

ang_mat = zeros(poi_num,4);              % ang_mat �д���ĸ�����Ƕȵ��������ֵ
ang_matR = zeros(poi_num,4);             % ang_matR �д���ĸ�����Ƕȵ�mixly����ֵ��������ƫ��

for poi = 1:poi_num
    
    px = inter_poi(1,poi);
    py = inter_poi(2,poi);
    pz = inter_poi(3,poi);

    if px < 0
        disp('��е�ṹ���ƣ��޽�');
    else
% 	��3�Ŷ����ˮƽ�Ƕ������Ϊ��ѽǶȣ��������ؽڽǶ�
        flag = 0;
        for alpha_i=0:-1:-135
            angsol_i = RK_analysis(px,py,pz,alpha_i,L_1,L_2,L_3,L_4);
            if angsol_i(5) == 0
                Alpha = alpha_i;
                flag = 1;
            end
        end
% 	����������ֵ���������
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
                disp('�޽�');
        end  
    end
    
end

function   T = RK_analysis(x,y,z,alpha,L1,L2,L3,L4)

% 	���theta1
    if y==0
        theta1 = 0;
    else
        theta1=atan(y/x)*180/pi;
    end

    ya = sqrt(x^2+y^2);
    yL = ya-L4*cos(alpha*pi/180);
    zL = z-L1-L4*sin(alpha*pi/180);
% 	���λ�õ��ڻ������޽�
    if zL < -L1
        T = [0,0,0,0,1];
        return;
    end

% 	�󲿣��ؽ�4��λ��Զ�ڱ�L2��L3���ȣ��޽�
    if (sqrt(yL^2+zL^2)) > (L2+L3)
        T = [0,0,0,0,2];                                                                                                                                                                                                                                                                                                                                                                                                                     
        return;
    end

    phi1 = acos(yL/(sqrt(yL^2+zL^2)));
% 	���Ҷ����޷�������޽�
    if (yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)) < -1 || (yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)) > 1
        T = [0,0,0,0,3];
        return;
    end
    phi2 = acos((yL^2+zL^2+L2^2-L3^2)/(2*L2*sqrt(yL^2+zL^2)));
% 	ͨ����λ�øߵͣ�ȷ��L2��L3λ��
    if zL < 0
        zf_flag = -1;
    else
        zf_flag = 1;
    end
% 	ͨ��L2��L3λ�����theta2
    theta2 = (zf_flag*phi1 + phi2)*180/pi;
% 	theta2������е��Χ���޽�
    if theta2 > 180 || theta2 < 0
        T = [0,0,0,0,4];
        return;
    end

% 	���Ҷ����޷�������޽�
    if -(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3)>1 || -(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3)<-1
        T = [0,0,0,0,5];
        return;
    end
% 	���theta3
    theta3 = acos(-(yL^2+zL^2-L2^2-L3^2)/(2*L2*L3));
    theta3 = 180-(theta3)*180/pi;
% 	theta3������е��Χ���޽�
    if theta3 > 135 || theta3 < -135
        T = [0,0,0,0,6];
        return;
    end

% 	���theta4
    theta4 = alpha - theta2 + theta3;
% 	theta4������е��Χ���޽�
    if theta4 > 90 || theta4 < -90
        T = [0,0,0,0,7];
        return;
    end

    T = [theta1,theta2-90,theta3,theta4,0];
end