% 修改下列量即可
% Trans是图案到相机的平移量
% val(:,:,i)是图案到相机的旋转矩阵
% JacoCartesianPose是机器人末端到基座的xyz四元数

% 需要安装机器人工具箱，RTB

clear;
close all;
clc;

%% 数据导入

% pattern pos
Trans=[   -5.8649   -7.3031  114.4374;
   -2.0182  -11.8461  117.1054;
  -5.6343   -3.8656  115.2085;
 -10.7174    0.8083  109.2783;
   -1.9600   -4.7108  111.3949;
%%%%%%%%%%%%%% 6.bmp missing
  -13.8778  -11.4548  117.7682;
    1.0698   -1.6015  120.1493;
   -7.5080   -8.0566  121.2449;
    4.9560   -7.9590  120.7114;
  -12.6271   -3.2471  112.9673;
   -5.4085   -7.6925  108.0745;
  -15.4373   -1.2118  114.3455;
%%%%%%%%%%%%%%14 .bmp missing
    7.2818  -18.2655  118.2794;];

val(:,:,1)= [    1.0000    0.0070    0.0000;
   -0.0070    1.0000    0.0003;
   -0.0000   -0.0003    1.0000;
];
val(:,:,2)= [    0.9408    0.1229   -0.3160;
   -0.0058    0.9377    0.3475;
    0.3390   -0.3251    0.8828;
];
val(:,:,3)= [    0.9396    0.0079   -0.3423;
   -0.0062    1.0000    0.0060;
    0.3423   -0.0035    0.9396;
];
val(:,:,4)= [    0.9056   -0.2708   -0.3264;
    0.1566    0.9287   -0.3361;
    0.3942    0.2533    0.8835;
];
val(:,:,5)= [    0.9870    0.1605   -0.0058;
   -0.1528    0.9275   -0.3412;
   -0.0493    0.3377    0.9400;
];
%%%%%%%%%%%%%% 6.bmp missing
val(:,:,6)= [    0.9267   -0.1550    0.3423;
    0.1666    0.9860   -0.0044;
   -0.3368    0.0611    0.9396;
];
val(:,:,7)= [    0.9054   -0.2720    0.3260;
    0.1570    0.9279    0.3380;
   -0.3944   -0.2549    0.8829;
];
val(:,:,8)= [    1.0000    0.0073    0.0000;
   -0.0073    1.0000    0.0007;
   -0.0000   -0.0007    1.0000;
];
val(:,:,9)= [    0.9895    0.1001   -0.1042;
   -0.1128    0.9859   -0.1238;
    0.0903    0.1342    0.9868;
];
val(:,:,10)= [    0.8638    0.0932   -0.4951;
   -0.1987    0.9661   -0.1647;
    0.4630    0.2407    0.8531;
];
val(:,:,11)= [    0.9181   -0.3211   -0.2322;
    0.1721    0.8510   -0.4961;
    0.3569    0.4155    0.8366;
];
val(:,:,12)= [    0.9587   -0.2069    0.1953;
    0.2390    0.9580   -0.1585;
   -0.1543    0.1986    0.9679;
];
%%%%%%%%%%%%%% 14.bmp missing
val(:,:,13)= [    0.7494   -0.5675    0.3411;
    0.4835    0.8210    0.3036;
   -0.4523   -0.0626    0.8897;
];

% Robot pose in Quatenion xyzw
 JacoCartesianPose =[
        -279.20 -551.02 231.32  0.0001    1.0000    0.0000   -0.0000
        -220.84 -618.71 178.39  0.1710   -0.9698    0.0302    0.1710
        -282.54 -614.81 222.45  0.1737   -0.9848    0.0001   -0.0000
        -342.01 -602.94 237.77  0.1852   -0.9636   -0.1145   -0.1554
        -349.37 -558.71 248.99  0.0135    0.9819   -0.0760    0.1730
        % -322.55 -497.11 237.71  0.1987    0.9490   -0.2024    0.1378  %%%%%%%%%%%%%% 6.bmp missing
        -289.29 -464.02 219.19  0.1729    0.9811    0.0858    0.0151
        -242.13 -468.70 185.25  0.1852    0.9635    0.1146   -0.1554
        -278.28 -549.53 238.15  0.0000   -1.0000         0         0
        -306.15 -588.96 246.49  0.0483   -0.9954    0.0499   -0.0654
        -329.84 -647.30 224.18  0.2483   -0.9598    0.0725   -0.1092
        -368.67 -597.53 244.26  0.1522   -0.9489   -0.1337   -0.2420
        -328.10 -491.63 236.44  0.0895    0.9850    0.1169    0.0895
        % -323.66 -445.95 215.09  0.2385    0.9556    0.1552    0.0770  %%%%%%%%%%%%%% 14.bmp missing
        -270.07 -425.93 184.98  0.2118    0.9291    0.2859   -0.1007
 ];

[m,n] = size(JacoCartesianPose); % dim

% Robot Pose stored in cell A .
A = cell(1,m); % 1*9
for i = 1: m
    robotHtool_qua =  q2R([ JacoCartesianPose(i, 4), JacoCartesianPose(i, 5), JacoCartesianPose(i, 6) , JacoCartesianPose(i, 7)]) ; % !! 7 4 5 6
    A{1,i}  = transl(JacoCartesianPose(i, 1), JacoCartesianPose(i, 2), JacoCartesianPose(i, 3)) *  robotHtool_qua;
end
 
% Pattern Pose(Homogeneous) stored in  cell B.
patternInCamPose=zeros(m,7); %相机 !!
for i=1:m  % !!
    V=(val(:,:,i));
    q=rotm2quat(V);
    patternInCamPose(i,1:3)=Trans(i,:);
    patternInCamPose(i,4:7)=q; 
end
[melem,nelem] = size(patternInCamPose); % 8*7
B=cell(1,melem);
for x=1: melem
    camHmarker_qua = q2R([ patternInCamPose(x, 4) , patternInCamPose(x, 5), patternInCamPose(x, 6) , patternInCamPose(x, 7)])    ;
    B{1,x} = transl(patternInCamPose(x, 1), patternInCamPose(x, 2), patternInCamPose(x, 3)) *  camHmarker_qua;
end

% 转换成手眼标定的格式
n2=m;
TA=cell(1,n2);
TB=cell(1,n2); 
M1=[];
M2=[];
 for j=[1: m-1]% 1-7.
     TA{1, j} = inv(A{1, j}) * A{1, j+1};    
     M1=[M1 TA{1, j}];
     TB{1, j} = B{1, j} * inv(B{1, j+1});
     M2=[M2 TB{1, j}];
 end 
 

%% 标定：相机外参
C_Tsai=tsai(M1, M2);
T_Tsai =  (transl(C_Tsai))';
C_Tsai_rad = tr2rpy(C_Tsai);
C_Tsai_rpy_rx_ry_rz =rad2deg(C_Tsai_rad);
Q_Tsai_Qxyzw=quaternion(C_Tsai);
fprintf('Tsai(rad) = \n%f, %f, %f, %f, %f, %f\n',T_Tsai(1,1), T_Tsai(1,2), T_Tsai(1,3), C_Tsai_rad(1,1), C_Tsai_rad(1,2), C_Tsai_rad(1,3));
fprintf('Tsai(deg) = \n%f, %f, %f, %f, %f, %f\n\n',T_Tsai(1,1), T_Tsai(1,2), T_Tsai(1,3), C_Tsai_rpy_rx_ry_rz(1,1), C_Tsai_rpy_rx_ry_rz(1,2), C_Tsai_rpy_rx_ry_rz(1,3));
% fprintf('Tsai(Qxyzw) = \n %f, %f, %f, %f\n\n',  Q_Tsai_Qxyzw.v(1),  Q_Tsai_Qxyzw.v(2), Q_Tsai_Qxyzw.v(3), Q_Tsai_Qxyzw.s);


%%
function R=q2R(q)
R=[ 2*q(1).^2-1+2*q(2)^2    2*(q(2)*q(3)-q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3)) 0;
    2*(q(2)*q(3)+q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2     2*(q(3)*q(4)-q(1)*q(2)) 0;
    2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2 0;
    0 0 0 1] ;
end

function X = tsai(A,B)
% Calculates the least squares solution of
% AX = XB
% 
% A New Technique for Fully Autonomous 
% and Efficient 3D Robotics Hand/Eye Calibration
% Lenz Tsai
%
% Mili Shah
% July 2014

[m,n] = size(A); n = n/4;
S = zeros(3*n,3);
v = zeros(3*n,1);
%Calculate best rotation R
for i = 1:n
    A1 = logm(A(1:3,4*i-3:4*i-1)); 
    B1 = logm(B(1:3,4*i-3:4*i-1));
    a = [A1(3,2) A1(1,3) A1(2,1)]'; a = a/norm(a);
    b = [B1(3,2) B1(1,3) B1(2,1)]'; b = b/norm(b);
    S(3*i-2:3*i,:) = skew(a+b);
    v(3*i-2:3*i,:) = a-b;
end
x = S\v;
theta = 2*atan(norm(x));
x = x/norm(x);
R = (eye(3)*cos(theta) + sin(theta)*skew(x) + (1-cos(theta))*x*x')';
%Calculate best translation t
C = zeros(3*n,3);
d = zeros(3*n,1);
I = eye(3);
for i = 1:n
    C(3*i-2:3*i,:) = I - A(1:3,4*i-3:4*i-1);
    d(3*i-2:3*i,:) = A(1:3,4*i)-R*B(1:3,4*i);
end
t = C\d;
%Put everything together to form X
X = [R t;0 0 0 1];
end

%%
function X = shiu(A,B)
% Calculates the least squares solution of
% AX = XB
% From
% Calibration of Wrist-Mounted Robotic Sensors 
% by Solving Homogeneous Transform Equations of the Form AX = XB
% Shiu and Ahmad
%
% Mili Shah
% July 2014

[m,n] = size(A); n = n/4;
AA = zeros(9*(n-1),2*n);
bb = zeros(9*(n-1),1);
%Calculate best rotation R
for i = 1:n
    A1 = logm(A(1:3,4*i-3:4*i-1));
    B1 = logm(B(1:3,4*i-3:4*i-1));
    a1 = [A1(3,2) A1(1,3) A1(2,1)]'; a1 = a1/norm(a1);
    b1 = [B1(3,2) B1(1,3) B1(2,1)]'; b1 = b1/norm(b1);
    v = cross(b1,a1);
    w = atan2(norm(v),b1'*a1);
    v = v/norm(v);
    XP = (eye(3)*cos(w) + sin(w)*skew(v) + (1-cos(w))*v*v');
    [Ai,bi] = shiuMatrix(a1,XP);
    if i == 1
        AA(:,1:2) = repmat(-Ai,n-1,1);
        bb(:,1) = repmat(-bi,n-1,1);
    else
        AA(9*(i-2)+1:9*(i-1),2*i-1:2*i) = Ai;
        bb(9*(i-2)+1:9*(i-1),1) = bb(9*(i-2)+1:9*(i-1),1) + bi;
    end
end
beta = AA\bb;
theta = atan2(beta(2*n),beta(2*n-1));
RA = (eye(3)*cos(theta) + sin(theta)*skew(a1) + (1-cos(theta))*a1*a1');
R = RA*XP;
%Calculate best translation t
C = zeros(3*n,3);
d = zeros(3*n,1);
I = eye(3);
for i = 1:n
    C(3*i-2:3*i,:) = I - A(1:3,4*i-3:4*i-1);
    d(3*i-2:3*i,:) = A(1:3,4*i)-R*B(1:3,4*i);
end
t = C\d;
%Put everything together to form X
X = [R t;0 0 0 1];
end

function [A,b] = shiuMatrix(ka1,X)
A = zeros(9,2);
b = zeros(9,1);

A(1,1) = X(1,1)-ka1(1)*X(:,1)'*ka1;
A(2,1) = X(1,2)-ka1(1)*X(:,2)'*ka1;
A(3,1) = X(1,3)-ka1(1)*X(:,3)'*ka1;

A(4,1) = X(2,1)-ka1(2)*X(:,1)'*ka1;
A(5,1) = X(2,2)-ka1(2)*X(:,2)'*ka1;
A(6,1) = X(2,3)-ka1(2)*X(:,3)'*ka1;

A(7,1) = X(3,1)-ka1(3)*X(:,1)'*ka1;
A(8,1) = X(3,2)-ka1(3)*X(:,2)'*ka1;
A(9,1) = X(3,3)-ka1(3)*X(:,3)'*ka1;

n = cross(X(:,1),ka1);
o = cross(X(:,2),ka1);
a = cross(X(:,3),ka1);

A(1,2) = -n(1);
A(2,2) = -o(1);
A(3,2) = -a(1);

A(4,2) = -n(2);
A(5,2) = -o(2);
A(6,2) = -a(2);

A(7,2) = -n(3);
A(8,2) = -o(3);
A(9,2) = -a(3);

n = X(:,1);
o = X(:,2);
a = X(:,3);

b(1) = -ka1(1)*n'*ka1;
b(2) = -ka1(1)*o'*ka1;
b(3) = -ka1(1)*a'*ka1;

b(4) = -ka1(2)*n'*ka1;
b(5) = -ka1(2)*o'*ka1;
b(6) = -ka1(2)*a'*ka1;

b(7) = -ka1(3)*n'*ka1;
b(8) = -ka1(3)*o'*ka1;
b(9) = -ka1(3)*a'*ka1;
end

%% 
function [X, Y, Z, RX, RY, RZ] = T_to_XYZ_RXRYRZ(T)
    % 提取平移部分
    X = T(1, 4);
    Y = T(2, 4);
    Z = T(3, 4);

    % 提取旋转矩阵部分
    R = T(1:3, 1:3);

    % 将旋转矩阵转为欧拉角 [RX, RY, RZ]，使用 'ZYX' 顺序（Yaw-Pitch-Roll）
    eul = rotm2eul(R, 'ZYX');

    % 提取欧拉角：Yaw (RZ), Pitch (RY), Roll (RX)
    RX = rad2deg(eul(3));  % Roll
    RY = rad2deg(eul(2));  % Pitch
    RZ = rad2deg(eul(1));  % Yaw
end

function T = XYZ_RXRYRZ_to_T(X, Y, Z, RX, RY, RZ)
    % 构造旋转矩阵 (按 ZYX 顺序: Roll -> Pitch -> Yaw)
    eul_angles = deg2rad([RZ, RY, RX]);  % 欧拉角 [Yaw, Pitch, Roll]
    R = eul2rotm(eul_angles, 'ZYX');  % 生成旋转矩阵

    % 构造齐次变换矩阵 T
    T = eye(4);       % 初始化为单位矩阵
    T(1:3, 1:3) = R;  % 将旋转矩阵填入 T 的前 3x3 部分
    T(1:3, 4) = [X; Y; Z];  % 将平移向量填入 T 的最后一列
end
