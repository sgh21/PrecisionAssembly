extrinsics = squeeze(cameraParams.PatternExtrinsics);
num_matrices = size(extrinsics, 1);  % 获取矩阵的数量

data_dir = "../calib_data/calib_new_biaoding4" ;
flange2base_path = fullfile(data_dir, "/flange2base.npy");
T_flange2base = readNPY(flange2base_path);

T_flange2base = permute(T_flange2base, [2, 3, 1]);
% 删除指定index的元素
% delete_index = [1,2,3,4,5,6,7,8,9];
% T_flange2base(:, :, delete_index) = [];
for i = 10:size(T_flange2base, 3)
    % 将平移部分乘以 1000，将单位从米转换为毫米
    T_flange2base(1:3, 4, i) = T_flange2base(1:3, 4, i) * 1000;
end
% disp(size(T_flange2base));
T_A = zeros(4,4,num_matrices-1);
T_B = zeros(4,4,num_matrices-1);
for i = 1:num_matrices-1
    T_A(:,:,i) = squeeze(inv(T_flange2base(:,:,i+1+9)))*squeeze(T_flange2base(:,:,i+9));
    T_B(:,:,i) = squeeze(cameraParams.PatternExtrinsics(i+1, 1).A)*squeeze(inv(cameraParams.PatternExtrinsics(i, 1).A));
end

T_A_reshape = zeros(4, 4*(num_matrices-1));
T_B_reshape = zeros(4, 4*(num_matrices-1));
for i = 1:num_matrices-1
    T_A_reshape(:, 4*i-3:4*i) = T_A(:,:,i);
    T_B_reshape(:, 4*i-3:4*i) = T_B(:,:,i);
end

X = TsaiHandEye(T_A, T_B);
disp('X:');
disp(X);
X(1:3,4) = X(1:3,4) / 1000;

rpy = rotm2eul(X(1:3,1:3),"ZYX");
rpy = rpy * 180 / pi;
disp('rpy:');
disp(rpy);

% 保存标定结果
save_path = fullfile(data_dir, "/camera_config.yaml");

rad_dist = squeeze(cameraParams.Intrinsics.RadialDistortion);
tan_dist = squeeze(cameraParams.Intrinsics.TangentialDistortion);
skew = squeeze(cameraParams.Intrinsics.Skew);
coff_dist = [rad_dist, tan_dist,skew];

camera_params = struct();
camera_params.distortion_coefficients = coff_dist;
camera_params.intrinsic = cameraParams.Intrinsics.K;
camera_params.extrinsic = X;
