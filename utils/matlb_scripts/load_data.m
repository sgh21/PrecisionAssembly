extrinsics = squeeze(cameraParams.PatternExtrinsics);
disp('extrinsics 的维度:');
disp(size(extrinsics));

num_matrices = size(extrinsics, 1);  % 获取矩阵的数量
v_rot = zeros(num_matrices,3);
v_trans = zeros(num_matrices,3);
data_dir = "../calib_data/calib_new_biaoding_zhijia" ;
% 保存内参矩阵的路径
output_intrinsics = fullfile(data_dir, "/mat_intri.npy");
output_coff_dist = fullfile(data_dir, "/coff_dist.npy");
% 保存旋转向量和平移向量的路径
output_v_rot = fullfile(data_dir,"/v_rot.npy");
output_v_trans = fullfile(data_dir,"/v_trans.npy");
for i = 1:num_matrices
    v_trans(i,:) = squeeze(extrinsics(i,1).Translation)/1000;
    v_rot(i,:) = squeeze(cameraParams.RotationVectors(i,:));
end
% 保存内参矩阵为 .npy 文件
mat_intri = squeeze(cameraParams.Intrinsics.K);
rad_dist = squeeze(cameraParams.Intrinsics.RadialDistortion);
tan_dist = squeeze(cameraParams.Intrinsics.TangentialDistortion);
skew = squeeze(cameraParams.Intrinsics.Skew);
coff_dist = [rad_dist, tan_dist,skew];
% disp(coff_dist)
writeNPY(coff_dist, output_coff_dist);
writeNPY(mat_intri, output_intrinsics);
% 保存旋转向量和平移向量为 .npy 文件
writeNPY(v_rot, output_v_rot);
writeNPY(v_trans, output_v_trans);

% 显示保存路径
disp(['内参矩阵已保存到: ', num2str(size(mat_intri)), output_intrinsics]);
disp(['畸变矩阵已保存到: ', num2str(size(coff_dist)), output_coff_dist]);
disp(['旋转向量已保存到: ', num2str(size(v_rot)),output_v_rot]);
disp(['平移向量已保存到: ',num2str(size(v_trans)), output_v_trans]);