function Hcg = TsaiHandEye(Hgij_list, Hcij_list)
    % 使用 Tsai 方法计算手眼标定矩阵 Hcg
    % 参数：
    % - Hgij_list：包含 (4, 4, n) 形状的数组，每个数组表示变换矩阵 Hgij[i]
    % - Hcij_list：包含 (4, 4, n) 形状的数组，每个数组表示变换矩阵 Hcij[i]
    % 返回：
    % - Hcg：计算得到的手眼标定矩阵，形状为 (4, 4)

    assert(size(Hgij_list, 3) == size(Hcij_list, 3), 'Hgij 和 Hcij 列表的长度必须相同。');
    nStatus = size(Hgij_list, 3);

    A_list = [];
    b_list = [];

    for i = 1:nStatus
        % 提取旋转矩阵
        Rgij = Hgij_list(1:3, 1:3, i);
        Rcij = Hcij_list(1:3, 1:3, i);

        % 将旋转矩阵转换为旋转向量
        rgij = rotm2vec(Rgij);
        rcij = rotm2vec(Rcij);

        % 计算旋转角度
        theta_gij = norm(rgij);
        theta_cij = norm(rcij);

        % 检查旋转角度是否为零
        if theta_gij == 0 || theta_cij == 0
            warning('theta_gij or theta_cij is zero at index %d. Skipping this iteration.', i);
            continue;
        end

        % 归一化旋转向量
        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;

        % 计算 Pgij 和 Pcij
        Pgij = 2 * sin(theta_gij / 2) * rngij;
        Pcij = 2 * sin(theta_cij / 2) * rncij;

        % 计算 tempA 和 tempb
        tempA = skew(Pgij + Pcij);
        tempb = (Pcij - Pgij);

        % 添加到列表
        A_list = [A_list; tempA];
        b_list = [b_list; tempb];
    end

    % 堆叠 A 和 b
    A = A_list;
    b = b_list;

    % 计算旋转部分
    pinA = pinv(A);
    Pcg_prime = pinA * b;
    norm_Pcg_prime = norm(Pcg_prime);
    Pcg = 2 * Pcg_prime / sqrt(1 + norm_Pcg_prime^2);
    PcgTrs = Pcg';

    eyeM = eye(3);
    norm_Pcg = norm(Pcg);
    Rcg = (1 - norm_Pcg^2 / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm_Pcg^2) * skew(Pcg));

    % 计算平移部分
    AA_list = [];
    bb_list = [];

    for i = 1:nStatus
        Rgij = Hgij_list(1:3, 1:3, i);
        Tgij = Hgij_list(1:3, 4, i);
        Tcij = Hcij_list(1:3, 4, i);

        tempAA = Rgij - eyeM;
        tempbb = Rcg * Tcij - Tgij;

        AA_list = [AA_list; tempAA];
        bb_list = [bb_list; tempbb];
    end

    % 堆叠 AA 和 bb
    AA = AA_list;
    bb = bb_list;

    % 计算 Tcg
    pinAA = pinv(AA);
    Tcg = pinAA * bb;

    % 构建 Hcg
    Hcg = eye(4);
    Hcg(1:3, 1:3) = Rcg;
    Hcg(1:3, 4) = Tcg;

end

function Sk = skew(v)
    % 根据一个 3 元向量创建反对称矩阵（斜对称矩阵）
    Sk = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function vec = rotm2vec(R)
    % 将旋转矩阵转换为旋转向量
    theta = acos((trace(R) - 1) / 2);
    if theta == 0
        vec = [0; 0; 0];
    else
        vec = theta / (2 * sin(theta)) * [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    end
end