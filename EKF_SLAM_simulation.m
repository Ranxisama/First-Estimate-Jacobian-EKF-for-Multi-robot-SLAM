clc
close all

load('Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
load('Measurements.mat','R1Xp0','R1Odo','R1Obs','R2Xp0','R2Odo','R2Obs')

Config;

for k = 0:size(R1Odo,1)/3

    % k
    

    R1Obs_k = R1Obs(R1Obs(:,1)==k,2:3);
    R2Obs_k = R2Obs(R2Obs(:,1)==k,2:3);

    if k == 0
        %% estimate observed feature's state of R1 at step 0 using the observation model
        R1Xfk = R1Obs_k;
        R1Xfk(1:2:(end-1),2) = R1Xp0(1,1) + cos(R1Xp0(3,1))*(R1Obs_k(1:2:(end-1),2)) - sin(R1Xp0(3,1))*(R1Obs_k(2:2:end,2));
        R1Xfk(2:2:end,2) = R1Xp0(2,1) + sin(R1Xp0(3,1))*(R1Obs_k(1:2:(end-1),2)) + cos(R1Xp0(3,1))*(R1Obs_k(2:2:end,2));
        
        % XfTrueAll

        R1deltaFX0 = sparse(3+size(R1Xfk,1),3+size(R1Xfk,1));
        
        R1deltaFX0(1:3,1:3) = eye(3);

        R1deltaFX0(4:2:(end-1),1:3) = [repmat([1,0],size(R1Xfk,1)/2,1), ...
            -sin(R1Xp0(3))*R1Obs_k(1:2:(end-1),2) - cos(R1Xp0(3))*R1Obs_k(2:2:end,2)];

        R1deltaFX0(5:2:end,1:3) = [repmat([0,1],size(R1Xfk,1)/2,1), ...
            cos(R1Xp0(3))*R1Obs_k(1:2:(end-1),2) - sin(R1Xp0(3))*R1Obs_k(2:2:end,2)];
        
        % covariance matrix of observed features at step 0
        R1Rn = [];
        for R1j = 1:size(R1Xfk)/2
            R1Rn = blkdiag(R1Rn, R1R);
            R1deltaFX0(3+(R1j-1)*2+(1:2),3+(R1j-1)*2+(1:2)) = [cos(R1Xp0(3,1)), -sin(R1Xp0(3,1));
                sin(R1Xp0(3,1)), cos(R1Xp0(3,1))];
        end

        R1Pk0 = blkdiag(R1O,R1Rn);

        R1Xk00e = [ones(3,1),zeros(3,1),R1Xp0;
            2*ones(size(R1Xfk,1),1),R1Xfk];
        R1Pk00 = R1deltaFX0 * R1Pk0 * R1deltaFX0';

       

        % find the shared observed feature IDs in 2nd robot
        % R2Zks_lv: logical vector of shared feature observation of
        % 2nd robot at step k
        R2Zks_lv = ismember(R2Obs_k(:,1), R1Xfk(:,1));
        % R2Zks_idx: index of shared feature observation of
        % 2nd robot in R2Obs_k
        R2Zks_idx = find(R2Zks_lv);
        % R2Zks: shared feature observation of 2nd robot
        R2Zks = R2Obs_k(R2Zks_idx,:);

        R1Xfks_lv = ismember(R1Xfk(:,1), R2Obs_k(:,1));
        R1Xfks_idx = find(R1Xfks_lv);
        R1Xfks = R1Xfk(R1Xfks_idx,:);

        R1Xfkn_lv = ~R1Xfks_lv;
        R1Xfkn_idx = find(R1Xfkn_lv);
        R1Pfk00 = R1Pk00(4:end,4:end);
        R1Pfk00s = R1Pfk00;
        R1Pfk00s(R1Xfkn_idx',:) = [];
        R1Pfk00s(:,R1Xfkn_idx') = [];

        %% use Gauss-Newton iteration to optimize the 2nd robot pose at step 0
        % use the true value Xr0_true_R2 as the initial value of Xr0_R2 in the GN iteration
        [R2Xp0Gni,XfkGni,R2P0Gni] = GNI(R2Zks,R2Xp0,R1Xfks,R1Pfk00s,CC);
      
        
        
        %% estimate new observed feature's state of R2 at step 0 using the observation model
        % find the new observed feature IDs in 2nd robot
        % R2Zkn_lv: logical vector of new feature observation of
        % 2nd robot at step k
        R2Zkn_lv = ~R2Zks_lv;
        % R2Zkn_idx: index of new feature observation of
        % 2nd robot in R2Obs_k
        R2Zkn_idx = find(R2Zkn_lv);
        % R2Zkn: new feature observation of 2nd robot
        R2Zkn = R2Obs_k(R2Zkn_idx,:);
        
        R2Xfkn = R2Zkn;
        R2Xfkn(1:2:(end-1),2) = R2Xp0Gni(1,1) + cos(R2Xp0Gni(3,1))*(R2Zkn(1:2:(end-1),2)) - sin(R2Xp0Gni(3,1))*(R2Zkn(2:2:end,2));
        R2Xfkn(2:2:end,2) = R2Xp0Gni(2,1) + sin(R2Xp0Gni(3,1))*(R2Zkn(1:2:(end-1),2)) + cos(R2Xp0Gni(3,1))*(R2Zkn(2:2:end,2));
        
        % XfTrueAll

        R2deltaFX0 = sparse(3+size(R2Xfkn,1),3+size(R2Xfkn,1));
        R2deltaFX0(1:3,1:3) = eye(3);
        R2deltaFX0(4:2:(end-1),1:3) = [repmat([1,0],size(R2Xfkn,1)/2,1), -sin(R2Xp0Gni(3,1))*R2Zkn(1:2:(end-1),2) - cos(R2Xp0Gni(3,1))*R2Zkn(2:2:end,2)];
        R2deltaFX0(5:2:end,1:3) = [repmat([0,1],size(R2Xfkn,1)/2,1), cos(R2Xp0Gni(3,1))*R2Zkn(1:2:(end-1),2) - sin(R2Xp0Gni(3,1))*R2Zkn(2:2:end,2)];
        
        % covariance matrix of observed features at step 0
        R2Rn = [];
        for R2jn = 1:size(R2Xfkn)/2
            R2Rn = blkdiag(R2Rn, R2R);
            R2deltaFX0(3+(R2jn-1)*2+(1:2),3+(R2jn-1)*2+(1:2)) = [cos(R2Xp0Gni(3,1)), -sin(R2Xp0Gni(3,1));
                sin(R2Xp0Gni(3,1)), cos(R2Xp0Gni(3,1))];
        end

        R2Pk0 = blkdiag(R2Pp0Gni,R2Rn);

        R2Xk00e = [ones(3,1),zeros(3,1),R2Xp0Gni;
            2*ones(size(R2Xfkn,1),1),R2Xfkn];
        R2Pk00 = R2deltaFX0 * R2Pk0 * R2deltaFX0';
        

        %% combined R1Xk00e and Xk00e_R2n as the feature positions observed by all robots of the state
        % vector
        Xk00_e =  [R1Xk00e(1:3,:);
            ones(3,1) zeros(3,1) R2Xp0Gni;
            R1Xk00e(4:end,:);
            R2Xk00e(4:end,:)];

        % Xk00_e:
        % first column: robot id;
        % second column: 1 -> robot posture; 2 -> feature position
        % third column: posture id or position id
        % fouth column: data


        Pk00 = blkdiag(R1Pk00(1:3,1:3),R2Pk00(1:3,1:3),R1Pk00(4:end,4:end),R2Pk00(4:end,4:end));
        Pk00(1:3,6+(1:(size(R1Pk00,1)-3))) = R1Pk00(1:3,4:end);
        Pk00(6+(1:(size(R1Pk00,1)-3)),1:3) = R1Pk00(4:end,1:3);
        Pk00(4:6,3+size(R1Pk00,1)+(1:(size(R2Pk00,1)-3))) = R2Pk00(1:3,4:end);
        Pk00(3+size(R1Pk00,1)+(1:(size(R2Pk00,1)-3)),4:6) = R2Pk00(4:end,1:3);
        
    
        XrR1_full = Xk00_e(1:3,2:3); % save all robot postures of R1
        XrR2_full = Xk00_e(4:6,2:3); % save all robot postures of R2
        continue
    end

    %% Prediction using the motion model
    Uk_R1 = U_R1(U_R1(:,2)==k,3);
    Uk_R2 = U_R2(U_R2(:,2)==k,3);
    
    Xk00_e([3,6],3) = wrap(Xk00_e([3,6],3));
    Xk10_e = Xk00_e;
    Xk10_e(1:6,2) = Xk10_e(1:6,2)+1;
    Xk10_e(1:3,3) = Xk00_e(1:3,3) + ...
        [cos(Xk00_e(3,3))*Uk_R1(1) - sin(Xk00_e(3,3))*Uk_R1(2);
         sin(Xk00_e(3,3))*Uk_R1(1) + cos(Xk00_e(3,3))*Uk_R1(2);
         Uk_R1(3)];
    Xk10_e(4:6,3) = Xk00_e(4:6,3) + ...
        [cos(Xk00_e(6,3))*Uk_R2(1) - sin(Xk00_e(6,3))*Uk_R2(2);
         sin(Xk00_e(6,3))*Uk_R2(1) + cos(Xk00_e(6,3))*Uk_R2(2);
         Uk_R2(3)]; 

    Xk10_e([3,6],3) = wrap(Xk10_e([3,6],3));

    DeltaFX = sparse(size(Xk10_e,1),size(Xk10_e,1));
    DeltaFX(1:6,1:6) = blkdiag([1, 0, -sin(Xk00_e(3,3))*Uk_R1(1) - cos(Xk00_e(3,3))*Uk_R1(2); ...
        0, 1, cos(Xk00_e(3,3))*Uk_R1(1) - sin(Xk00_e(3,3))*Uk_R1(2); ...
        0, 0, 1], ...
        [1, 0, -sin(Xk00_e(6,3))*Uk_R2(1) - cos(Xk00_e(6,3))*Uk_R2(2); ...
        0, 1, cos(Xk00_e(6,3))*Uk_R2(1) - sin(Xk00_e(6,3))*Uk_R2(2); ...
        0, 0, 1]);
    DeltaFX(7:end,7:end) = eye(size(DeltaFX(7:end,7:end)));

    DeltaFW = sparse(size(Xk10_e,1),6);
    DeltaFW(1:6,1:6) = blkdiag([cos(Xk00_e(3,3)), -sin(Xk00_e(3,3)), 0; ...
        sin(Xk00_e(3,3)), cos(Xk00_e(3,3)), 0; ...
        0, 0, 1], ...
        [cos(Xk00_e(6,3)), -sin(Xk00_e(6,3)), 0; ...
        sin(Xk00_e(6,3)), cos(Xk00_e(6,3)), 0; ...
        0, 0, 1]);
    
    Pk10 = DeltaFX * Pk00 * DeltaFX' + DeltaFW * DWk * DeltaFW';

    % find the feature observations in R1 from new features of Xk10_e
    ZkR1n_logvec = ~ismember(R1Obs_k(:,1),Xk10_e(7:end,2));
    ZkR1n = R1Obs_k(find(ZkR1n_logvec),:);
    
    % find the feature observations in R2 from shared features of Xk10_e
    R2Zks_lv = ismember(R2Obs_k(:,1),Xk10_e(7:end,2));
    R2Zks = R2Obs_k(find(R2Zks_lv),:);
    Xfk10eR2s_logvec = ismember(Xk10_e(7:end,2), R2Obs_k(:,1));
    Xfk10eR2s_index = find(Xfk10eR2s_logvec)+6;
    Xfk10eR2s = Xk10_e(Xfk10eR2s_index,:);

    %% Feature initialization using feature observations from R1
    if ~isempty(ZkR1n)
        Xfk10e_n = ZkR1n;
        Xfk10e_n(1:2:end, 2) = Xk10_e(1,3) + cos(Xk10_e(3,3))*ZkR1n(1:2:end, 2) - sin(Xk10_e(3,3))*ZkR1n(2:2:end, 2);
        Xfk10e_n(2:2:end, 2) = Xk10_e(2,3) + sin(Xk10_e(3,3))*ZkR1n(1:2:end, 2) + cos(Xk10_e(3,3))*ZkR1n(2:2:end, 2);


        Xk10_eS = [Xk10_e;
            2*ones(size(Xfk10e_n,1),1), Xfk10e_n];

        DeltaGX = sparse(size(Xk10_eS,1),size(Xk10_e,1));
        DeltaGX(1:size(Xk10_e,1),1:size(Xk10_e,1)) = eye(size(Xk10_e,1));
        DeltaGX((size(Xk10_e,1)+1):2:end,1:3) = [repmat([1, 0],size(Xfk10e_n,1)/2,1),-sin(Xk10_e(3,3))*ZkR1n(1:2:end, 2)-cos(Xk10_e(3,3))*ZkR1n(2:2:end, 2)];
        DeltaGX((size(Xk10_e,1)+2):2:end,1:3) = [repmat([0, 1],size(Xfk10e_n,1)/2,1),cos(Xk10_e(3,3))*ZkR1n(1:2:end, 2)-sin(Xk10_e(3,3))*ZkR1n(2:2:end, 2)];

        DeltaGV = sparse(size(Xk10_eS,1),size(Xfk10e_n,1));
        DV1_S = [];
        for fken = 1:size(Xfk10e_n,1)/2
            DV1_S = blkdiag(DV1_S,DVk_R1);
            DeltaGV(size(Xk10_e,1)+(fken-1)*2+(1:2),(fken-1)*2+(1:2)) = [cos(Xk10_e(3,3)), -sin(Xk10_e(3,3));sin(Xk10_e(3,3)), cos(Xk10_e(3,3))];
        end
        Pk10_s = DeltaGX*Pk10*DeltaGX'+DeltaGV*DV1_S*DeltaGV';
    else
        Xk10_eS = Xk10_e;
        Pk10_s = Pk10;
    end

    %% Update using feature observations from R2
    % Xk10_e = [Xr_R1;Xr_R2;Xf_R1];
    if ~isempty(R2Zks)
    HX10e_R2 = sparse(size(R2Zks,1), 1);
    HX10e_R2(1:2:end,1) = cos(Xk10_eS(6,3))*(Xfk10eR2s(1:2:end,3)-Xk10_eS(4,3))+sin(Xk10_eS(6,3))*(Xfk10eR2s(2:2:end,3)-Xk10_eS(5,3));
    HX10e_R2(2:2:end,1) = -sin(Xk10_eS(6,3))*(Xfk10eR2s(1:2:end,3)-Xk10_eS(4,3))+cos(Xk10_eS(6,3))*(Xfk10eR2s(2:2:end,3)-Xk10_eS(5,3));

    DeltaHX10e_R2 = sparse(size(R2Zks,1), size(Xk10_eS,1));
    DeltaHX10e_R2(1:2:end,4:6) = [repmat([-cos(Xk10_eS(6,3)),-sin(Xk10_eS(6,3))],size(DeltaHX10e_R2,1)/2,1), -sin(Xk10_eS(6,3))*(Xfk10eR2s(1:2:end,3)-Xk10_eS(4,3))+cos(Xk10_eS(6,3))*(Xfk10eR2s(2:2:end,3)-Xk10_eS(5,3))];
    DeltaHX10e_R2(2:2:end,4:6) = [repmat([sin(Xk10_eS(6,3)),-cos(Xk10_eS(6,3))],size(DeltaHX10e_R2,1)/2,1), -cos(Xk10_eS(6,3))*(Xfk10eR2s(1:2:end,3)-Xk10_eS(4,3))-sin(Xk10_eS(6,3))*(Xfk10eR2s(2:2:end,3)-Xk10_eS(5,3))];
   
    for fkr2 = 1:size(R2Zks,1)/2
        DeltaHX10e_R2((fkr2-1)*2+(1:2), Xfk10eR2s_index((fkr2-1)*2+(1:2))) = [cos(Xk10_eS(6,3)), sin(Xk10_eS(6,3));-sin(Xk10_eS(6,3)),cos(Xk10_eS(6,3))];
        DV_R2 = blkdiag(DV_R2,DVk_R2);
    end

    % Innovation Covariance S and Kalman Gain K
    S = DeltaHX10e_R2 * Pk10_s * DeltaHX10e_R2' + DV_R2;
    K = Pk10_s * DeltaHX10e_R2' /S;

    % Updating process using observation model
    Xk11_e = Xk10_eS;
    Xk11_e(:,3) = Xk10_eS(:,3) + K*(R2Zks(:,2)-HX10e_R2);
    Pk11 = Pk10_s - K*S*K';
    else
        Xk11_e = Xk10_eS;
        Pk11 = Pk10_s;
    end
    
    Xk11_e([3,6],3) = wrap(Xk11_e([3,6],3));

    XrR1_full = [XrR1_full;Xk11_e(1:3,2:3)];
    XrR2_full = [XrR2_full;Xk11_e(4:6,2:3)];

    Xk00_e = Xk11_e;
    Pk00 = Pk11;

end

figure

hold on 

plot(XrR1_full(1:3:end,2),XrR1_full(2:3:end,2),'-ro');
plot(XrR2_full(1:3:end,2),XrR2_full(2:3:end,2),'--mo');

plot(Xk11_e(7:2:end,3),Xk11_e(8:2:end,3),'^','Color', [0.6, 0.4, 0.2])
text(Xk11_e(7:2:end,3),Xk11_e(8:2:end,3), num2str(Xk11_e(7:2:end,2)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.6, 0.4, 0.2])

xlim([fea_xlb-Dfr_threshold, fea_xub+Dfr_threshold])
ylim([fea_ylb-Dfr_threshold, fea_yub+Dfr_threshold])

XftrueS_logvec = ismember(Xf_true(:,1),Xk11_e(7:2:end,2));
XftrueS_idx = find(XftrueS_logvec);
Xftrue_S = Xf_true(XftrueS_idx,:);

plot(Xp_true_R1(:,2),Xp_true_R1(:,3),'-bo')
plot(Xp_true_R2(:,2),Xp_true_R2(:,3),'--co')

% plot(Xf_true(:,2),Xf_true(:,3),'k^')
plot(Xftrue_S(:,2),Xftrue_S(:,3),'k^')
text(Xftrue_S(:,2),Xftrue_S(:,3), num2str(Xftrue_S(:,1)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k')

legend('Estimated robot postures of R1','Estimated robot postures of R2','Estimated feature positions' , 'True robot postures of R1', 'True robot postures of R2', 'True feature positions')
xlabel('x')
ylabel('y')
title('True X vs Multi-robots EKF')

axis equal
ax = gca;

% 设置坐标轴属性，使其穿过原点
ax.XAxisLocation = 'origin'; % x 轴穿过原点
ax.YAxisLocation = 'origin'; % y 轴穿过原点
hold off