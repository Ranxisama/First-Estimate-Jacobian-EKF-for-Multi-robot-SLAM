clc
close all

for i = 1:3
    if i == 1
        load('MT_Parameters_20fea.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_20fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    elseif i == 2
        load('MT_Parameters_60fea.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_60fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    else
        load('MT_Parameters_100fea.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_100fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    end

    Config;

    poseNum = size(R1OdoSet,1)/3;

    % Standard EKF
    R1XpFullSet = [];
    R1PFullSet = [];
    R2XpFullSet = [];
    R2PFullSet = [];
    XfFullSet = [];
    PfFullSet = [];

    for mc = 1:mcNum

        R1Xp0 = R1Xp0Set(:,mc);
        R2Xp0 = R2Xp0Set(:,mc);

        R1Odo = R1OdoSet(:,[1,2,2+mc]);
        R2Odo = R2OdoSet(:,[1,2,2+mc]);

        R1Obs = R1ObsSet(:,[1,2,2+mc]);
        R2Obs = R2ObsSet(:,[1,2,2+mc]);

        % Standard EKF
        R1XpFull = [];
        R1PFull = [];
        R2XpFull = [];
        R2PFull = [];
        PfFull = [];

        Xs = [];
        X0 = [];

        % std EKF
        Xk00e = [];
        Xk10e = [];
        Xk10efi = [];
        Xk11e = [];

        for k = 0:poseNum

            % k

            % Standard & EFJ EKF & Ideal EKF
            R1Obs_k = R1Obs(R1Obs(:,1)==k,2:3);
            R2Obs_k = R2Obs(R2Obs(:,1)==k,2:3);

            if k == 0
                % find the shared observed feature IDs in 1st robot
                % R1Z0s_lv: logical vector of shared feature observation of
                % 1st robot at step 0
                R1Z0s_lv = ismember(R1Obs_k(:,1), R2Obs_k(:,1));
                % R1Z0s_idx: index of shared feature observation of
                % 1st robot in R2Obs_k
                R1Z0s_idx = find(R1Z0s_lv);
                % R1Z0s: shared feature observation of 1st robot
                R1Z0s = R1Obs_k(R1Z0s_idx,:);
                R1Xf0s = R1Z0s;
                R1Xf0s(1:2:(end-1),2) = R1Xp0(1,1) + cos(R1Xp0(3,1))*(R1Z0s(1:2:(end-1),2)) - sin(R1Xp0(3,1))*(R1Z0s(2:2:end,2));
                R1Xf0s(2:2:end,2) = R1Xp0(2,1) + sin(R1Xp0(3,1))*(R1Z0s(1:2:(end-1),2)) + cos(R1Xp0(3,1))*(R1Z0s(2:2:end,2));

                % find the shared observed feature IDs in 2nd robot
                % R2Z0s_lv: logical vector of shared feature observation of
                % 2nd robot at step k
                R2Z0s_lv = ismember(R2Obs_k(:,1), R1Obs_k(:,1));
                % R2Z0s_idx: index of shared feature observation of
                % 2nd robot in R2Obs_k
                R2Z0s_idx = find(R2Z0s_lv);
                % R2Z0s: shared feature observation of 2nd robot
                R2Z0s = R2Obs_k(R2Z0s_idx,:);



                %% use Gauss-Newton iteration to optimize the shared state at step 0
                % use R2Xp0 as the initial value of R2's pose in the GN iteration
                Xs = [2*ones(3,1),zeros(3,1),R2Xp0;
                    zeros(size(R1Xf0s,1),1),R1Xf0s];

                R1sRn = [];
                R2sRn = [];
                for kj = 1:(size(R1Xf0s,1)/2)
                    R1sRn = blkdiag(R1sRn,R1R);
                    R2sRn = blkdiag(R2sRn,R2R);
                end
                Pz = blkdiag(R1sRn,R2sRn);

                Z0s = [R1Z0s(:,2);R2Z0s(:,2)];

                Xs(3,1) = wrap(Xs(3,1));
                XsGni = Xs;

                %% 显示GNI的结果是奇异矩阵是因为加噪声随机生成的R2Xp0落在feature的真值上了

                [XsGni(:,3),PzGni] = GNI(R1Xp0,Xs(:,3),Pz,Z0s,CC);
                XsGni(3,1) = wrap(XsGni(3,1));

                % Set the elements that are less than CovT to zero. This can be useful for dealing with numerical errors or avoiding unnecessary imaginary parts in calculations.
                PzGni(abs(PzGni)<CovT) = 0;

                %% Add the information of R1 into XsGni and PsGni
                X0 = [ones(3,1),zeros(3,1),R1Xp0;XsGni];
                P0 = blkdiag(R1O,PzGni);



                %% estimate new observed feature's state at step 0 using the observation model
                % find the new observed feature IDs in 1st robot
                % R1Zkn_lv: logical vector of new feature observation of
                % 2nd robot at step k
                R1Zkn_lv = ~ismember(R1Obs_k(:,1),X0(7:end,2));
                % R1Zkn_idx: index of new feature observation of
                % 1st robot in R1Obs_k
                R1Zkn_idx = find(R1Zkn_lv);
                % R1Z0n: new feature observation of 1st robot
                R1Z0n = R1Obs_k(R1Zkn_idx,:);

                R1Xfkn = R1Z0n;
                R1Xfkn(1:2:(end-1),2) = X0(1,3) + cos(X0(3,3))*(R1Z0n(1:2:(end-1),2)) - sin(X0(3,3))*(R1Z0n(2:2:end,2));
                R1Xfkn(2:2:end,2) = X0(2,3) + sin(X0(3,3))*(R1Z0n(1:2:(end-1),2)) + cos(X0(3,3))*(R1Z0n(2:2:end,2));

                % find the new observed feature IDs in 2nd robot
                % R2Zkn_lv: logical vector of new feature observation of
                % 2nd robot at step k
                R2Zkn_lv = ~ismember(R2Obs_k(:,1),X0(7:end,2));
                % R2Zkn_idx: index of new feature observation of
                % 2nd robot in R2Obs_k
                R2Zkn_idx = find(R2Zkn_lv);
                % R2Z0n: new feature observation of 2nd robot
                R2Z0n = R2Obs_k(R2Zkn_idx,:);

                % Zkns = intersect(R1Z0n(:,1),R2Z0n(:,1));

                R2Xfkn = R2Z0n;
                R2Xfkn(1:2:(end-1),2) = X0(4,3) + cos(X0(6,3))*(R2Z0n(1:2:(end-1),2)) - sin(X0(6,3))*(R2Z0n(2:2:end,2));
                R2Xfkn(2:2:end,2) = X0(5,3) + sin(X0(6,3))*(R2Z0n(1:2:(end-1),2)) + cos(X0(6,3))*(R2Z0n(2:2:end,2));

                % Xk00e: state estimate
                % first column: 1 -> robot posture; 2 -> feature position
                % second column: posture id or position id
                % third column: data
                Xk00e = [X0;
                    ones(size(R1Xfkn,1),1),R1Xfkn;
                    2*ones(size(R2Xfkn,1),1),R2Xfkn];

                % Jacobian of Xk
                JFXk = sparse(size(Xk00e,1),size(X0,1));
                JFXk(1:size(X0,1),1:size(X0,1)) = eye(size(X0,1));
                JFXk(size(X0,1)+(1:2:(size(R1Xfkn,1)-1)),1:3) = [repmat([1,0],size(R1Xfkn,1)/2,1), -sin(X0(3,3))*R1Z0n(1:2:(end-1),2) - cos(X0(3,3))*R1Z0n(2:2:end,2)];
                JFXk(size(X0,1)+(2:2:size(R1Xfkn,1)),1:3) = [repmat([0,1],size(R1Xfkn,1)/2,1), cos(X0(3,3))*R1Z0n(1:2:(end-1),2) - sin(X0(3,3))*R1Z0n(2:2:end,2)];

                JFXk(size(X0,1)+size(R1Xfkn,1)+(1:2:(size(R2Xfkn,1)-1)),4:6) = [repmat([1,0],size(R2Xfkn,1)/2,1), -sin(X0(6,3))*R2Z0n(1:2:(end-1),2) - cos(X0(6,3))*R2Z0n(2:2:end,2)];
                JFXk(size(X0,1)+size(R1Xfkn,1)+(2:2:size(R2Xfkn,1)),4:6) = [repmat([0,1],size(R2Xfkn,1)/2,1), cos(X0(6,3))*R2Z0n(1:2:(end-1),2) - sin(X0(6,3))*R2Z0n(2:2:end,2)];


                R1nRn = [];
                R2nRn = [];
                JFWk = sparse(size(Xk00e,1),size(R1Z0n,1)+size(R2Z0n,1));
                for R1jn = 1:(size(R1Z0n,1)/2)
                    R1nRn = blkdiag(R1nRn, R1R);
                    JFWk(size(X0,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = rotationMatrix(X0(3,3));
                end
                for R2jn = 1:(size(R2Z0n,1)/2)
                    R2nRn = blkdiag(R2nRn, R2R);
                    JFWk(size(X0,1)+size(R1Z0n,1)+(R2jn-1)*2+(1:2),size(R1Z0n,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(X0(6,3));
                end

                nRn = blkdiag(R1nRn,R2nRn);
                Pk00 = JFXk*P0*JFXk'+JFWk*nRn*JFWk';

                Pk00(abs(Pk00)<CovT) = 0;

                % Standard EKF
                R1XpFull = Xk00e(1:3,2:3); % save all robot postures of R1
                R2XpFull = Xk00e(4:6,2:3); % save all robot postures of R2

                R1PFull = Pk00(1:3,1:3);
                R2PFull = Pk00(4:6,4:6);

                continue
            end

            %% Prediction using the motion model

            R1Odo_k = R1Odo(R1Odo(:,2)==k,3);
            R2Odo_k = R2Odo(R2Odo(:,2)==k,3);

            %% Standard EKF
            Xk10e = Xk00e;
            Xk10e(1:6,2) = Xk10e(1:6,2)+1;
            Xk10e(1:3,3) = Xk00e(1:3,3) + ...
                [cos(Xk00e(3,3))*R1Odo_k(1,1) - sin(Xk00e(3,3))*R1Odo_k(2,1);
                sin(Xk00e(3,3))*R1Odo_k(1,1) + cos(Xk00e(3,3))*R1Odo_k(2,1);
                R1Odo_k(3,1)];
            Xk10e(4:6,3) = Xk00e(4:6,3) + ...
                [cos(Xk00e(6,3))*R2Odo_k(1,1) - sin(Xk00e(6,3))*R2Odo_k(2,1);
                sin(Xk00e(6,3))*R2Odo_k(1,1) + cos(Xk00e(6,3))*R2Odo_k(2,1);
                R2Odo_k(3,1)];

            Xk10e([3,6],3) = wrap(Xk10e([3,6],3));

            DeltaXfX = sparse(size(Xk10e,1),size(Xk00e,1));
            DeltaXfX(1:6,1:6) = blkdiag([1,0,-sin(Xk00e(3,3))*R1Odo_k(1,1) - cos(Xk00e(3,3))*R1Odo_k(2,1); ...
                0,1,cos(Xk00e(3,3))*R1Odo_k(1,1) - sin(Xk00e(3,3))*R1Odo_k(2,1); ...
                0,0,1], ...
                [1,0,-sin(Xk00e(6,3))*R2Odo_k(1,1) - cos(Xk00e(6,3))*R2Odo_k(2,1); ...
                0,1,cos(Xk00e(6,3))*R2Odo_k(1,1) - sin(Xk00e(6,3))*R2Odo_k(2,1); ...
                0,0,1]);
            DeltaXfX(7:end,7:end) = eye(size(DeltaXfX(7:end,7:end)));

            DeltaXfW = sparse(size(Xk00e,1),6);
            DeltaXfW(1:6,1:6) = blkdiag([cos(Xk00e(3,3)),-sin(Xk00e(3,3)),0; ...
                sin(Xk00e(3,3)),cos(Xk00e(3,3)),0; ...
                0,0,1], ...
                [cos(Xk00e(6,3)),-sin(Xk00e(6,3)),0; ...
                sin(Xk00e(6,3)),cos(Xk00e(6,3)),0; ...
                0,0,1]);

            DWk = blkdiag(R1Q,R2Q);
            Pk10 = DeltaXfX * Pk00 * DeltaXfX' + DeltaXfW * DWk * DeltaXfW';

            % Set the elements that are less than CovT to zero. This can be
            % useful for dealing with numerical errors or avoiding unnecessary
            % imaginary parts in calculations.
            Pk10(abs(Pk10)<CovT) = 0;



            %% Feature initialization using new feature observations from R1 and R2
            % find the feature observations in R1 from new features of Xk10e
            R1Zkn_lv = ~ismember(R1Obs_k(:,1),Xk10e(7:end,2));
            R1Zkn_idx = find(R1Zkn_lv);
            R1Zkn = R1Obs_k(R1Zkn_idx,:);

            % find the feature observations in R2 from new features of Xk10e
            R2Zkn_lv = ~ismember(R2Obs_k(:,1),Xk10e(7:end,2));
            R2Zkn_idx = find(R2Zkn_lv);
            R2Zkn = R2Obs_k(R2Zkn_idx,:);

            % R1和R2都看到同一个新feature怎么办
            Zkns = intersect(R1Zkn(:,1),R2Zkn(:,1));
            % 用R1的来initialization,R2的从R2Zkn中去掉，后面用来做update
            if ~isempty(Zkns)
                for ZknsNum = 1:size(Zkns,1)
                    R2Zkn(R2Zkn(:,1)==Zkns(ZknsNum,1),:) = [];
                end
            end

            Zkn = [R1Zkn;R2Zkn];

            % Standard EKF
            Xk10efi = Xk10e;
            Pk10fi = Pk10;

            if ~isempty(Zkn)
                % Standard EKF
                R1Xfn = R1Zkn;
                R2Xfn = R2Zkn;

                DeltaGX = sparse(size(Xk10efi,1),size(Xk10e,1));
                DeltaGX(1:size(Xk10e,1),1:size(Xk10e,1)) = eye(size(Xk10e,1));

                %% Observation noise Cov
                R1nRn = [];
                R2nRn = [];
                % Standard ekf
                DeltaGV = sparse(size(Xk10efi,1),size(Zkn,1));

                if ~isempty(R1Xfn)
                    %% Standard EKF
                    R1Xfn(1:2:(end-1),2) = Xk10e(1,3) + cos(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) - sin(Xk10e(3,3))*R1Zkn(2:2:end,2);
                    R1Xfn(2:2:end,2) = Xk10e(2,3) + sin(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) + cos(Xk10e(3,3))*R1Zkn(2:2:end,2);

                    % Cov
                    DeltaGX(size(Xk10e,1)+(1:2:(size(R1Xfn,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn,1)/2,1), ...
                        -sin(Xk10e(3,3))*R1Zkn(1:2:(end-1),2)-cos(Xk10e(3,3))*R1Zkn(2:2:end,2)];
                    DeltaGX(size(Xk10e,1)+(2:2:size(R1Xfn,1)),1:3) = [repmat([0, 1],size(R1Xfn,1)/2,1), ...
                        cos(Xk10e(3,3))*R1Zkn(1:2:(end-1),2)-sin(Xk10e(3,3))*R1Zkn(2:2:end,2)];

                    for R1jn = 1:(size(R1Zkn,1)/2)
                        R1nRn = blkdiag(R1nRn, R1R);
                        DeltaGV(size(Xk10e,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = rotationMatrix(Xk10e(3,3));
                    end
                end

                if ~isempty(R2Xfn)
                    % Standard EKF
                    R2Xfn(1:2:(end-1),2) = Xk10e(4,3) + cos(Xk10e(6,3))*R2Zkn(1:2:end,2) - sin(Xk10e(6,3))*R2Zkn(2:2:end,2);
                    R2Xfn(2:2:end,2) = Xk10e(5,3) + sin(Xk10e(6,3))*R2Zkn(1:2:end,2) + cos(Xk10e(6,3))*R2Zkn(2:2:end,2);

                    % Cov
                    DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(1:2:(size(R2Xfn,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn,1)/2,1), ...
                        -sin(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-cos(Xk10e(6,3))*R2Zkn(2:2:end,2)];
                    DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(2:2:size(R2Xfn,1)),4:6) = [repmat([0, 1],size(R2Xfn,1)/2,1), ...
                        cos(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-sin(Xk10e(6,3))*R2Zkn(2:2:end,2)];

                    for R2jn = 1:(size(R2Zkn,1)/2)
                        R2nRn = blkdiag(R2nRn, R2R);
                        DeltaGV(size(Xk10e,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(Xk10e(6,3));
                    end
                end

                nRn = blkdiag(R1nRn,R2nRn);

                % Standard EKF
                Xfn = [R1Xfn;R2Xfn];
                Xk10efi = [Xk10e;
                    [ones(size(R1Xfn,1),1);2*ones(size(R2Xfn,1),1)],Xfn];

                Pk10fi = DeltaGX*Pk10*DeltaGX'+DeltaGV*nRn*DeltaGV';
                Pk10fi(abs(Pk10fi)<CovT) = 0;
            end



            %% Update using shared feature observations from R1 and R2
            %% R1
            % find the shared feature observations in R1 and R2 of Xk10e
            R1Zks_lv = ismember(R1Obs_k(:,1),Xk10efi(7:end,2));
            R1Zks_idx = find(R1Zks_lv);
            R1Zks1 = R1Obs_k(R1Zks_idx,:);

            % Standard EKF
            % find the shared features in Xk10efi
            R1Xfks_lv = ismember(Xk10efi(7:end,2),R1Obs_k(:,1));
            R1Xfks_idx = find(R1Xfks_lv)+6; % R1Xfks_idx: index of R1's shared features with Xk10efi at step k
            R1Xfks = Xk10efi(R1Xfks_idx,2:3);

            % re-order the shared feature R1Zks to make it consistent with R1Xfks
            [~,R1ZkS_idx] = ismember(R1Xfks(:,1),R1Zks1(:,1));
            R1ZkS_idx(2:2:end,1)=R1ZkS_idx(2:2:end,1)+1;
            % R1ZkS_idx: index of R1's shared features with R1Zks at step k
            R1Zks2 = R1Zks1(R1ZkS_idx,:);

            %% R2
            R2Zks_lv = ismember(R2Obs_k(:,1),Xk10efi(7:end,2));
            R2Zks_idx = find(R2Zks_lv);
            R2Zks1 = R2Obs_k(R2Zks_idx,:);

            % Standard EKF
            R2Xfks_lv = ismember(Xk10efi(7:end,2),R2Obs_k(:,1));
            R2Xfks_idx = find(R2Xfks_lv)+6;
            R2Xfks = Xk10efi(R2Xfks_idx,2:3);

            [~,R2ZkS_idx] = ismember(R2Xfks(:,1),R2Zks1(:,1));
            R2ZkS_idx(2:2:end,1)=R2ZkS_idx(2:2:end,1)+1;
            R2Zks2 = R2Zks1(R2ZkS_idx,:);

            %%
            Zks = [ones(size(R1Zks2,1),1),R1Zks2;2*ones(size(R2Zks2,1),1),R2Zks2];

            % Standard EKF
            Xk11e = Xk10efi;
            Pk11 = Pk10fi;

            if ~isempty(Zks)

                % Standard EKF
                HX10e = sparse(size(Zks,1), 1);
                JHX10e = sparse(size(Zks,1), size(Xk10efi,1));

                R1DV = [];
                R2DV = [];

                %% R1
                if ~isempty(R1Zks2)
                    % Standard EKF
                    HX10e(1:2:(size(R1Zks2,1)-1),1) = cos(Xk10efi(3,3))*(R1Xfks(1:2:(end-1),2)-Xk10efi(1,3)) + ...
                        sin(Xk10efi(3,3))*(R1Xfks(2:2:end,2)-Xk10efi(2,3));
                    HX10e(2:2:size(R1Zks2,1),1) = -sin(Xk10efi(3,3))*(R1Xfks(1:2:(end-1),2)-Xk10efi(1,3)) + ...
                        cos(Xk10efi(3,3))*(R1Xfks(2:2:end,2)-Xk10efi(2,3));

                    JHX10e(1:2:(size(R1Zks2,1)-1),1:3) = [repmat([-cos(Xk10efi(3,3)),-sin(Xk10efi(3,3))],size(R1Zks2,1)/2,1), ...
                        -sin(Xk10efi(3,3))*(R1Xfks(1:2:(end-1),2)-Xk10efi(1,3))+cos(Xk10efi(3,3))*(R1Xfks(2:2:end,2)-Xk10efi(2,3))];

                    JHX10e(2:2:size(R1Zks2,1),1:3) = [repmat([sin(Xk10efi(3,3)),-cos(Xk10efi(3,3))],size(R1Zks2,1)/2,1), ...
                        -cos(Xk10efi(3,3))*(R1Xfks(1:2:(end-1),2)-Xk10efi(1,3))-sin(Xk10efi(3,3))*(R1Xfks(2:2:end,2)-Xk10efi(2,3))];

                    for R1kj = 1:size(R1Zks2,1)/2
                        R1DV = blkdiag(R1DV,R1R);

                        % Standard EKF
                        JHX10e((R1kj-1)*2+(1:2), R1Xfks_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(3,3))';
                    end
                end

                %% R2
                if ~isempty(R2Zks2)

                    % Standard EKF
                    HX10e(size(R1Zks2,1)+(1:2:(size(R2Zks2,1)-1)),1) = cos(Xk10efi(6,3))*(R2Xfks(1:2:(end-1),2)-Xk10efi(4,3)) + ...
                        sin(Xk10efi(6,3))*(R2Xfks(2:2:end,2)-Xk10efi(5,3));
                    HX10e(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),1) = -sin(Xk10efi(6,3))*(R2Xfks(1:2:(end-1),2)-Xk10efi(4,3)) + ...
                        cos(Xk10efi(6,3))*(R2Xfks(2:2:end,2)-Xk10efi(5,3));

                    JHX10e(size(R1Zks2,1)+(1:2:size(R2Zks2,1)-1),4:6) = [repmat([-cos(Xk10efi(6,3)),-sin(Xk10efi(6,3))],size(R2Zks2,1)/2,1), ...
                        -sin(Xk10efi(6,3))*(R2Xfks(1:2:(end-1),2)-Xk10efi(4,3))+cos(Xk10efi(6,3))*(R2Xfks(2:2:end,2)-Xk10efi(5,3))];

                    JHX10e(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),4:6) = [repmat([sin(Xk10efi(6,3)),-cos(Xk10efi(6,3))],size(R2Zks2,1)/2,1), ...
                        -cos(Xk10efi(6,3))*(R2Xfks(1:2:(end-1),2)-Xk10efi(4,3))-sin(Xk10efi(6,3))*(R2Xfks(2:2:end,2)-Xk10efi(5,3))];

                    for R2kj = 1:size(R2Zks2,1)/2
                        R2DV = blkdiag(R2DV,R2R);

                        % Standard EKF
                        JHX10e(size(R1Zks2,1)+(R2kj-1)*2+(1:2), R2Xfks_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(6,3))';
                    end
                end

                DV = blkdiag(R1DV,R2DV);

                %% Innovation Covariance S and Kalman Gain K
                % Standard EKF
                Ss = JHX10e * Pk10fi * JHX10e' + DV;
                Ks = Pk10fi * JHX10e' /Ss;

                %% Updating process using observation model
                % Standard EKF
                Xk11e(:,3) = Xk10efi(:,3) + Ks*(Zks(:,3)-HX10e);
                Xk11e([3,6],3) = wrap(Xk11e([3,6],3));
                Pk11 = Pk10fi - Ks*Ss*Ks';
                Pk11(abs(Pk11)<CovT) = 0;
            end

            % Standard EKF
            R1XpFull = [R1XpFull;Xk11e(1:3,2:3)];
            R1PFull = [R1PFull;Pk11(1:3,1:3)];

            R2XpFull = [R2XpFull;Xk11e(4:6,2:3)];
            R2PFull = [R2PFull;Pk11(4:6,4:6)];

            % R1ee = errorEllipse(Pk11(1:2,1:2), Xk11e(1:2,3), CI);
            % R2ee = errorEllipse(Pk11(4:5,4:5), Xk11e(4:5,3), CI);
            % 
            % R1eeFull = [R1eeFull,R1ee];
            % R2eeFull = [R2eeFull,R2ee];

            Xk00e = Xk11e;
            Pk00 = Pk11;
        end

        % Standard EKF
        R1XpFullSet = [R1XpFullSet,R1XpFull(:,2)];
        R1PFullSet = [R1PFullSet,R1PFull];

        R2XpFullSet = [R2XpFullSet,R2XpFull(:,2)];
        R2PFullSet = [R2PFullSet,R2PFull];

        XfFullSet = [XfFullSet,Xk11e(7:end,3)];
        PfFullSet = [PfFullSet,Pk11(7:end,7:end)];
    end

    feaNum = size(Xk11e(7:end,1),1)/2;

    %% Standard EKF
    R1XpFullSet = [R1XpFull(:,1),R1XpFullSet];
    R2XpFullSet = [R2XpFull(:,1),R2XpFullSet];
    XfFullSet = [Xk11e(7:end,2),XfFullSet];

    R1XrFullSet = [];
    R2XrFullSet = [];
    R1XphiFullSet = [];
    R2XphiFullSet = [];

    DeltaR1XpFullSet = [];
    DeltaR2XpFullSet = [];

    DeltaR1XrFullSet = [];
    DeltaR2XrFullSet = [];

    DeltaR1XphiFullSet = [];
    DeltaR2XphiFullSet = [];

    for pn = 0:poseNum

        %% debug
        % if pn == 34 || pn == 60
        %     keyboard
        % end
        %%

        R1XpTrue = [R1XrTrue(pn*2+(1:2),:);R1XphiT(pn+1,:)];
        R2XpTrue = [R2XrTrue(pn*2+(1:2),:);R2XphiT(pn+1,:)];

        %% Standard EKF
        DeltaR1XpFullSet((end+1):(end+3),:) = [R1XpTrue(:,1),R1XpFullSet(pn*3+(1:3),2:end)-R1XpTrue(:,2)];
        DeltaR2XpFullSet((end+1):(end+3),:) = [R2XpTrue(:,1),R2XpFullSet(pn*3+(1:3),2:end)-R2XpTrue(:,2)];

        % wrap the delta angle
        DeltaR1XpFullSet(end,2:end) = wrap(DeltaR1XpFullSet(end,2:end));
        DeltaR2XpFullSet(end,2:end) = wrap(DeltaR2XpFullSet(end,2:end));

        DeltaR1XrFullSet((end+1):(end+2),:) = DeltaR1XpFullSet((end-2):(end-1),:);
        DeltaR2XrFullSet((end+1):(end+2),:) = DeltaR2XpFullSet((end-2):(end-1),:);

        DeltaR1XphiFullSet(end+1,:) = DeltaR1XpFullSet(end,:);
        DeltaR2XphiFullSet(end+1,:) = DeltaR2XpFullSet(end,:);
    end

    %% re-order the true features XfTrueAll's IDs to make it consistent with FFullSet 每一步的
    [~,XfTrue_idx] = ismember(XfFullSet(:,1),XfTrueAll(:,1));
    XfTrue_idx(2:2:end,1) = XfTrue_idx(2:2:end,1)+1;
    XfTrue = XfTrueAll(XfTrue_idx,:);
    % Standard EKF
    DeltaXfFullSet = [XfFullSet(:,1),XfFullSet(:,2:end)-XfTrue(:,2)];

    %% save the Monte Carlo Experiments result
    if i == 1
        save('MTE_results_StdEKF_20fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
    elseif i == 2
        save('MTE_results_StdEKF_60fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
    else
        save('MTE_results_StdEKF_100fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
    end
end