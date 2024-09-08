clc
close all

% for i = 1:3
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

    % Ideal EKF
    R1XpIdeFullSet = [];
    R1PIdeFullSet = [];
    R2XpIdeFullSet = [];
    R2PIdeFullSet = [];
    XfIdeFullSet = [];
    PfIdeFullSet = [];

    % FEJ EKF
    R1XpFejFullSet = [];
    R1PFejFullSet = [];
    R2XpFejFullSet = [];
    R2PFejFullSet = [];
    XfFejFullSet = [];
    PfFejFullSet = [];

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

        % Ideal EKF
        R1XpIdeFull = [];
        R1PIdeFull = [];
        R2XpIdeFull = [];
        R2PIdeFull = [];
        PfIdeFull = [];

        % FEJ EKF
        R1XpFejFull = [];
        R1PFejFull = [];
        R2XpFejFull = [];
        R2PFejFull = [];
        PfFejFull = [];

        Xs = [];
        X0 = [];

        % std EKF
        Xk00e = [];
        Xk10e = [];
        Xk10efi = [];
        Xk11e = [];
        % Ideal EKF
        Xk00e_ide = [];
        Xk10e_ide = [];
        Xk10efi_ide = [];
        Xk11e_ide = [];

        % Fej EKF
        Xk00e_fej = [];
        Xk10e_fej = [];
        Xk10efi_fej = [];
        Xk11e_fej = [];
        XfFe = []; % First estimated feature position

        for k = 0:poseNum

            % k

            % Standard & EFJ EKF
            R1Obs_k = R1Obs(R1Obs(:,1)==k,2:3);
            R2Obs_k = R2Obs(R2Obs(:,1)==k,2:3);

            % Ideal EKF
            R1ObsT_k = R1ObsT(R1ObsT(:,1)==k,2:3);
            R2ObsT_k = R2ObsT(R2ObsT(:,1)==k,2:3);

            if k == 0
                % find the shared observed feature IDs in 1st robot
                % R1Z0s_lv: logical vector of shared feature observation of
                % 1st robot at step 0
                R1Z0s_lv = ismember(R1Obs_k(:,1),R2Obs_k(:,1));
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

                % Ideal EKF
                R1XpIdeFull = Xk00e(1:3,2:3); % save all robot postures of R1
                R2XpIdeFull = Xk00e(4:6,2:3); % save all robot postures of R2

                R1PIdeFull = Pk00(1:3,1:3);
                R2PIdeFull = Pk00(4:6,4:6);

                % FEJ EKF
                R1XpFejFull = Xk00e(1:3,2:3); % save all robot postures of R1
                R2XpFejFull = Xk00e(4:6,2:3); % save all robot postures of R2

                R1PFejFull = Pk00(1:3,1:3);
                R2PFejFull = Pk00(4:6,4:6);
                
                % record the initialized features for FEJ
                XfFe = Xk00e(7:end,:); % First estimated feature position

                continue
            end

            %% Prediction using the motion model

            R1Odo_k = R1Odo(R1Odo(:,2)==k,3);
            R2Odo_k = R2Odo(R2Odo(:,2)==k,3);

            %% Standard EKF
            Xk10e = Xk00e;
            Xk10e(1:6,2) = Xk10e(1:6,2)+1;
            Xk10e(1:3,3) = Xk00e(1:3,3) + ...
                [rotationMatrix(Xk00e(3,3))*R1Odo_k(1:2,1);
                R1Odo_k(3,1)];
            Xk10e(4:6,3) = Xk00e(4:6,3) + ...
                [rotationMatrix(Xk00e(6,3))*R2Odo_k(1:2,1);
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

            DeltaXfW = sparse(size(Xk10e,1),6);
            DeltaXfW(1:6,1:6) = blkdiag(rotationMatrix(Xk00e(3,3)),1,rotationMatrix(Xk00e(6,3)),1);

            DWk = blkdiag(R1Q,R2Q);
            Pk10 = DeltaXfX * Pk00 * DeltaXfX' + DeltaXfW * DWk * DeltaXfW';

            % Set the elements that are less than CovT to zero. This can be
            % useful for dealing with numerical errors or avoiding unnecessary
            % imaginary parts in calculations.
            Pk10(abs(Pk10)<CovT) = 0;

            %% Ideal EKF
            R1OdoT_k = R1OdoT(R1OdoT(:,2)==k,3);
            R2OdoT_k = R2OdoT(R2OdoT(:,2)==k,3);

            if k == 1
                Xk00e_ide = Xk00e;
                Pk00_ide = Pk00;
            end

            Xk10e_ide = Xk00e_ide;
            Xk10e_ide(1:6,2) = Xk10e_ide(1:6,2)+1;
            Xk10e_ide(1:3,3) = Xk00e_ide(1:3,3) + ...
                [rotationMatrix(Xk00e_ide(3,3))*R1Odo_k(1:2,1);
                R1Odo_k(3,1)];
            Xk10e_ide(4:6,3) = Xk00e_ide(4:6,3) + ...
                [rotationMatrix(Xk00e_ide(6,3))*R2Odo_k(1:2,1);
                R2Odo_k(3,1)];

            Xk10e_ide([3,6],3) = wrap(Xk10e_ide([3,6],3));

            Xrk00T = [R1XrTrue(R1XrTrue(:,1)==k-1,2);R1XphiT(R1XphiT(:,1)==k-1,2); ...
                R2XrTrue(R2XrTrue(:,1)==k-1,2);R2XphiT(R2XphiT(:,1)==k-1,2)];

            DeltaXfX_ide = sparse(size(Xk10e_ide,1),size(Xk00e_ide,1));
            DeltaXfX_ide(1:6,1:6) = blkdiag([1,0,-sin(Xrk00T(3,1))*R1OdoT_k(1,1) - cos(Xrk00T(3,1))*R1OdoT_k(2,1); ...
                0,1,cos(Xrk00T(3,1))*R1OdoT_k(1,1) - sin(Xrk00T(3,1))*R1OdoT_k(2,1); ...
                0,0,1], ...
                [1,0,-sin(Xrk00T(6,1))*R2OdoT_k(1,1) - cos(Xrk00T(6,1))*R2OdoT_k(2,1); ...
                0,1,cos(Xrk00T(6,1))*R2OdoT_k(1,1) - sin(Xrk00T(6,1))*R2OdoT_k(2,1); ...
                0,0,1]);
            DeltaXfX_ide(7:end,7:end) = eye(size(DeltaXfX_ide(7:end,7:end)));

            DeltaXfW_ide = sparse(size(Xk00e_ide,1),6);
            DeltaXfW_ide(1:6,1:6) = blkdiag(rotationMatrix(Xrk00T(3,1)),1,rotationMatrix(Xrk00T(6,1)),1);

            Pk10_ide = DeltaXfX_ide * Pk00_ide * DeltaXfX_ide' + DeltaXfW_ide * DWk * DeltaXfW_ide';
            Pk10_ide(abs(Pk10_ide)<CovT) = 0;

            %% FEJ EKF
            R1Odo_k = R1Odo(R1Odo(:,2)==k,3);
            R2Odo_k = R2Odo(R2Odo(:,2)==k,3);

            if k == 1
                Xk10e_fej = Xk00e;
                % Pk10_fej = Pk00;

                Xk00e_fej = Xk00e;
                Pk00_fej = Pk00;
            end

            DeltaXfX_fej = sparse(size(Xk00e_fej,1),size(Xk00e_fej,1));
            DeltaXfX_fej(1:6,1:6) = blkdiag([1,0,-sin(Xk10e_fej(3,3))*R1Odo_k(1,1) - cos(Xk10e_fej(3,3))*R1Odo_k(2,1); ...
                0,1,cos(Xk10e_fej(3,3))*R1Odo_k(1,1) - sin(Xk10e_fej(3,3))*R1Odo_k(2,1); ...
                0,0,1], ...
                [1,0,-sin(Xk10e_fej(6,3))*R2Odo_k(1,1) - cos(Xk10e_fej(6,3))*R2Odo_k(2,1); ...
                0,1,cos(Xk10e_fej(6,3))*R2Odo_k(1,1) - sin(Xk10e_fej(6,3))*R2Odo_k(2,1); ...
                0,0,1]);
            DeltaXfX_fej(7:end,7:end) = eye(size(DeltaXfX_fej(7:end,7:end),1));

            DeltaXfW_fej = sparse(size(Xk00e_fej,1),6);
            DeltaXfW_fej(1:6,1:6) = blkdiag(rotationMatrix(Xk10e_fej(3,3)),1,rotationMatrix(Xk10e_fej(6,3)),1);
            
            % Debug
            % if k == 8
            %     keyboard
            % end

            Xk10e_fej = Xk00e_fej;
            Xk10e_fej(1:6,2) = Xk00e_fej(1:6,2)+1;
            Xk10e_fej(1:3,3) = Xk00e_fej(1:3,3) + ...
                [rotationMatrix(Xk00e_fej(3,3))*R1Odo_k(1:2,1);
                R1Odo_k(3,1)];
            Xk10e_fej(4:6,3) = Xk00e_fej(4:6,3) + ...
                [rotationMatrix(Xk00e_fej(6,3))*R2Odo_k(1:2,1);
                R2Odo_k(3,1)];

            Xk10e_fej([3,6],3) = wrap(Xk10e_fej([3,6],3));

            Pk10_fej = DeltaXfX_fej * Pk00_fej * DeltaXfX_fej' + DeltaXfW_fej * DWk * DeltaXfW_fej';
            Pk10_fej(abs(Pk10_fej)<CovT) = 0;



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
                
                % Debug
                keyboard;

                for ZknsNum = 1:size(Zkns,1)
                    R2Zkn(R2Zkn(:,1)==Zkns(ZknsNum,1),:) = [];
                end
            end

            Zkn = [R1Zkn;R2Zkn];

            % Standard EKF
            Xk10efi = Xk10e;
            Pk10fi = Pk10;

            % Ideal EKF
            Xk10efi_ide = Xk10e_ide;
            Pk10fi_ide = Pk10_ide;

            % FEJ EKF
            Xk10efi_fej = Xk10e_fej;
            Pk10fi_fej = Pk10_fej;

            if ~isempty(Zkn)
                % Standard EKF
                R1Xfn = R1Zkn;
                R2Xfn = R2Zkn;

                DeltaGX = sparse(size(Xk10efi,1),size(Xk10e,1));
                DeltaGX(1:size(Xk10e,1),1:size(Xk10e,1)) = eye(size(Xk10e,1));

                % Ideal EKF
                R1Xfn_ide = R1Zkn;
                R2Xfn_ide = R2Zkn;

                DeltaGX_ide = sparse(size(Xk10efi_ide,1),size(Xk10e_ide,1));
                DeltaGX_ide(1:size(Xk10e_ide,1),1:size(Xk10e_ide,1)) = eye(size(Xk10e_ide,1));

                % FEJ EKF
                R1Xfn_fej = R1Zkn;
                R2Xfn_fej = R2Zkn;

                DeltaGX_fej = sparse(size(Xk10efi_fej,1),size(Xk10e_fej,1));
                DeltaGX_fej(1:size(Xk10e_fej,1),1:size(Xk10e_fej,1)) = eye(size(Xk10e_fej,1));

                %% Observation noise Cov
                R1nRn = [];
                R2nRn = [];
                % Standard ekf
                DeltaGV = sparse(size(Xk10efi,1),size(Zkn,1));
                % Ideal ekf
                DeltaGV_ide = sparse(size(Xk10efi_ide,1),size(Zkn,1));
                % fej ekf
                DeltaGV_fej = sparse(size(Xk10efi_fej,1),size(Zkn,1));


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

                    %% Ideal EKF
                    R1Xfn_ide(1:2:(end-1),2) = Xk10e_ide(1,3) + cos(Xk10e_ide(3,3))*R1Zkn(1:2:(end-1),2) - sin(Xk10e_ide(3,3))*R1Zkn(2:2:end,2);
                    R1Xfn_ide(2:2:end,2) = Xk10e_ide(2,3) + sin(Xk10e_ide(3,3))*R1Zkn(1:2:(end-1),2) + cos(Xk10e_ide(3,3))*R1Zkn(2:2:end,2);

                    % Cov
                    DeltaGX_ide(size(Xk10e_ide,1)+(1:2:(size(R1Xfn_ide,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn_ide,1)/2,1), ...
                        -sin(Xk10e_ide(3,3))*R1Zkn(1:2:(end-1),2)-cos(Xk10e_ide(3,3))*R1Zkn(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+(2:2:size(R1Xfn_ide,1)),1:3) = [repmat([0, 1],size(R1Xfn_ide,1)/2,1), ...
                        cos(Xk10e_ide(3,3))*R1Zkn(1:2:(end-1),2)-sin(Xk10e_ide(3,3))*R1Zkn(2:2:end,2)];

                    for R1jn_ide = 1:(size(R1Zkn,1)/2)
                        % R1nRn_ide = blkdiag(R1nRn_ide, R1R);
                        DeltaGV_ide(size(Xk10e_ide,1)+(R1jn_ide-1)*2+(1:2),(R1jn_ide-1)*2+(1:2)) = rotationMatrix(Xk10e_ide(3,3));
                    end

                    %% FEJ EKF
                    R1Xfn_fej(1:2:(end-1),2) = Xk10e_fej(1,3) + cos(Xk10e_fej(3,3))*R1Zkn(1:2:(end-1),2) - sin(Xk10e_fej(3,3))*R1Zkn(2:2:end,2);
                    R1Xfn_fej(2:2:end,2) = Xk10e_fej(2,3) + sin(Xk10e_fej(3,3))*R1Zkn(1:2:(end-1),2) + cos(Xk10e_fej(3,3))*R1Zkn(2:2:end,2);

                    XfFe = [XfFe;ones(size(R1Xfn_fej,1),1),R1Xfn_fej]; % First estimated feature position of R1

                    % Cov
                    DeltaGX_fej(size(Xk10e_fej,1)+(1:2:(size(R1Xfn_fej,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn_fej,1)/2,1), ...
                        -sin(Xk10e_fej(3,3))*R1Zkn(1:2:(end-1),2)-cos(Xk10e_fej(3,3))*R1Zkn(2:2:end,2)];
                    DeltaGX_fej(size(Xk10e_fej,1)+(2:2:size(R1Xfn_fej,1)),1:3) = [repmat([0, 1],size(R1Xfn_fej,1)/2,1), ...
                        cos(Xk10e_fej(3,3))*R1Zkn(1:2:(end-1),2)-sin(Xk10e_fej(3,3))*R1Zkn(2:2:end,2)];

                    for R1jn_fej = 1:(size(R1Zkn,1)/2)
                        % R1nRn_fej = blkdiag(R1nRn_fej, R1R);
                        DeltaGV_fej(size(Xk10e_fej,1)+(R1jn_fej-1)*2+(1:2),(R1jn_fej-1)*2+(1:2)) = rotationMatrix(Xk10e_fej(3,3));
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

                    % Ideal EKF
                    R2Xfn_ide(1:2:(end-1),2) = Xk10e_ide(4,3) + cos(Xk10e_ide(6,3))*R2Zkn(1:2:end,2) - sin(Xk10e_ide(6,3))*R2Zkn(2:2:end,2);
                    R2Xfn_ide(2:2:end,2) = Xk10e_ide(5,3) + sin(Xk10e_ide(6,3))*R2Zkn(1:2:end,2) + cos(Xk10e_ide(6,3))*R2Zkn(2:2:end,2);

                    % Cov
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(1:2:(size(R2Xfn_ide,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn_ide,1)/2,1), ...
                        -sin(Xk10e_ide(6,3))*R2Zkn(1:2:(end-1),2)-cos(Xk10e_ide(6,3))*R2Zkn(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(2:2:size(R2Xfn_ide,1)),4:6) = [repmat([0, 1],size(R2Xfn_ide,1)/2,1), ...
                        cos(Xk10e_ide(6,3))*R2Zkn(1:2:(end-1),2)-sin(Xk10e_ide(6,3))*R2Zkn(2:2:end,2)];

                    % FEJ EKF
                    R2Xfn_fej(1:2:(end-1),2) = Xk10e_fej(4,3) + cos(Xk10e_fej(6,3))*R2Zkn(1:2:end,2) - sin(Xk10e_fej(6,3))*R2Zkn(2:2:end,2);
                    R2Xfn_fej(2:2:end,2) = Xk10e_fej(5,3) + sin(Xk10e_fej(6,3))*R2Zkn(1:2:end,2) + cos(Xk10e_fej(6,3))*R2Zkn(2:2:end,2);

                    XfFe = [XfFe;2*ones(size(R2Xfn_fej,1),1),R2Xfn_fej]; % First estimated feature position of R2

                    % Cov
                    DeltaGX_fej(size(Xk10e_fej,1)+size(R1Xfn_fej,1)+(1:2:(size(R2Xfn_fej,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn_fej,1)/2,1), ...
                        -sin(Xk10e_fej(6,3))*R2Zkn(1:2:(end-1),2)-cos(Xk10e_fej(6,3))*R2Zkn(2:2:end,2)];
                    DeltaGX_fej(size(Xk10e_fej,1)+size(R1Xfn_fej,1)+(2:2:size(R2Xfn_fej,1)),4:6) = [repmat([0, 1],size(R2Xfn_fej,1)/2,1), ...
                        cos(Xk10e_fej(6,3))*R2Zkn(1:2:(end-1),2)-sin(Xk10e_fej(6,3))*R2Zkn(2:2:end,2)];

                    for R2jn = 1:(size(R2Zkn,1)/2)
                        R2nRn = blkdiag(R2nRn, R2R);
                        % Standard EKF
                        DeltaGV(size(Xk10e,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(Xk10e(6,3));
                        % Ideal EKF
                        DeltaGV_ide(size(Xk10e_ide,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(Xk10e_ide(6,3));
                        % FEJ EKF
                        DeltaGV_fej(size(Xk10e_fej,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(Xk10e_fej(6,3));
                    end
                end

                nRn = blkdiag(R1nRn,R2nRn);

                % Standard EKF
                Xfn = [R1Xfn;R2Xfn];
                Xk10efi = [Xk10e;
                    [ones(size(R1Xfn,1),1);2*ones(size(R2Xfn,1),1)],Xfn];

                Pk10fi = DeltaGX*Pk10*DeltaGX'+DeltaGV*nRn*DeltaGV';
                Pk10fi(abs(Pk10fi)<CovT) = 0;

                % Ideal EKF
                Xfn_ide = [R1Xfn_ide;R2Xfn_ide];
                Xk10efi_ide = [Xk10e_ide;
                    [ones(size(R1Xfn_ide,1),1);2*ones(size(R2Xfn_ide,1),1)],Xfn_ide];

                Pk10fi_ide = DeltaGX_ide*Pk10_ide*DeltaGX_ide'+DeltaGV_ide*nRn*DeltaGV_ide';
                Pk10fi_ide(abs(Pk10fi_ide)<CovT) = 0;

                % FEJ EKF
                Xfn_fej = [R1Xfn_fej;R2Xfn_fej];
                Xk10efi_fej = [Xk10e_fej;
                    [ones(size(R1Xfn_fej,1),1);2*ones(size(R2Xfn_fej,1),1)],Xfn_fej];

                Pk10fi_fej = DeltaGX_fej*Pk10_fej*DeltaGX_fej'+DeltaGV_fej*nRn*DeltaGV_fej';
                Pk10fi_fej(abs(Pk10fi_fej)<CovT) = 0;

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

            % Ideal EKF
            % find the shared true feature R1XfksT in Xk10efi_ide
            % and re-order them to to make them consistent with R1ObsT_k
            R1XfksIde_lv = ismember(Xk10efi_ide(7:end,2),R1ObsT_k(:,1));
            R1XfksIde_idx = find(R1XfksIde_lv)+6;
            R1Xfks_ide = Xk10efi_ide(R1XfksIde_idx,2:3); % For Observation function

            % find the shared true feature R1XfksT in feature truth XfTrueAll
            % and re-order them to to make them consistent with R1Xfks
            [~,R1XfksT_idx] = ismember(R1Xfks(:,1),XfTrueAll(:,1));
            R1XfksT_idx(2:2:end,1)=R1XfksT_idx(2:2:end,1)+1;
            R1XfksT = XfTrueAll(R1XfksT_idx,:); % True feature position for Jacobian function

            % FEJ EKF
            % find the shared feature R1Xfks_fej in Xk10efi_fej
            % and re-order them to to make them consistent with R1Obs_k
            R1XfksFej_lv = ismember(Xk10efi_fej(7:end,2),R1Obs_k(:,1));
            R1XfksFej_idx = find(R1XfksFej_lv)+6;
            R1Xfks_fej = Xk10efi_fej(R1XfksFej_idx,2:3);  % For Observation function

            % find the shared feature R1Xfks_fe in first estimated feature XfFe
            % and re-order them to to make them consistent with R1Xfks
            [~,R1XfksFe_idx] = ismember(R1Xfks(:,1),XfFe(:,2));
            R1XfksFe_idx(2:2:end,1)=R1XfksFe_idx(2:2:end,1)+1;
            R1Xfks_fe = XfFe(R1XfksFe_idx,2:3); % First estimated feature position for Jacobian function

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

            % Ideal EKF
            % find the shared true feature R1XfksT in Xk10efi_ide
            % and re-order them to to make them consistent with R1ObsT_k
            R2XfksIde_lv = ismember(Xk10efi_ide(7:end,2),R2ObsT_k(:,1));
            R2XfksIde_idx = find(R2XfksIde_lv)+6;
            R2Xfks_ide = Xk10efi_ide(R2XfksIde_idx,2:3);

            [~,R2XfksT_idx] = ismember(R2Xfks(:,1),XfTrueAll(:,1));
            R2XfksT_idx(2:2:end,1)=R2XfksT_idx(2:2:end,1)+1;
            R2XfksT = XfTrueAll(R2XfksT_idx,:);

            % FEJ EKF
            R2XfksFej_lv = ismember(Xk10efi_fej(7:end,2),R2Obs_k(:,1));
            R2XfksFej_idx = find(R2XfksFej_lv)+6; % R1Xfks_idx: index of R1's shared features with Xk10efi at step k
            R2Xfks_fej = Xk10efi_fej(R2XfksFej_idx,2:3);

            [~,R2XfksFe_idx] = ismember(R2Xfks(:,1),XfFe(:,2));
            R2XfksFe_idx(2:2:end,1)=R2XfksFe_idx(2:2:end,1)+1;
            R2Xfks_fe = XfFe(R2XfksFe_idx,2:3); % First estimated feature position for Jacobian function

            %%
            Zks = [ones(size(R1Zks2,1),1),R1Zks2;2*ones(size(R2Zks2,1),1),R2Zks2];

            % Standard EKF
            Xk11e = Xk10efi;
            Pk11 = Pk10fi;

            % Ideal EKF
            Xk11e_ide = Xk10efi_ide;
            Pk11_ide = Pk10fi_ide;

            % FEJ EKF
            Xk11e_fej = Xk10efi_fej;
            Pk11_fej = Pk10fi_fej;

            if ~isempty(Zks)

                % Standard EKF
                HX10e = sparse(size(Zks,1), 1);
                JHX10e = sparse(size(Zks,1), size(Xk10efi,1));
                % Ideal EKF
                HX10e_ide = sparse(size(Zks,1), 1);
                JHX10e_ide = sparse(size(Zks,1), size(Xk10efi_ide,1));

                Xrk10Tfi = [R1XrTrue(R1XrTrue(:,1)==k,2);R1XphiT(R1XphiT(:,1)==k,2); ...
                    R2XrTrue(R2XrTrue(:,1)==k,2);R2XphiT(R2XphiT(:,1)==k,2)]; % for Ideal EKF only

                % FEJ EKF
                HX10e_fej = sparse(size(Zks,1), 1);
                JHX10e_fej = sparse(size(Zks,1), size(Xk10efi_fej,1));



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

                    % Ideal EKF
                    HX10e_ide(1:2:(size(R1Zks2,1)-1),1) = cos(Xk10efi_ide(3,3))*(R1Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(1,3)) + ...
                        sin(Xk10efi_ide(3,3))*(R1Xfks_ide(2:2:end,2)-Xk10efi_ide(2,3));
                    HX10e_ide(2:2:size(R1Zks2,1),1) = -sin(Xk10efi_ide(3,3))*(R1Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(1,3)) + ...
                        cos(Xk10efi_ide(3,3))*(R1Xfks_ide(2:2:end,2)-Xk10efi_ide(2,3));

                    JHX10e_ide(1:2:(size(R1Zks2,1)-1),1:3) = [repmat([-cos(Xrk10Tfi(3,1)),-sin(Xrk10Tfi(3,1))],size(R1Zks2,1)/2,1), ...
                        -sin(Xrk10Tfi(3,1))*(R1XfksT(1:2:(end-1),2)-Xrk10Tfi(1,1))+cos(Xrk10Tfi(3,1))*(R1XfksT(2:2:end,2)-Xrk10Tfi(2,1))];

                    JHX10e_ide(2:2:size(R1Zks2,1),1:3) = [repmat([sin(Xrk10Tfi(3,1)),-cos(Xrk10Tfi(3,1))],size(R1Zks2,1)/2,1), ...
                        -cos(Xrk10Tfi(3,1))*(R1XfksT(1:2:(end-1),2)-Xrk10Tfi(1,1))-sin(Xrk10Tfi(3,1))*(R1XfksT(2:2:end,2)-Xrk10Tfi(2,1))];

                    % FEJ EKF
                    HX10e_fej(1:2:(size(R1Zks2,1)-1),1) = cos(Xk10efi_fej(3,3))*(R1Xfks_fej(1:2:(end-1),2)-Xk10efi_fej(1,3)) + ...
                        sin(Xk10efi_fej(3,3))*(R1Xfks_fej(2:2:end,2)-Xk10efi_fej(2,3));
                    HX10e_fej(2:2:size(R1Zks2,1),1) = -sin(Xk10efi_fej(3,3))*(R1Xfks_fej(1:2:(end-1),2)-Xk10efi_fej(1,3)) + ...
                        cos(Xk10efi_fej(3,3))*(R1Xfks_fej(2:2:end,2)-Xk10efi_fej(2,3));

                    JHX10e_fej(1:2:(size(R1Zks2,1)-1),1:3) = [repmat([-cos(Xk10efi_fej(3,3)),-sin(Xk10efi_fej(3,3))],size(R1Zks2,1)/2,1), ...
                        -sin(Xk10efi_fej(3,3))*(R1Xfks_fe(1:2:(end-1),2)-Xk10efi_fej(1,3))+cos(Xk10efi_fej(3,3))*(R1Xfks_fe(2:2:end,2)-Xk10efi_fej(2,3))];

                    JHX10e_fej(2:2:size(R1Zks2,1),1:3) = [repmat([sin(Xk10efi_fej(3,3)),-cos(Xk10efi_fej(3,3))],size(R1Zks2,1)/2,1), ...
                        -cos(Xk10efi_fej(3,3))*(R1Xfks_fe(1:2:(end-1),2)-Xk10efi_fej(1,3))-sin(Xk10efi_fej(3,3))*(R1Xfks_fe(2:2:end,2)-Xk10efi_fej(2,3))];
                    %%
                    for R1kj = 1:size(R1Zks2,1)/2
                        R1DV = blkdiag(R1DV,R1R);

                        % Standard EKF
                        JHX10e((R1kj-1)*2+(1:2), R1Xfks_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(3,3))';
                        % Ideal EKF
                        JHX10e_ide((R1kj-1)*2+(1:2), R1XfksIde_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xrk10Tfi(3,1))';
                        % FEJ EKF
                        JHX10e_fej((R1kj-1)*2+(1:2), R1XfksFej_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi_fej(3,3))';

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

                    % Ideal EKF
                    HX10e_ide(size(R1Zks2,1)+(1:2:(size(R2Zks2,1)-1)),1) = cos(Xk10efi_ide(6,3))*(R2Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(4,3)) + ...
                        sin(Xk10efi_ide(6,3))*(R2Xfks_ide(2:2:end,2)-Xk10efi_ide(5,3));
                    HX10e_ide(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),1) = -sin(Xk10efi_ide(6,3))*(R2Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(4,3)) + ...
                        cos(Xk10efi_ide(6,3))*(R2Xfks_ide(2:2:end,2)-Xk10efi_ide(5,3));

                    JHX10e_ide(size(R1Zks2,1)+(1:2:size(R2Zks2,1)-1),4:6) = [repmat([-cos(Xrk10Tfi(6,1)),-sin(Xrk10Tfi(6,1))],size(R2Zks2,1)/2,1), ...
                        -sin(Xrk10Tfi(6,1))*(R2XfksT(1:2:(end-1),2)-Xrk10Tfi(4,1))+cos(Xrk10Tfi(6,1))*(R2XfksT(2:2:end,2)-Xrk10Tfi(5,1))];

                    JHX10e_ide(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),4:6) = [repmat([sin(Xrk10Tfi(6,1)),-cos(Xrk10Tfi(6,1))],size(R2Zks2,1)/2,1), ...
                        -cos(Xrk10Tfi(6,1))*(R2XfksT(1:2:(end-1),2)-Xrk10Tfi(4,1))-sin(Xrk10Tfi(6,1))*(R2XfksT(2:2:end,2)-Xrk10Tfi(5,1))];

                    % FEJ EKF
                    HX10e_fej(size(R1Zks2,1)+(1:2:(size(R2Zks2,1)-1)),1) = cos(Xk10efi_fej(6,3))*(R2Xfks_fej(1:2:(end-1),2)-Xk10efi_fej(4,3)) + ...
                        sin(Xk10efi_fej(6,3))*(R2Xfks_fej(2:2:end,2)-Xk10efi_fej(5,3));
                    HX10e_fej(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),1) = -sin(Xk10efi_fej(6,3))*(R2Xfks_fej(1:2:(end-1),2)-Xk10efi_fej(4,3)) + ...
                        cos(Xk10efi_fej(6,3))*(R2Xfks_fej(2:2:end,2)-Xk10efi_fej(5,3));

                    JHX10e_fej(size(R1Zks2,1)+(1:2:size(R2Zks2,1)-1),4:6) = [repmat([-cos(Xk10efi_fej(6,3)),-sin(Xk10efi_fej(6,3))],size(R2Zks2,1)/2,1), ...
                        -sin(Xk10efi_fej(6,3))*(R2Xfks_fe(1:2:(end-1),2)-Xk10efi_fej(4,3))+cos(Xk10efi_fej(6,3))*(R2Xfks_fe(2:2:end,2)-Xk10efi_fej(5,3))];

                    JHX10e_fej(size(R1Zks2,1)+(2:2:size(R2Zks2,1)),4:6) = [repmat([sin(Xk10efi_fej(6,3)),-cos(Xk10efi_fej(6,3))],size(R2Zks2,1)/2,1), ...
                        -cos(Xk10efi_fej(6,3))*(R2Xfks_fe(1:2:(end-1),2)-Xk10efi_fej(4,3))-sin(Xk10efi_fej(6,3))*(R2Xfks_fe(2:2:end,2)-Xk10efi_fej(5,3))];

                    for R2kj = 1:size(R2Zks2,1)/2
                        R2DV = blkdiag(R2DV,R2R);

                        % Standard EKF
                        JHX10e(size(R1Zks2,1)+(R2kj-1)*2+(1:2), R2Xfks_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(6,3))';
                        % Ideal EKF
                        JHX10e_ide(size(R1Zks2,1)+(R2kj-1)*2+(1:2), R2XfksIde_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xrk10Tfi(6,1))';
                        % FEJ EKF
                        JHX10e_fej(size(R1Zks2,1)+(R2kj-1)*2+(1:2), R2XfksFej_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi_fej(6,3))';
                    end
                end

                DV = blkdiag(R1DV,R2DV);

                %% Innovation Covariance S and Kalman Gain K
                % Standard EKF
                Ss = JHX10e * Pk10fi * JHX10e' + DV;
                Ks = Pk10fi * JHX10e' /Ss;

                % Ideal EKF
                Ssi = JHX10e_ide * Pk10fi_ide * JHX10e_ide' + DV;
                Ksi = Pk10fi_ide * JHX10e_ide' /Ssi;

                % FEJ EKF
                Ssf = JHX10e_fej * Pk10fi_fej * JHX10e_fej' + DV;
                Ksf = Pk10fi_fej * JHX10e_fej' /Ssf;

                %% Updating process using observation model
                % Standard EKF
                Xk11e(:,3) = Xk10efi(:,3) + Ks*(Zks(:,3)-HX10e);
                Xk11e([3,6],3) = wrap(Xk11e([3,6],3));
                Pk11 = Pk10fi - Ks*Ss*Ks';
                Pk11(abs(Pk11)<CovT) = 0;

                % Ideal EKF
                Xk11e_ide(:,3) = Xk10efi_ide(:,3) + Ksi*(Zks(:,3)-HX10e_ide);
                Xk11e_ide([3,6],3) = wrap(Xk11e_ide([3,6],3));
                Pk11_ide = Pk10fi_ide - Ksi*Ssi*Ksi';
                Pk11_ide(abs(Pk11_ide)<CovT) = 0;

                % FEJ EKF
                Xk11e_fej(:,3) = Xk10efi_fej(:,3) + Ksf*(Zks(:,3)-HX10e_fej);
                Xk11e_fej([3,6],3) = wrap(Xk11e_fej([3,6],3));
                Pk11_fej = Pk10fi_fej - Ksf*Ssf*Ksf';
                Pk11_fej(abs(Pk11_fej)<CovT) = 0;
            end

            % Standard EKF
            R1XpFull = [R1XpFull;Xk11e(1:3,2:3)];
            R1PFull = [R1PFull;Pk11(1:3,1:3)];

            R2XpFull = [R2XpFull;Xk11e(4:6,2:3)];
            R2PFull = [R2PFull;Pk11(4:6,4:6)];

            Xk00e = Xk11e;
            Pk00 = Pk11;

            % Ideal EKF
            R1XpIdeFull = [R1XpIdeFull;Xk11e_ide(1:3,2:3)];
            R1PIdeFull = [R1PIdeFull;Pk11_ide(1:3,1:3)];

            R2XpIdeFull = [R2XpIdeFull;Xk11e_ide(4:6,2:3)];
            R2PIdeFull = [R2PIdeFull;Pk11_ide(4:6,4:6)];

            Xk00e_ide = Xk11e_ide;
            Pk00_ide = Pk11_ide;

            % FEJ EKF
            R1XpFejFull = [R1XpFejFull;Xk11e_fej(1:3,2:3)];
            R1PFejFull = [R1PFejFull;Pk11_fej(1:3,1:3)];

            R2XpFejFull = [R2XpFejFull;Xk11e_fej(4:6,2:3)];
            R2PFejFull = [R2PFejFull;Pk11_fej(4:6,4:6)];

            Xk00e_fej = Xk11e_fej;
            Pk00_fej = Pk11_fej;
        end

        % Standard EKF
        R1XpFullSet = [R1XpFullSet,R1XpFull(:,2)];
        R1PFullSet = [R1PFullSet,R1PFull];

        R2XpFullSet = [R2XpFullSet,R2XpFull(:,2)];
        R2PFullSet = [R2PFullSet,R2PFull];

        XfFullSet = [XfFullSet,Xk11e(7:end,3)];
        PfFullSet = [PfFullSet,Pk11(7:end,7:end)];

        % Ideal EKF
        R1XpIdeFullSet = [R1XpIdeFullSet,R1XpIdeFull(:,2)];
        R1PIdeFullSet = [R1PIdeFullSet,R1PIdeFull];

        R2XpIdeFullSet = [R2XpIdeFullSet,R2XpIdeFull(:,2)];
        R2PIdeFullSet = [R2PIdeFullSet,R2PIdeFull];

        XfIdeFullSet = [XfIdeFullSet,Xk11e_ide(7:end,3)];
        PfIdeFullSet = [PfIdeFullSet,Pk11_ide(7:end,7:end)];

        % FEJ EKF
        R1XpFejFullSet = [R1XpFejFullSet,R1XpFejFull(:,2)];
        R1PFejFullSet = [R1PFejFullSet,R1PFejFull];

        R2XpFejFullSet = [R2XpFejFullSet,R2XpFejFull(:,2)];
        R2PFejFullSet = [R2PFejFullSet,R2PFejFull];

        XfFejFullSet = [XfFejFullSet,Xk11e_fej(7:end,3)];
        PfFejFullSet = [PfFejFullSet,Pk11_fej(7:end,7:end)];

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

    %% Ideal EKF
    R1XpIdeFullSet = [R1XpIdeFull(:,1),R1XpIdeFullSet];
    R2XpIdeFullSet = [R2XpIdeFull(:,1),R2XpIdeFullSet];
    XfIdeFullSet = [Xk11e_ide(7:end,2),XfIdeFullSet];

    R1XrIdeFullSet = [];
    R2XrIdeFullSet = [];
    R1XphiIdeFullSet = [];
    R2XphiIdeFullSet = [];

    DeltaR1XpIdeFullSet = [];
    DeltaR2XpIdeFullSet = [];

    DeltaR1XrIdeFullSet = [];
    DeltaR2XrIdeFullSet = [];

    DeltaR1XphiIdeFullSet = [];
    DeltaR2XphiIdeFullSet = [];

    %% FEJ EKF
    R1XpFejFullSet = [R1XpFejFull(:,1),R1XpFejFullSet];
    R2XpFejFullSet = [R2XpFejFull(:,1),R2XpFejFullSet];
    XfFejFullSet = [Xk11e_fej(7:end,2),XfFejFullSet];

    R1XrFejFullSet = [];
    R2XrFejFullSet = [];
    R1XphiFejFullSet = [];
    R2XphiFejFullSet = [];

    DeltaR1XpFejFullSet = [];
    DeltaR2XpFejFullSet = [];

    DeltaR1XrFejFullSet = [];
    DeltaR2XrFejFullSet = [];

    DeltaR1XphiFejFullSet = [];
    DeltaR2XphiFejFullSet = [];

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

        %% Ideal EKF
        DeltaR1XpIdeFullSet((end+1):(end+3),:) = [R1XpTrue(:,1),R1XpIdeFullSet(pn*3+(1:3),2:end)-R1XpTrue(:,2)];
        DeltaR2XpIdeFullSet((end+1):(end+3),:) = [R2XpTrue(:,1),R2XpIdeFullSet(pn*3+(1:3),2:end)-R2XpTrue(:,2)];

        % wrap the delta angle
        DeltaR1XpIdeFullSet(end,2:end) = wrap(DeltaR1XpIdeFullSet(end,2:end));
        DeltaR2XpIdeFullSet(end,2:end) = wrap(DeltaR2XpIdeFullSet(end,2:end));

        DeltaR1XrIdeFullSet((end+1):(end+2),:) = DeltaR1XpIdeFullSet((end-2):(end-1),:);
        DeltaR2XrIdeFullSet((end+1):(end+2),:) = DeltaR2XpIdeFullSet((end-2):(end-1),:);

        DeltaR1XphiIdeFullSet(end+1,:) = DeltaR1XpIdeFullSet(end,:);
        DeltaR2XphiIdeFullSet(end+1,:) = DeltaR2XpIdeFullSet(end,:);

        %% FEJ EKF
        DeltaR1XpFejFullSet((end+1):(end+3),:) = [R1XpTrue(:,1),R1XpFejFullSet(pn*3+(1:3),2:end)-R1XpTrue(:,2)];
        DeltaR2XpFejFullSet((end+1):(end+3),:) = [R2XpTrue(:,1),R2XpFejFullSet(pn*3+(1:3),2:end)-R2XpTrue(:,2)];

        % wrap the delta angle
        DeltaR1XpFejFullSet(end,2:end) = wrap(DeltaR1XpFejFullSet(end,2:end));
        DeltaR2XpFejFullSet(end,2:end) = wrap(DeltaR2XpFejFullSet(end,2:end));

        DeltaR1XrFejFullSet((end+1):(end+2),:) = DeltaR1XpFejFullSet((end-2):(end-1),:);
        DeltaR2XrFejFullSet((end+1):(end+2),:) = DeltaR2XpFejFullSet((end-2):(end-1),:);

        DeltaR1XphiFejFullSet(end+1,:) = DeltaR1XpFejFullSet(end,:);
        DeltaR2XphiFejFullSet(end+1,:) = DeltaR2XpFejFullSet(end,:);

    end

    %% re-order the true features XfTrueAll's IDs to make it consistent with FFullSet 每一步的
    [~,XfTrue_idx] = ismember(XfFullSet(:,1),XfTrueAll(:,1));
    XfTrue_idx(2:2:end,1) = XfTrue_idx(2:2:end,1)+1;
    XfTrue = XfTrueAll(XfTrue_idx,:);
    % Standard EKF
    DeltaXfFullSet = [XfFullSet(:,1),XfFullSet(:,2:end)-XfTrue(:,2)];
    % Ideal EKF
    DeltaXfIdeFullSet = [XfIdeFullSet(:,1),XfIdeFullSet(:,2:end)-XfTrue(:,2)];
    % FEJ EKF
    DeltaXfFejFullSet = [XfIdeFullSet(:,1),XfFejFullSet(:,2:end)-XfTrue(:,2)];

    %% save the Monte Carlo Experiments result
    if i == 1
        save('MTE_results_StdEKF_20fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ... 
            'DeltaXfFullSet','PfFullSet')
        save('MTE_results_IdeEKF_20fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        save('MTE_results_FejEKF_20fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ... 
            'DeltaXfFejFullSet','PfFejFullSet')
    elseif i == 2
        save('MTE_results_StdEKF_60fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ... 
            'DeltaXfFullSet','PfFullSet')
        save('MTE_results_IdeEKF_60fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        save('MTE_results_FejEKF_60fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ... 
            'DeltaXfFejFullSet','PfFejFullSet')
    else
        save('MTE_results_StdEKF_100fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ... 
            'DeltaXfFullSet','PfFullSet')
        save('MTE_results_IdeEKF_100fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        save('MTE_results_FejEKF_100fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ... 
            'DeltaXfFejFullSet','PfFejFullSet')
    end
end