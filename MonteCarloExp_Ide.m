clear

Config;

if ec == 1
    if env == 1
        load('MT_Parameters_20fea_1.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_20fea_1.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    else
        load('MT_Parameters_20fea_2.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_20fea_2.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    end
elseif ec == 2
    if env == 1
        load('MT_Parameters_60fea_1.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_60fea_1.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    else
        load('MT_Parameters_60fea_2.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_60fea_2.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    end
elseif ec == 3
    if env == 1
        load('MT_Parameters_100fea_1.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_100fea_1.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    else
        load('MT_Parameters_100fea_2.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        load('MT_Measurements_100fea_2.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    end
elseif ec == 4
    error('No real-world experiment uses truth!')
    % load('VicP_Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
    % load('VicP_Measurements.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    % mcNum = 1;
end


poseNum = size(R1OdoSet,1)/3;

% Ideal EKF
R1XpIdeFullSet = [];
R1PIdeFullSet = [];
R2XpIdeFullSet = [];
R2PIdeFullSet = [];
XfIdeFullSet = [];
PfIdeFullSet = [];

R1Xp0T = [R1XrTrue(1:2,:);R1XphiT(1,:)];
R2Xp0T = [R2XrTrue(1:2,:);R2XphiT(1,:)];

for mc = 1:mcNum

    R1Xp0 = R1Xp0Set(:,mc);
    R2Xp0 = R2Xp0Set(:,mc);

    R1Odo = R1OdoSet(:,[1,2,2+mc]);
    R2Odo = R2OdoSet(:,[1,2,2+mc]);

    R1Obs = R1ObsSet(:,[1,2,2+mc]);
    R2Obs = R2ObsSet(:,[1,2,2+mc]);

    % Ideal EKF
    R1XpIdeFull = [];
    R1PIdeFull = [];
    R2XpIdeFull = [];
    R2PIdeFull = [];
    PfIdeFull = [];

    Xs = [];
    X0 = [];

    % Ideal EKF
    Xk00e_ide = [];
    Xk10e_ide = [];
    Xk10efi_ide = [];
    Xk11e_ide = [];

    for k = 0:poseNum

        % k

        % Standard & EFJ EKF & Ideal EKF
        R1Obs_k = R1Obs(R1Obs(:,1)==k,2:3);
        R2Obs_k = R2Obs(R2Obs(:,1)==k,2:3);

        % Ideal EKF
        R1ObsT_k = R1ObsT(R1ObsT(:,1)==k,2:3);
        R2ObsT_k = R2ObsT(R2ObsT(:,1)==k,2:3);

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
            R1Z0sT = R1ObsT_k(R1Z0s_idx,:);

            R1Xf0s = R1Z0s;
            R1Xf0s(1:2:(end-1),2) = R1Xp0(1,1) + cos(R1Xp0(3,1))*(R1Z0s(1:2:(end-1),2)) - sin(R1Xp0(3,1))*(R1Z0s(2:2:end,2));
            R1Xf0s(2:2:end,2) = R1Xp0(2,1) + sin(R1Xp0(3,1))*(R1Z0s(1:2:(end-1),2)) + cos(R1Xp0(3,1))*(R1Z0s(2:2:end,2));


            R1Xf0sT_lv = ismember(XfTrueAll(:,1),R1Xf0s(:,1));
            R1Xf0sT_idx = find(R1Xf0sT_lv);
            R1Xf0sT = XfTrueAll(R1Xf0sT_idx,:);
            % find the shared observed feature IDs in 2nd robot
            % R2Z0s_lv: logical vector of shared feature observation of
            % 2nd robot at step k
            R2Z0s_lv = ismember(R2Obs_k(:,1), R1Obs_k(:,1));
            % R2Z0s_idx: index of shared feature observation of
            % 2nd robot in R2Obs_k
            R2Z0s_idx = find(R2Z0s_lv);
            % R2Z0s: shared feature observation of 2nd robot
            R2Z0s = R2Obs_k(R2Z0s_idx,:);
            R2Z0sT = R2ObsT_k(R2Z0s_idx,:);



            %% use Gauss-Newton iteration to optimize the shared state at step 0
            % use R2Xp0 as the initial value of R2's pose in the GN iteration
            Xs = [2*ones(3,1),zeros(3,1),R2Xp0;
                zeros(size(R1Xf0s,1),1),R1Xf0s];

            XsT = [2*ones(3,1),zeros(3,1),R2Xp0T(:,2);
                zeros(size(R1Xf0s,1),1),R1Xf0sT];

            R1sRn = [];
            R2sRn = [];
            for kj = 1:(size(R1Xf0s,1)/2)
                R1sRn = blkdiag(R1sRn,R1R);
                R2sRn = blkdiag(R2sRn,R2R);
            end
            Pz = blkdiag(R1sRn,R2sRn);

            Z0s = [R1Z0s(:,2);R2Z0s(:,2)];

            Z0sT = [R1Z0sT(:,2);R2Z0sT(:,2)];

            Xs(3,1) = wrap(Xs(3,1));
            XsGni = Xs;

            % The result of displaying GNI as a singular matrix is due to
            % the fact that the R2Xp0 randomly generated by adding noise 
            % falls on the true value of the feature

            %%
            if step0GNI_ide == 0
                [XsGni(:,3),PzGni] = GNI(R1Xp0,Xs(:,3),Pz,Z0s,CC);
            else
                [XsGni(:,3),PzGni] = GNI_Ide(R1Xp0,R1Xp0T(:,2),Xs(:,3),XsT(:,3),Pz,Z0s,Z0sT,CC);
            end
            %%

            XsGni(3,1) = wrap(XsGni(3,1));

            % Set the elements that are less than CovT to zero. This can be useful for dealing with numerical errors or avoiding unnecessary imaginary parts in calculations.
            PzGni(abs(PzGni)<CovT) = 0;

            % Add the information of R1 into XsGni and PsGni
            X0 = [ones(3,1),zeros(3,1),R1Xp0;XsGni];
            P0 = blkdiag(R1O,PzGni);



            %% estimate new observed feature's state at step 0 using the observation model
            % find the new observed feature IDs in 1st robot
            % R1Zkn_lv: logical vector of new feature observation of
            % 2nd robot at step k
            R1ZknIde_lv = ~ismember(R1Obs_k(:,1),X0(7:end,2));
            % R1Zkn_idx: index of new feature observation of
            % 1st robot in R1Obs_k
            R1ZknIde_idx = find(R1ZknIde_lv);
            % R1Z0n: new feature observation of 1st robot
            R1Z0n = R1Obs_k(R1ZknIde_idx,:);

            R1XfknT_lv = ismember(XfTrueAll(:,1),R1Z0n(:,1));
            R1XfknT_idx = find(R1XfknT_lv);
            R1XfknT = XfTrueAll(R1XfknT_idx,:);

            % if ~isempty(R1Z0n)
            %     keyboard;
            % end

            R1Xfkn = R1Z0n;
            R1Xfkn(1:2:(end-1),2) = X0(1,3) + cos(X0(3,3))*(R1Z0n(1:2:(end-1),2)) - sin(X0(3,3))*(R1Z0n(2:2:end,2));
            R1Xfkn(2:2:end,2) = X0(2,3) + sin(X0(3,3))*(R1Z0n(1:2:(end-1),2)) + cos(X0(3,3))*(R1Z0n(2:2:end,2));

            % find the new observed feature IDs in 2nd robot
            % R2Zkn_lv: logical vector of new feature observation of
            % 2nd robot at step k
            R2ZknIde_lv = ~ismember(R2Obs_k(:,1),X0(7:end,2));
            % R2Zkn_idx: index of new feature observation of
            % 2nd robot in R2Obs_k
            R2ZknIde_idx = find(R2ZknIde_lv);
            % R2Z0n: new feature observation of 2nd robot
            R2Z0n = R2Obs_k(R2ZknIde_idx,:);

            R2XfknT_lv = ismember(XfTrueAll(:,1),R2Z0n(:,1));
            R2XfknT_idx = find(R2XfknT_lv);
            R2XfknT = XfTrueAll(R2XfknT_idx,:);

            R2Xfkn = R2Z0n;
            R2Xfkn(1:2:(end-1),2) = X0(4,3) + cos(X0(6,3))*(R2Z0n(1:2:(end-1),2)) - sin(X0(6,3))*(R2Z0n(2:2:end,2));
            R2Xfkn(2:2:end,2) = X0(5,3) + sin(X0(6,3))*(R2Z0n(1:2:(end-1),2)) + cos(X0(6,3))*(R2Z0n(2:2:end,2));

            % Xk00e: state estimate
            % first column: 1 -> robot posture; 2 -> feature position
            % second column: posture id or position id
            % third column: data
            Xk00e_ide = [X0;
                ones(size(R1Xfkn,1),1),R1Xfkn;
                2*ones(size(R2Xfkn,1),1),R2Xfkn];

            if ~isempty(R1Xfkn) || ~isempty(R2Xfkn)
                % Jacobian of Xk
                JFXk = sparse(size(Xk00e_ide,1),size(X0,1));
                JFXk(1:size(X0,1),1:size(X0,1)) = eye(size(X0,1));

                %% COV
                if step0FI_ide == 0
                    % use optimized X0
                    JFXk(size(X0,1)+(1:2:(size(R1Xfkn,1)-1)),1:3) = [repmat([1,0],size(R1Xfkn,1)/2,1), -sin(X0(3,3))*R1Z0n(1:2:(end-1),2) - cos(X0(3,3))*R1Z0n(2:2:end,2)];
                    JFXk(size(X0,1)+(2:2:size(R1Xfkn,1)),1:3) = [repmat([0,1],size(R1Xfkn,1)/2,1), cos(X0(3,3))*R1Z0n(1:2:(end-1),2) - sin(X0(3,3))*R1Z0n(2:2:end,2)];

                    JFXk(size(X0,1)+size(R1Xfkn,1)+(1:2:(size(R2Xfkn,1)-1)),4:6) = [repmat([1,0],size(R2Xfkn,1)/2,1), -sin(X0(6,3))*R2Z0n(1:2:(end-1),2) - cos(X0(6,3))*R2Z0n(2:2:end,2)];
                    JFXk(size(X0,1)+size(R1Xfkn,1)+(2:2:size(R2Xfkn,1)),4:6) = [repmat([0,1],size(R2Xfkn,1)/2,1), cos(X0(6,3))*R2Z0n(1:2:(end-1),2) - sin(X0(6,3))*R2Z0n(2:2:end,2)];
                else
                    % use truth
                    JFXk(size(X0,1)+(1:2:(size(R1Xfkn,1)-1)),1:3) = [repmat([1,0],size(R1Xfkn,1)/2,1), -(R1XfknT(2:2:end,2)-R1Xp0T(2,2))];
                    JFXk(size(X0,1)+(2:2:size(R1Xfkn,1)),1:3) = [repmat([0,1],size(R1Xfkn,1)/2,1), R1XfknT(1:2:(end-1),2)-R1Xp0T(1,2)];

                    JFXk(size(X0,1)+size(R1Xfkn,1)+(1:2:(size(R2Xfkn,1)-1)),4:6) = [repmat([1,0],size(R2Xfkn,1)/2,1), -(R2XfknT(2:2:end,2)-R2Xp0T(2,2))];
                    JFXk(size(X0,1)+size(R1Xfkn,1)+(2:2:size(R2Xfkn,1)),4:6) = [repmat([0,1],size(R2Xfkn,1)/2,1), R2XfknT(1:2:(end-1),2)-R2Xp0T(1,2)];
                end
                %%

                R1nRn_ide = [];
                R2nRn_ide = [];
                JFWk = sparse(size(Xk00e_ide,1),size(R1Z0n,1)+size(R2Z0n,1));
                for R1jn = 1:(size(R1Z0n,1)/2)
                    R1nRn_ide = blkdiag(R1nRn_ide, R1R);
                    JFWk(size(X0,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = -Rot(X0(3,3));
                end
                for R2jn_ide = 1:(size(R2Z0n,1)/2)
                    R2nRn_ide = blkdiag(R2nRn_ide, R2R);
                    JFWk(size(X0,1)+size(R1Z0n,1)+(R2jn_ide-1)*2+(1:2),size(R1Z0n,1)+(R2jn_ide-1)*2+(1:2)) = -Rot(X0(6,3));
                end

                nRn_ide = blkdiag(R1nRn_ide,R2nRn_ide);
                Pk00_ide = JFXk*P0*JFXk'+JFWk*nRn_ide*JFWk';

                Pk00_ide(abs(Pk00_ide)<CovT) = 0;

            else
                Pk00_ide = P0;
            end

            % Ideal EKF
            % R1XpIdeFull = Xk00e_ide(1:3,2:3); % save all robot postures of R1
            R2XpIdeFull = Xk00e_ide(4:6,2:3); % save all robot postures of R2

            % R1PIdeFull = Pk00_ide(1:3,1:3);
            R2PIdeFull = Pk00_ide(4:6,4:6);

            continue
        end

        % if k == 482
        %     keyboard
        % end

        %% Prediction using the motion model

        R1Odo_k = R1Odo(R1Odo(:,2)==k,3);
        R2Odo_k = R2Odo(R2Odo(:,2)==k,3);

        % Ideal EKF
        % R1OdoT_k = R1OdoT(R1OdoT(:,2)==k,3);
        % R2OdoT_k = R2OdoT(R2OdoT(:,2)==k,3);

        % if k == 1
        %     Xk00e_ide = Xk00e_ide;
        %     Pk00_ide = Pk00_ide;
        % end

        Xk10e_ide = Xk00e_ide;
        Xk10e_ide(1:6,2) = Xk10e_ide(1:6,2)+1;
        Xk10e_ide(1:3,3) = Xk00e_ide(1:3,3) + ...
            [cos(Xk00e_ide(3,3))*R1Odo_k(1,1) - sin(Xk00e_ide(3,3))*R1Odo_k(2,1);
            sin(Xk00e_ide(3,3))*R1Odo_k(1,1) + cos(Xk00e_ide(3,3))*R1Odo_k(2,1);
            R1Odo_k(3,1)];
        Xk10e_ide(4:6,3) = Xk00e_ide(4:6,3) + ...
            [cos(Xk00e_ide(6,3))*R2Odo_k(1,1) - sin(Xk00e_ide(6,3))*R2Odo_k(2,1);
            sin(Xk00e_ide(6,3))*R2Odo_k(1,1) + cos(Xk00e_ide(6,3))*R2Odo_k(2,1);
            R2Odo_k(3,1)];

        Xk10e_ide([3,6],3) = wrap(Xk10e_ide([3,6],3));

        Xrk00T = [R1XrTrue(R1XrTrue(:,1)==k-1,2);R1XphiT(R1XphiT(:,1)==k-1,2); ...
            R2XrTrue(R2XrTrue(:,1)==k-1,2);R2XphiT(R2XphiT(:,1)==k-1,2)];

        Xrk10T = [R1XrTrue(R1XrTrue(:,1)==k,2);R1XphiT(R1XphiT(:,1)==k,2); ...
            R2XrTrue(R2XrTrue(:,1)==k,2);R2XphiT(R2XphiT(:,1)==k,2)];

        %%
        DeltaXfX_ide = sparse(size(Xk10e_ide,1),size(Xk00e_ide,1));
        DeltaXfX_ide(1:6,1:6) = blkdiag([eye(2),J*(Xrk10T(1:2,1)-Xrk00T(1:2,1));
            zeros(1,2),1], ...
            [eye(2),J*(Xrk10T(4:5,1)-Xrk00T(4:5,1));
            zeros(1,2),1]);
        DeltaXfX_ide(7:end,7:end) = eye(size(DeltaXfX_ide(7:end,7:end)));

        % DeltaXfX_ide = sparse(size(Xk10e_ide,1),size(Xk00e_ide,1));
        % DeltaXfX_ide(1:6,1:6) = blkdiag([1,0,-sin(Xrk00T(3,1))*R1OdoT_k(1,1) - cos(Xrk00T(3,1))*R1OdoT_k(2,1); ...
        %     0,1,cos(Xrk00T(3,1))*R1OdoT_k(1,1) - sin(Xrk00T(3,1))*R1OdoT_k(2,1); ...
        %     0,0,1], ...
        %     [1,0,-sin(Xrk00T(6,1))*R2OdoT_k(1,1) - cos(Xrk00T(6,1))*R2OdoT_k(2,1); ...
        %     0,1,cos(Xrk00T(6,1))*R2OdoT_k(1,1) - sin(Xrk00T(6,1))*R2OdoT_k(2,1); ...
        %     0,0,1]);
        % DeltaXfX_ide(7:end,7:end) = eye(size(DeltaXfX_ide(7:end,7:end)));

        %%
        % DeltaXfW_ide = sparse(size(Xk00e_ide,1),6);
        % DeltaXfW_ide(1:6,1:6) = blkdiag([cos(Xrk00T(3,1)),-sin(Xrk00T(3,1)),0; ...
        %     sin(Xrk00T(3,1)),cos(Xrk00T(3,1)),0; ...
        %     0,0,1], ...
        %     [cos(Xrk00T(6,1)),-sin(Xrk00T(6,1)),0; ...
        %     sin(Xrk00T(6,1)),cos(Xrk00T(6,1)),0; ...
        %     0,0,1]);

        DeltaXfW_ide = sparse(size(Xk00e_ide,1),6);
        DeltaXfW_ide(1:6,1:6) = blkdiag([cos(Xk00e_ide(3,3)),-sin(Xk00e_ide(3,3)),0; ...
            sin(Xk00e_ide(3,3)),cos(Xk00e_ide(3,3)),0; ...
            0,0,1], ...
            [cos(Xk00e_ide(6,3)),-sin(Xk00e_ide(6,3)),0; ...
            sin(Xk00e_ide(6,3)),cos(Xk00e_ide(6,3)),0; ...
            0,0,1]);
        %%

        DWk = blkdiag(R1Q,R2Q);

        Pk10_ide = DeltaXfX_ide * Pk00_ide * DeltaXfX_ide' + DeltaXfW_ide * DWk * DeltaXfW_ide';
        Pk10_ide(abs(Pk10_ide)<CovT) = 0;



        %% Feature initialization using new feature observations from R1 and R2
        % find the feature observations in R1 from new features of Xk10e
        R1ZknIde_lv = ~ismember(R1Obs_k(:,1),Xk10e_ide(7:end,2));
        R1ZknIde_idx = find(R1ZknIde_lv);
        R1ZknIde = R1Obs_k(R1ZknIde_idx,:);

        R1ZknIdeT = R1ObsT_k(R1ZknIde_idx,:);

        % find the feature observations in R2 from new features of Xk10e
        R2ZknIde_lv = ~ismember(R2Obs_k(:,1),Xk10e_ide(7:end,2));
        R2ZknIde_idx = find(R2ZknIde_lv);
        R2ZknIde = R2Obs_k(R2ZknIde_idx,:);

        R2ZknIdeT = R2ObsT_k(R2ZknIde_idx,:);

        % What if R1 and R2 both see the same new feature
        ZknsIde_ID = intersect(R1ZknIde(:,1),R2ZknIde(:,1));
        % Use R1 for initialization, remove R2 from R2Zkn, and use it for updates later
        R2ZknsIde = [];
        if ~isempty(ZknsIde_ID)
            for ZknsNum = 1:size(ZknsIde_ID,1)
                R2ZknsIde = [R2ZknsIde;R2ZknIde(R2ZknIde(:,1)==ZknsIde_ID(ZknsNum,1),:)];
                R2ZknIde(R2ZknIde(:,1)==ZknsIde_ID(ZknsNum,1),:) = [];

                R2ZknIdeT(R2ZknIdeT(:,1)==ZknsIde_ID(ZknsNum,1),:) = [];
            end
        end

        ZknIde = [R1ZknIde;R2ZknIde];

        if ~isempty(ZknIde)
            % Ideal EKF

            DeltaGX_ide = sparse(size(Xk10e_ide,1)+size(ZknIde,1),size(Xk10e_ide,1));
            DeltaGX_ide(1:size(Xk10e_ide,1),1:size(Xk10e_ide,1)) = eye(size(Xk10e_ide,1));

            % Observation noise Cov
            R1nRn_ide = [];
            R2nRn_ide = [];

            DeltaGV_ide = sparse(size(Xk10e_ide,1)+size(ZknIde,1),size(ZknIde,1));

            R1Xfn_ide = [];
            R2Xfn_ide = [];

            if ~isempty(R1ZknIde)
                % Ideal EKF
                R1Xfn_ide = R1ZknIde;

                R1Xfn_ide(1:2:(end-1),2) = Xk10e_ide(1,3) + cos(Xk10e_ide(3,3))*R1ZknIde(1:2:(end-1),2) - sin(Xk10e_ide(3,3))*R1ZknIde(2:2:end,2);
                R1Xfn_ide(2:2:end,2) = Xk10e_ide(2,3) + sin(Xk10e_ide(3,3))*R1ZknIde(1:2:(end-1),2) + cos(Xk10e_ide(3,3))*R1ZknIde(2:2:end,2);

                if stepkFI_ide == 0
                    %% Cov
                    DeltaGX_ide(size(Xk10e_ide,1)+(1:2:(size(R1Xfn_ide,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn_ide,1)/2,1), ...
                        -sin(Xk10e_ide(3,3))*R1ZknIde(1:2:(end-1),2)-cos(Xk10e_ide(3,3))*R1ZknIde(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+(2:2:size(R1Xfn_ide,1)),1:3) = [repmat([0, 1],size(R1Xfn_ide,1)/2,1), ...
                        cos(Xk10e_ide(3,3))*R1ZknIde(1:2:(end-1),2)-sin(Xk10e_ide(3,3))*R1ZknIde(2:2:end,2)];
                else
                    DeltaGX_ide(size(Xk10e_ide,1)+(1:2:(size(R1Xfn_ide,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn_ide,1)/2,1), ...
                        -sin(Xrk10T(3,1))*R1ZknIdeT(1:2:(end-1),2)-cos(Xrk10T(3,1))*R1ZknIdeT(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+(2:2:size(R1Xfn_ide,1)),1:3) = [repmat([0, 1],size(R1Xfn_ide,1)/2,1), ...
                        cos(Xrk10T(3,1))*R1ZknIdeT(1:2:(end-1),2)-sin(Xrk10T(3,1))*R1ZknIdeT(2:2:end,2)];
                end
                %%
                for R1jn_ide = 1:(size(R1ZknIde,1)/2)
                    R1nRn_ide = blkdiag(R1nRn_ide, R1R);
                    DeltaGV_ide(size(Xk10e_ide,1)+(R1jn_ide-1)*2+(1:2),(R1jn_ide-1)*2+(1:2)) = -Rot(Xk10e_ide(3,3));
                end

                % for R1jn_ide = 1:(size(R1ZknIde,1)/2)
                %     R1nRn_ide = blkdiag(R1nRn_ide, R1R);
                %     DeltaGV_ide(size(Xk10e_ide,1)+(R1jn_ide-1)*2+(1:2),(R1jn_ide-1)*2+(1:2)) = Rot(Xrk10T(3,1));
                % end
            end

            if ~isempty(R2ZknIde)
                % Ideal EKF
                R2Xfn_ide = R2ZknIde;

                R2Xfn_ide(1:2:(end-1),2) = Xk10e_ide(4,3) + cos(Xk10e_ide(6,3))*R2ZknIde(1:2:(end-1),2) - sin(Xk10e_ide(6,3))*R2ZknIde(2:2:end,2);
                R2Xfn_ide(2:2:end,2) = Xk10e_ide(5,3) + sin(Xk10e_ide(6,3))*R2ZknIde(1:2:(end-1),2) + cos(Xk10e_ide(6,3))*R2ZknIde(2:2:end,2);

                %% Cov
                if stepkFI_ide == 0
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(1:2:(size(R2Xfn_ide,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn_ide,1)/2,1), ...
                        -sin(Xk10e_ide(6,3))*R2ZknIde(1:2:(end-1),2)-cos(Xk10e_ide(6,3))*R2ZknIde(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(2:2:size(R2Xfn_ide,1)),4:6) = [repmat([0, 1],size(R2Xfn_ide,1)/2,1), ...
                        cos(Xk10e_ide(6,3))*R2ZknIde(1:2:(end-1),2)-sin(Xk10e_ide(6,3))*R2ZknIde(2:2:end,2)];
                else
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(1:2:(size(R2Xfn_ide,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn_ide,1)/2,1), ...
                        -sin(Xrk10T(6,1))*R2ZknIdeT(1:2:(end-1),2)-cos(Xrk10T(6,1))*R2ZknIdeT(2:2:end,2)];
                    DeltaGX_ide(size(Xk10e_ide,1)+size(R1Xfn_ide,1)+(2:2:size(R2Xfn_ide,1)),4:6) = [repmat([0, 1],size(R2Xfn_ide,1)/2,1), ...
                        cos(Xrk10T(6,1))*R2ZknIdeT(1:2:(end-1),2)-sin(Xrk10T(6,1))*R2ZknIdeT(2:2:end,2)];
                end
                %%
                for R2jn_ide = 1:(size(R2ZknIde,1)/2)
                    R2nRn_ide = blkdiag(R2nRn_ide, R2R);
                    % Ideal EKF
                    DeltaGV_ide(size(Xk10e_ide,1)+size(R1ZknIde,1)+(R2jn_ide-1)*2+(1:2),size(R1ZknIde,1)+(R2jn_ide-1)*2+(1:2)) = -Rot(Xk10e_ide(6,3));
                end

                % for R2jn_ide = 1:(size(R2ZknIde,1)/2)
                %     R2nRn_ide = blkdiag(R2nRn_ide, R2R);
                %     % Ideal EKF
                %     DeltaGV_ide(size(Xk10e_ide,1)+size(R1ZknIde,1)+(R2jn_ide-1)*2+(1:2),size(R1ZknIde,1)+(R2jn_ide-1)*2+(1:2)) = Rot(Xrk10T(6,1));
                % end
            end

            nRn_ide = blkdiag(R1nRn_ide,R2nRn_ide);

            % Ideal EKF
            Xfn_ide = [R1Xfn_ide;R2Xfn_ide];
            Xk10efi_ide = [Xk10e_ide;
                [ones(size(R1Xfn_ide,1),1);2*ones(size(R2Xfn_ide,1),1)],Xfn_ide];

            Pk10fi_ide = DeltaGX_ide*Pk10_ide*DeltaGX_ide'+DeltaGV_ide*nRn_ide*DeltaGV_ide';
            Pk10fi_ide(abs(Pk10fi_ide)<CovT) = 0;

        else
            % Ideal EKF
            Xk10efi_ide = Xk10e_ide;
            Pk10fi_ide = Pk10_ide;
        end



        %% Update using shared feature observations from R1 and R2
        %% R1
        % find the shared feature observations in R1 and R2 of Xk10e
        R1ZksIde_lv = ismember(R1Obs_k(:,1),Xk10e_ide(7:end,2));
        R1ZksIde_idx = find(R1ZksIde_lv);
        R1Zks1Ide = R1Obs_k(R1ZksIde_idx,:);

        % Ideal EKF
        % find the shared true feature R1XfksT in Xk10efi_ide
        % and re-order them to to make them consistent with R1ObsT_k
        R1XfksIde_lv = ismember(Xk10efi_ide(7:end,2),R1Zks1Ide(:,1));
        R1XfksIde_idx = find(R1XfksIde_lv)+6;
        R1Xfks_ide = Xk10efi_ide(R1XfksIde_idx,2:3); % For Observation function

        R1Zks2Ide = [];
        R1XfksT = [];
        if ~isempty(R1Zks1Ide)
            [~,R1ZkSIde_idx] = ismember(R1Xfks_ide(:,1),R1Zks1Ide(:,1));
            R1ZkSIde_idx(2:2:end,1)=R1ZkSIde_idx(2:2:end,1)+1;
            R1Zks2Ide = R1Zks1Ide(R1ZkSIde_idx,:);

            % find the shared true feature R1XfksT in feature truth XfTrueAll
            % and re-order them to to make them consistent with R1Xfks
            [~,R1XfksT_idx] = ismember(R1Xfks_ide(:,1),XfTrueAll(:,1));
            R1XfksT_idx(2:2:end,1)=R1XfksT_idx(2:2:end,1)+1;
            R1XfksT = XfTrueAll(R1XfksT_idx,:); % True feature position for Jacobian function
        end
        %%


        %% R2
        R2ZksIde_lv = ismember(R2Obs_k(:,1),Xk10e_ide(7:end,2));
        R2ZksIde_idx = find(R2ZksIde_lv);
        R2Zks1Ide = [R2Obs_k(R2ZksIde_idx,:);R2ZknsIde];

        % Ideal EKF
        % find the shared true feature R1XfksT in Xk10efi_ide
        % and re-order them to to make them consistent with R1ObsT_k
        R2XfksIde_lv = ismember(Xk10efi_ide(7:end,2),R2Zks1Ide(:,1));
        R2XfksIde_idx = find(R2XfksIde_lv)+6;
        R2Xfks_ide = Xk10efi_ide(R2XfksIde_idx,2:3);

        R2Zks2Ide = [];
        R2XfksT = [];
        if ~isempty(R2Zks1Ide)
            [~,R2ZkSIde_idx] = ismember(R2Xfks_ide(:,1),R2Zks1Ide(:,1));
            R2ZkSIde_idx(2:2:end,1)=R2ZkSIde_idx(2:2:end,1)+1;
            R2Zks2Ide = R2Zks1Ide(R2ZkSIde_idx,:);

            [~,R2XfksT_idx] = ismember(R2Xfks_ide(:,1),XfTrueAll(:,1));
            R2XfksT_idx(2:2:end,1)=R2XfksT_idx(2:2:end,1)+1;
            R2XfksT = XfTrueAll(R2XfksT_idx,:);
        end
        %%


        ZksIde = [ones(size(R1Zks2Ide,1),1),R1Zks2Ide;2*ones(size(R2Zks2Ide,1),1),R2Zks2Ide];

        %% Observation model
        % Ideal EKF
        Xk11e_ide = Xk10efi_ide;
        Pk11_ide = Pk10fi_ide;

        if ~isempty(ZksIde)
            % Ideal EKF
            HX10e_ide = sparse(size(ZksIde,1), 1);
            JHX10e_ide = sparse(size(ZksIde,1), size(Xk10efi_ide,1));

            Xrk10Tfi = Xrk10T; % for Ideal EKF only

            R1DV = [];
            R2DV = [];

            % R1
            if ~isempty(R1Zks2Ide)
                % Ideal EKF
                HX10e_ide(1:2:(size(R1Zks2Ide,1)-1),1) = cos(Xk10efi_ide(3,3))*(R1Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(1,3)) + ...
                    sin(Xk10efi_ide(3,3))*(R1Xfks_ide(2:2:end,2)-Xk10efi_ide(2,3));
                HX10e_ide(2:2:size(R1Zks2Ide,1),1) = -sin(Xk10efi_ide(3,3))*(R1Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(1,3)) + ...
                    cos(Xk10efi_ide(3,3))*(R1Xfks_ide(2:2:end,2)-Xk10efi_ide(2,3));

                JHX10e_ide(1:2:(size(R1Zks2Ide,1)-1),1:3) = [repmat([-cos(Xrk10Tfi(3,1)),-sin(Xrk10Tfi(3,1))],size(R1Zks2Ide,1)/2,1), ...
                    -sin(Xrk10Tfi(3,1))*(R1XfksT(1:2:(end-1),2)-Xrk10Tfi(1,1))+cos(Xrk10Tfi(3,1))*(R1XfksT(2:2:end,2)-Xrk10Tfi(2,1))];

                JHX10e_ide(2:2:size(R1Zks2Ide,1),1:3) = [repmat([sin(Xrk10Tfi(3,1)),-cos(Xrk10Tfi(3,1))],size(R1Zks2Ide,1)/2,1), ...
                    -cos(Xrk10Tfi(3,1))*(R1XfksT(1:2:(end-1),2)-Xrk10Tfi(1,1))-sin(Xrk10Tfi(3,1))*(R1XfksT(2:2:end,2)-Xrk10Tfi(2,1))];

                for R1kjIde = 1:size(R1Zks2Ide,1)/2
                    R1DV = blkdiag(R1DV,R1R);

                    % Ideal EKF
                    JHX10e_ide((R1kjIde-1)*2+(1:2), R1XfksIde_idx((R1kjIde-1)*2+(1:2),1)') = Rot(Xrk10Tfi(3,1))';
                end
            end

            % R2
            if ~isempty(R2Zks2Ide)
                % Ideal EKF
                HX10e_ide(size(R1Zks2Ide,1)+(1:2:(size(R2Zks2Ide,1)-1)),1) = cos(Xk10efi_ide(6,3))*(R2Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(4,3)) + ...
                    sin(Xk10efi_ide(6,3))*(R2Xfks_ide(2:2:end,2)-Xk10efi_ide(5,3));
                HX10e_ide(size(R1Zks2Ide,1)+(2:2:size(R2Zks2Ide,1)),1) = -sin(Xk10efi_ide(6,3))*(R2Xfks_ide(1:2:(end-1),2)-Xk10efi_ide(4,3)) + ...
                    cos(Xk10efi_ide(6,3))*(R2Xfks_ide(2:2:end,2)-Xk10efi_ide(5,3));

                JHX10e_ide(size(R1Zks2Ide,1)+(1:2:size(R2Zks2Ide,1)-1),4:6) = [repmat([-cos(Xrk10Tfi(6,1)),-sin(Xrk10Tfi(6,1))],size(R2Zks2Ide,1)/2,1), ...
                    -sin(Xrk10Tfi(6,1))*(R2XfksT(1:2:(end-1),2)-Xrk10Tfi(4,1))+cos(Xrk10Tfi(6,1))*(R2XfksT(2:2:end,2)-Xrk10Tfi(5,1))];

                JHX10e_ide(size(R1Zks2Ide,1)+(2:2:size(R2Zks2Ide,1)),4:6) = [repmat([sin(Xrk10Tfi(6,1)),-cos(Xrk10Tfi(6,1))],size(R2Zks2Ide,1)/2,1), ...
                    -cos(Xrk10Tfi(6,1))*(R2XfksT(1:2:(end-1),2)-Xrk10Tfi(4,1))-sin(Xrk10Tfi(6,1))*(R2XfksT(2:2:end,2)-Xrk10Tfi(5,1))];

                for R2kjIde = 1:size(R2Zks2Ide,1)/2
                    R2DV = blkdiag(R2DV,R2R);

                    % Ideal EKF
                    JHX10e_ide(size(R1Zks2Ide,1)+(R2kjIde-1)*2+(1:2), R2XfksIde_idx((R2kjIde-1)*2+(1:2),1)') = Rot(Xrk10Tfi(6,1))';
                end
            end

            DV = blkdiag(R1DV,R2DV);

            % Innovation Covariance S and Kalman Gain K
            % Ideal EKF
            Ssi = JHX10e_ide * Pk10fi_ide * JHX10e_ide' + DV;
            Ksi = Pk10fi_ide * JHX10e_ide' /Ssi;

            % Updating process using observation model
            % Ideal EKF
            Xk11e_ide(:,3) = Xk10efi_ide(:,3) + Ksi*(ZksIde(:,3)-HX10e_ide);
            Xk11e_ide([3,6],3) = wrap(Xk11e_ide([3,6],3));
            Pk11_ide = Pk10fi_ide - Ksi*Ssi*Ksi';
            Pk11_ide(abs(Pk11_ide)<CovT) = 0;
        end

        % Ideal EKF
        R1XpIdeFull = [R1XpIdeFull;Xk11e_ide(1:3,2:3)];
        R1PIdeFull = [R1PIdeFull;Pk11_ide(1:3,1:3)];

        R2XpIdeFull = [R2XpIdeFull;Xk11e_ide(4:6,2:3)];
        R2PIdeFull = [R2PIdeFull;Pk11_ide(4:6,4:6)];

        Xk00e_ide = Xk11e_ide;
        Pk00_ide = Pk11_ide;
    end

    % Ideal EKF
    R1XpIdeFullSet = [R1XpIdeFullSet,R1XpIdeFull(:,2)];
    R1PIdeFullSet = [R1PIdeFullSet,R1PIdeFull];

    R2XpIdeFullSet = [R2XpIdeFullSet,R2XpIdeFull(:,2)];
    R2PIdeFullSet = [R2PIdeFullSet,R2PIdeFull];

    XfIdeFullSet = [XfIdeFullSet,Xk11e_ide(7:end,3)];
    PfIdeFullSet = [PfIdeFullSet,Pk11_ide(7:end,7:end)];
end

feaNum = size(Xk11e_ide(7:end,1),1)/2;

% Ideal EKF
R1XpIdeFullSet = [R1XpIdeFull(:,1),R1XpIdeFullSet];
R2XpIdeFullSet = [R2XpIdeFull(:,1),R2XpIdeFullSet];
XfIdeFullSet = [Xk11e_ide(7:end,2),XfIdeFullSet];

% if ec == 4 && TrajP == 1
%     figure((ec-1)*6+8)
%     hold on
%     R1PosiVPP = plot(R1XpIdeFullSet(1:3:(end-2),2),R1XpIdeFullSet(2:3:(end-1),2),'bo','DisplayName','R1 Position');
%     R2PosiVPP = plot(R2XpIdeFullSet(1:3:(end-2),2),R2XpIdeFullSet(2:3:(end-1),2),'ro','DisplayName','R2 Position');
%     FeaPosiVPP = plot(XfIdeFullSet(1:2:(end-1),2),XfIdeFullSet(2:2:end,2),'g^','DisplayName','Feature Position');
%     legend([R1PosiVPP,R2PosiVPP,FeaPosiVPP])
%     title('VictoriaPark Trajectory for ideal EKF')
%     hold off
% end

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

for pn = 0:poseNum

    if pn ~= 0
        R1XpTrue = [R1XrTrue(pn*2+(1:2),:);R1XphiT(pn+1,:)];

        DeltaR1XpIdeFullSet((end+1):(end+3),:) = [R1XpTrue(:,1),R1XpIdeFullSet((pn-1)*3+(1:3),2:end)-R1XpTrue(:,2)];

        % wrap the delta angle
        DeltaR1XpIdeFullSet(end,2:end) = wrap(DeltaR1XpIdeFullSet(end,2:end));

        DeltaR1XrIdeFullSet((end+1):(end+2),:) = DeltaR1XpIdeFullSet((end-2):(end-1),:);

        DeltaR1XphiIdeFullSet(end+1,:) = DeltaR1XpIdeFullSet(end,:);
    end

    R2XpTrue = [R2XrTrue(pn*2+(1:2),:);R2XphiT(pn+1,:)];

    DeltaR2XpIdeFullSet((end+1):(end+3),:) = [R2XpTrue(:,1),R2XpIdeFullSet(pn*3+(1:3),2:end)-R2XpTrue(:,2)];

    DeltaR2XpIdeFullSet(end,2:end) = wrap(DeltaR2XpIdeFullSet(end,2:end));

    DeltaR2XrIdeFullSet((end+1):(end+2),:) = DeltaR2XpIdeFullSet((end-2):(end-1),:);

    DeltaR2XphiIdeFullSet(end+1,:) = DeltaR2XpIdeFullSet(end,:);
end

%% re-order the true features XfTrueAll's IDs to make it consistent with FFullSet for every step
[~,XfTrueIde_idx] = ismember(XfIdeFullSet(:,1),XfTrueAll(:,1));
XfTrueIde_idx(2:2:end,1) = XfTrueIde_idx(2:2:end,1)+1;
XfTrueIde = XfTrueAll(XfTrueIde_idx,:);
% Ideal EKF
DeltaXfIdeFullSet = [XfIdeFullSet(:,1),XfIdeFullSet(:,2:end)-XfTrueIde(:,2)];

%% save the Monte Carlo Experiments result
if ec == 1
    if env == 1
        save('MTE_results_IdeEKF_20fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    else
        save('MTE_results_IdeEKF_20fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    end
elseif ec == 2
    if env == 1
        save('MTE_results_IdeEKF_60fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    else
        save('MTE_results_IdeEKF_60fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    end
elseif ec == 3
    if env == 1
        save('MTE_results_IdeEKF_100fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    else
        save('MTE_results_IdeEKF_100fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
    end
    % elseif ec == 4
    %     save('VicP_results_IdeEKF.mat','poseNum','feaNum', ...
    %         'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
    %         'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
    %         'DeltaXfIdeFullSet','PfIdeFullSet')
end

disp('Ide EKF Complete!')