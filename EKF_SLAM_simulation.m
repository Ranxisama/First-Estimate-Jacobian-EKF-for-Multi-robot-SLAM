clc
close all

load('Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
load('Measurements.mat','R1Xp0','R1Odo','R1Obs','R2Xp0','R2Odo','R2Obs')

Config;

Xf11eP = [];
eeXf11ePHandles = [];

for k = 0:size(R1Odo,1)/3

    % k

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

        XsGni = Xs;
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
        % R1Zkn: new feature observation of 1st robot
        R1Zkn = R1Obs_k(R1Zkn_idx,:);

        R1Xfkn = R1Zkn;
        R1Xfkn(1:2:(end-1),2) = X0(1,3) + cos(X0(3,3))*(R1Zkn(1:2:(end-1),2)) - sin(X0(3,3))*(R1Zkn(2:2:end,2));
        R1Xfkn(2:2:end,2) = X0(2,3) + sin(X0(3,3))*(R1Zkn(1:2:(end-1),2)) + cos(X0(3,3))*(R1Zkn(2:2:end,2));

        % find the new observed feature IDs in 2nd robot
        % R2Zkn_lv: logical vector of new feature observation of
        % 2nd robot at step k
        R2Zkn_lv = ~ismember(R2Obs_k(:,1),X0(7:end,2));
        % R2Zkn_idx: index of new feature observation of
        % 2nd robot in R2Obs_k
        R2Zkn_idx = find(R2Zkn_lv);
        % R2Zkn: new feature observation of 2nd robot
        R2Zkn = R2Obs_k(R2Zkn_idx,:);

        % Zkns = intersect(R1Zkn(:,1),R2Zkn(:,1));

        R2Xfkn = R2Zkn;
        R2Xfkn(1:2:(end-1),2) = X0(4,3) + cos(X0(6,3))*(R2Zkn(1:2:(end-1),2)) - sin(X0(6,3))*(R2Zkn(2:2:end,2));
        R2Xfkn(2:2:end,2) = X0(5,3) + sin(X0(6,3))*(R2Zkn(1:2:(end-1),2)) + cos(X0(6,3))*(R2Zkn(2:2:end,2));

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
        JFXk(size(X0,1)+(1:2:(size(R1Xfkn,1)-1)),1:3) = [repmat([1,0],size(R1Xfkn,1)/2,1), -sin(X0(3,3))*R1Zkn(1:2:(end-1),2) - cos(X0(3,3))*R1Zkn(2:2:end,2)];
        JFXk(size(X0,1)+(2:2:size(R1Xfkn,1)),1:3) = [repmat([0,1],size(R1Xfkn,1)/2,1), cos(X0(3,3))*R1Zkn(1:2:(end-1),2) - sin(X0(3,3))*R1Zkn(2:2:end,2)];

        JFXk(size(X0,1)+size(R1Xfkn,1)+(1:2:(size(R2Xfkn,1)-1)),4:6) = [repmat([1,0],size(R2Xfkn,1)/2,1), -sin(X0(6,3))*R2Zkn(1:2:(end-1),2) - cos(X0(6,3))*R2Zkn(2:2:end,2)];
        JFXk(size(X0,1)+size(R1Xfkn,1)+(2:2:size(R2Xfkn,1)),4:6) = [repmat([0,1],size(R2Xfkn,1)/2,1), cos(X0(6,3))*R2Zkn(1:2:(end-1),2) - sin(X0(6,3))*R2Zkn(2:2:end,2)];


        R1nRn = [];
        R2nRn = [];
        JFWk = sparse(size(Xk00e,1),size(R1Zkn,1)+size(R2Zkn,1));
        for R1jn = 1:(size(R1Zkn,1)/2)
            R1nRn = blkdiag(R1nRn, R1R);
            JFWk(size(X0,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = rotationMatrix(X0(3,3));
        end
        for R2jn = 1:(size(R2Zkn,1)/2)
            R2nRn = blkdiag(R2nRn, R2R);
            JFWk(size(X0,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(X0(6,3));
        end

        nRn = blkdiag(R1nRn,R2nRn);
        Pk00 = JFXk*P0*JFXk'+JFWk*nRn*JFWk';

        Pk00(abs(Pk00)<CovT) = 0;

        R1XpFull = Xk00e(1:3,2:3); % save all robot postures of R1
        R2XpFull = Xk00e(4:6,2:3); % save all robot postures of R2

        R1ee = errorEllipse(Pk00(1:2,1:2), Xk00e(1:2,3), CI);
        R2ee = errorEllipse(Pk00(4:5,4:5), Xk00e(4:5,3), CI);

        R1eeFull = R1ee;
        R2eeFull = R2ee;

        continue
    end

    %% Prediction using the motion model
    R1Odo_k = R1Odo(R1Odo(:,2)==k,3);
    R2Odo_k = R2Odo(R2Odo(:,2)==k,3);

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

    DeltaFX = sparse(size(Xk10e,1),size(Xk10e,1));
    DeltaFX(1:6,1:6) = blkdiag([1,0,-sin(Xk00e(3,3))*R1Odo_k(1,1) - cos(Xk00e(3,3))*R1Odo_k(2,1); ...
        0,1,cos(Xk00e(3,3))*R1Odo_k(1,1) - sin(Xk00e(3,3))*R1Odo_k(2,1); ...
        0,0,1], ...
        [1,0,-sin(Xk00e(6,3))*R2Odo_k(1,1) - cos(Xk00e(6,3))*R2Odo_k(2,1); ...
        0,1,cos(Xk00e(6,3))*R2Odo_k(1,1) - sin(Xk00e(6,3))*R2Odo_k(2,1); ...
        0,0,1]);
    DeltaFX(7:end,7:end) = eye(size(DeltaFX(7:end,7:end)));

    DeltaFW = sparse(size(Xk10e,1),6);
    DeltaFW(1:6,1:6) = blkdiag([cos(Xk00e(3,3)),-sin(Xk00e(3,3)),0; ...
        sin(Xk00e(3,3)),cos(Xk00e(3,3)),0; ...
        0,0,1], ...
        [cos(Xk00e(6,3)),-sin(Xk00e(6,3)),0; ...
        sin(Xk00e(6,3)),cos(Xk00e(6,3)),0; ...
        0,0,1]);

    DWk = blkdiag(R1Q,R2Q);
    Pk10 = DeltaFX * Pk00 * DeltaFX' + DeltaFW * DWk * DeltaFW';

    % Set the elements that are less than CovT to zero. This can be useful for dealing with numerical errors or avoiding unnecessary imaginary parts in calculations.
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

    % % R1和R2都看到同一个新feature怎么办
    Zkns = intersect(R1Zkn(:,1),R2Zkn(:,1));
    % 用R1的来initialization,R2的从R2Zkn中去掉，后面用来做update
    if ~isempty(Zkns)
        for ZknsNum = 1:size(Zkns,1)
            R2Zkn(R2Zkn(:,1)==Zkns(ZknsNum,1),:) = [];
        end
    end

    Zkn = [R1Zkn;R2Zkn];

    Xk10efi = Xk10e;
    Pk10fi = Pk10;

    if ~isempty(Zkn)
        R1Xfn = R1Zkn;
        R2Xfn = R2Zkn;

        DeltaGX = sparse(size(Xk10efi,1),size(Xk10e,1));
        DeltaGX(1:size(Xk10e,1),1:size(Xk10e,1)) = eye(size(Xk10e,1));

        R1nRn = [];
        R2nRn = [];
        DeltaGV = sparse(size(Xk10efi,1),size(Zkn,1));

        if ~isempty(R1Xfn)
            R1Xfn(1:2:(end-1),2) = Xk10e(1,3) + cos(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) - sin(Xk10e(3,3))*R1Zkn(2:2:end,2);
            R1Xfn(2:2:end,2) = Xk10e(2,3) + sin(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) + cos(Xk10e(3,3))*R1Zkn(2:2:end,2);

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
            R2Xfn(1:2:(end-1),2) = Xk10e(4,3) + cos(Xk10e(6,3))*R2Zkn(1:2:end,2) - sin(Xk10e(6,3))*R2Zkn(2:2:end,2);
            R2Xfn(2:2:end,2) = Xk10e(5,3) + sin(Xk10e(6,3))*R2Zkn(1:2:end,2) + cos(Xk10e(6,3))*R2Zkn(2:2:end,2);

            DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(1:2:(size(R2Xfn,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn,1)/2,1), ...
                -sin(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-cos(Xk10e(6,3))*R2Zkn(2:2:end,2)];
            DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(2:2:size(R2Xfn,1)),4:6) = [repmat([0, 1],size(R2Xfn,1)/2,1), ...
                cos(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-sin(Xk10e(6,3))*R2Zkn(2:2:end,2)];

            for R2jn = 1:(size(R2Zkn,1)/2)
                R2nRn = blkdiag(R2nRn, R2R);
                DeltaGV(size(Xk10e,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(Xk10e(6,3));
            end
        end

        Xfn = [R1Xfn;R2Xfn];
        Xk10efi = [Xk10e;
            [ones(size(R1Xfn,1),1);2*ones(size(R2Xfn,1),1)],Xfn];

        nRn = blkdiag(R1nRn,R2nRn);
        Pk10fi = DeltaGX*Pk10*DeltaGX'+DeltaGV*nRn*DeltaGV';

        Pk10fi(abs(Pk10fi)<CovT) = 0;

    end



    %% Update using shared feature observations from R1 and R2
    % find the shared feature observations in R1 and R2 of Xk10e
    R1Zks_lv = ismember(R1Obs_k(:,1),Xk10efi(7:end,2));
    R1Zks_idx = find(R1Zks_lv);
    R1Zks1 = R1Obs_k(R1Zks_idx,:);

    % find the shared features in Xk10efi
    R1Xfks_lv = ismember(Xk10efi(7:end,2),R1Obs_k(:,1));
    R1Xfks_idx = find(R1Xfks_lv)+6;
    % R1Xfks_idx: index of R1's shared features with Xk10efi at step k
    R1Xfks = Xk10efi(R1Xfks_idx,2:3);

    % re-order the shared features R1Zks to make it consistent with R1Xfks
    [~,R1ZkS_idx] = ismember(R1Xfks(:,1),R1Zks1(:,1));
    R1ZkS_idx(2:2:end,1)=R1ZkS_idx(2:2:end,1)+1;
    % R1ZkS_idx: index of R1's shared features with R1Zks at step k
    R1Zks2 = R1Zks1(R1ZkS_idx,:);



    R2Zks_lv = ismember(R2Obs_k(:,1),Xk10efi(7:end,2));
    R2Zks_idx = find(R2Zks_lv);
    R2Zks1 = R2Obs_k(R2Zks_idx,:);

    R2Xfks_lv = ismember(Xk10efi(7:end,2),R2Obs_k(:,1));
    R2Xfks_idx = find(R2Xfks_lv)+6;
    R2Xfks = Xk10efi(R2Xfks_idx,2:3);

    [~,R2ZkS_idx] = ismember(R2Xfks(:,1),R2Zks1(:,1));
    R2ZkS_idx(2:2:end,1)=R2ZkS_idx(2:2:end,1)+1;
    % R1ZkS_idx: index of R1's shared features with R1Zks at step k
    R2Zks2 = R2Zks1(R2ZkS_idx,:);

    Zks = [ones(size(R1Zks2,1),1),R1Zks2;2*ones(size(R2Zks2,1),1),R2Zks2];
    % Xfks_idx = [R1Xfks_idx;R2Xfks_idx];

    Xk11e = Xk10efi;
    Pk11 = Pk10fi;
    if ~isempty(Zks)
        HX10e = sparse(size(Zks,1), 1);
        JHX10e = sparse(size(Zks,1), size(Xk10efi,1));

        R1DV = [];
        R2DV = [];
        if ~isempty(R1Zks2)
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
                JHX10e((R1kj-1)*2+(1:2), R1Xfks_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(3,3))';
            end
        end

        if ~isempty(R2Zks2)
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
                JHX10e(size(R1Zks2,1)+(R2kj-1)*2+(1:2), R2Xfks_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xk10efi(6,3))';
            end
        end

        % JHX10_e = full(JHX10e)
        DV = blkdiag(R1DV,R2DV);

        % Innovation Covariance S and Kalman Gain K
        Ss = JHX10e * Pk10fi * JHX10e' + DV;
        Ks = Pk10fi * JHX10e' /Ss;

        % Updating process using observation model
        % Xk11e(:,3) = Xk10efi(:,3) + Ks*(Zks(:,3)-HX10e);
        Xk11e(:,3) = Xk10efi(:,3) + Ks*(Zks(:,3)-HX10e);
        Xk11e([3,6],3) = wrap(Xk11e([3,6],3));

        Pk11 = Pk10fi - Ks*Ss*Ks';
    end

    R1XpFull = [R1XpFull;Xk11e(1:3,2:3)];
    R2XpFull = [R2XpFull;Xk11e(4:6,2:3)];

    R1ee = errorEllipse(Pk11(1:2,1:2), Xk11e(1:2,3), CI);
    R2ee = errorEllipse(Pk11(4:5,4:5), Xk11e(4:5,3), CI);

    R1eeFull = [R1eeFull,R1ee];
    R2eeFull = [R2eeFull,R2ee];

    Xk00e = Xk11e;
    Pk00 = Pk11;

end

figure(1)

hold on

R1XrTrueP = plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'-bo','MarkerSize',MS);
R1XrTrueP(1).DisplayName = 'R1 true pose';
R2XrTrueP = plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'-co','MarkerSize',MS);
R2XrTrueP(1).DisplayName = 'R2 true pose';
XfTrueAllP = plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'*','Color',darkGreen,'MarkerSize',MS);
XfTrueAllP(1).DisplayName = 'True features';

R1XreP = plot(R1XpFull(1:3:(end-2),2),R1XpFull(2:3:(end-1),2),'--ro','MarkerSize',MS);
R1XreP.DisplayName = 'R1 estimated pose';
R2XreP = plot(R2XpFull(1:3:(end-2),2),R2XpFull(2:3:(end-1),2),'--mo','MarkerSize',MS);
R2XreP(1).DisplayName = 'R2 estimated pose';
XfeP = plot(Xk11e(7:2:(end-1),3),Xk11e(8:2:end,3),'*','Color',brightGreen,'MarkerSize',MS);
XfeP(1).DisplayName = 'Estimated features';

if poseCovCheck == 1
    R1eeP = plot(R1eeFull(:,1:2:(end-1)),R1eeFull(:,2:2:end),'r');
    R2eeP = plot(R2eeFull(:,1:2:(end-1)),R2eeFull(:,2:2:end),'m');
    R1eeP(1).DisplayName = 'R1 estimated pose Cov';
    R2eeP(1).DisplayName = 'R1 estimated pose Cov';
end


if feaCovCheck == 1
    Fee = [];
    for feanum = 1:((size(Xk11e,1)-6)/2)
        Fee = [Fee,errorEllipse(Pk11(6+(feanum-1)*2+(1:2),6+(feanum-1)*2+(1:2)), Xk11e(6+(feanum-1)*2+(1:2),3), CI)];
    end
    FeeP = plot(Fee(:,1:2:(end-1)),Fee(:,2:2:end),'Color',brightGreen);
    FeeP(1).DisplayName = 'Estimated features Cov';
end

if feaCovCheck == 1
    legend([R1XrTrueP(1),R2XrTrueP(1),XfTrueAllP(1),R1XreP(1),R2XreP(1),XfeP(1),R1eeP(1),R2eeP(1),FeeP(1)])
else
    if poseCovCheck == 1
        legend([R1XrTrueP(1),R2XrTrueP(1),XfTrueAllP(1),R1XreP(1),R2XreP(1),XfeP(1),R1eeP(1),R2eeP(1)])
    else
        legend([R1XrTrueP(1),R2XrTrueP(1),XfTrueAllP(1),R1XreP(1),R2XreP(1),XfeP(1)])
    end

end

axis([fea_xlb, fea_xub,fea_ylb, fea_yub])
xlabel('x')
ylabel('y')
title(sprintf('Truth vs multi-robot EKF of %d steps and %d features',Xk11e(1,2),(size(Xk11e,1)-6)/2))

hold off