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
        % R1Xfk = R1Obs_k;
        % R1Xfk(1:2:(end-1),2) = R1Xp0(1,1) + cos(R1Xp0(3,1))*(R1Obs_k(1:2:(end-1),2)) - sin(R1Xp0(3,1))*(R1Obs_k(2:2:end,2));
        % R1Xfk(2:2:end,2) = R1Xp0(2,1) + sin(R1Xp0(3,1))*(R1Obs_k(1:2:(end-1),2)) + cos(R1Xp0(3,1))*(R1Obs_k(2:2:end,2));
        % 
        % % XfTrueAll
        % 
        % R1deltaFX0 = sparse(3+size(R1Xfk,1),3+size(R1Xfk,1));
        % 
        % R1deltaFX0(1:3,1:3) = eye(3);
        % 
        % R1deltaFX0(4:2:(end-1),1:3) = [repmat([1,0],size(R1Xfk,1)/2,1), ...
        %     -sin(R1Xp0(3))*R1Obs_k(1:2:(end-1),2) - cos(R1Xp0(3))*R1Obs_k(2:2:end,2)];
        % 
        % R1deltaFX0(5:2:end,1:3) = [repmat([0,1],size(R1Xfk,1)/2,1), ...
        %     cos(R1Xp0(3))*R1Obs_k(1:2:(end-1),2) - sin(R1Xp0(3))*R1Obs_k(2:2:end,2)];
        % 
        % % covariance matrix of observed features at step 0
        % R1Rn = [];
        % for R1j = 1:size(R1Xfk)/2
        %     R1Rn = blkdiag(R1Rn, R1R);
        %     R1deltaFX0(3+(R1j-1)*2+(1:2),3+(R1j-1)*2+(1:2)) = [cos(R1Xp0(3,1)), -sin(R1Xp0(3,1));
        %         sin(R1Xp0(3,1)), cos(R1Xp0(3,1))];
        % end
        % 
        % R1Pk0 = blkdiag(R1O,R1Rn);
        % 
        % R1Xk00e = [ones(3,1),zeros(3,1),R1Xp0;
        %     2*ones(size(R1Xfk,1),1),R1Xfk];
        % R1Pk00 = R1deltaFX0 * R1Pk0 * R1deltaFX0';

       
        
        % find the shared observed feature IDs in 2nd robot
        % R2Zks_lv: logical vector of shared feature observation of
        % 2nd robot at step k
        R1Zks_lv = ismember(R1Obs_k(:,1), R2Obs_k(:,1));
        % R2Zks_idx: index of shared feature observation of
        % 2nd robot in R2Obs_k
        R1Zks_idx = find(R1Zks_lv);
        % R2Zks: shared feature observation of 2nd robot
        R1Zks = R1Obs_k(R1Zks_idx,:);
        R1Xfks = R1Zks;
        R1Xfks(1:2:(end-1),2) = R1Xp0(1,1) + cos(R1Xp0(3,1))*(R1Zks(1:2:(end-1),2)) - sin(R1Xp0(3,1))*(R1Zks(2:2:end,2));
        R1Xfks(2:2:end,2) = R1Xp0(2,1) + sin(R1Xp0(3,1))*(R1Zks(1:2:(end-1),2)) + cos(R1Xp0(3,1))*(R1Zks(2:2:end,2));

        % find the shared observed feature IDs in 2nd robot
        % R2Zks_lv: logical vector of shared feature observation of
        % 2nd robot at step k
        R2Zks_lv = ismember(R2Obs_k(:,1), R1Obs_k(:,1));
        % R2Zks_idx: index of shared feature observation of
        % 2nd robot in R2Obs_k
        R2Zks_idx = find(R2Zks_lv);
        % R2Zks: shared feature observation of 2nd robot
        R2Zks = R2Obs_k(R2Zks_idx,:);

        

        %% use Gauss-Newton iteration to optimize the shared state at step 0
        % use R2Xp0 as the initial value of R2's pose in the GN iteration
        Xs = [ones(3,1),zeros(3,1),R1Xp0;
            ones(3,1),zeros(3,1),R2Xp0;
            2*ones(size(R1Xfks,1),1),R1Xfks];
        
        R1sRn = [];
        R2sRn = [];
        for kj = 1:(size(R1Xfks,1)/2)
            R1sRn = blkdiag(R1sRn,R1R);
            R2sRn = blkdiag(R2sRn,R2R);
        end
        Ps = blkdiag(R1O,R2O,R1sRn,R2sRn);

        Zks = [Xs(1:6,3);R1Zks(:,2);R2Zks(:,2)];
        
        XsGni = Xs;
        [XsGni(:,3),PsGni] = GNI(Xs(:,3),Ps,Zks,CC);
      
        
        
        %% estimate new observed feature's state at step 0 using the observation model
        % find the new observed feature IDs in 1st robot
        % R1Zkn_lv: logical vector of new feature observation of
        % 2nd robot at step k
        R1Zkn_lv = ~R1Zks_lv;
        % R1Zkn_idx: index of new feature observation of
        % 1st robot in R1Obs_k
        R1Zkn_idx = find(R1Zkn_lv);
        % R1Zkn: new feature observation of 1st robot
        R1Zkn = R1Obs_k(R1Zkn_idx,:);

        R1Xfkn = R1Zkn;
        R1Xfkn(1:2:(end-1),2) = XsGni(1,3) + cos(XsGni(3,3))*(R1Zkn(1:2:(end-1),2)) - sin(XsGni(3,3))*(R1Zkn(2:2:end,2));
        R1Xfkn(2:2:end,2) = XsGni(2,3) + sin(XsGni(3,3))*(R1Zkn(1:2:(end-1),2)) + cos(XsGni(3,3))*(R1Zkn(2:2:end,2));

        % find the new observed feature IDs in 2nd robot
        % R2Zkn_lv: logical vector of new feature observation of
        % 2nd robot at step k
        R2Zkn_lv = ~R2Zks_lv;
        % R2Zkn_idx: index of new feature observation of
        % 2nd robot in R2Obs_k
        R2Zkn_idx = find(R2Zkn_lv);
        % R2Zkn: new feature observation of 2nd robot
        R2Zkn = R2Obs_k(R2Zkn_idx,:);

        % Zkns = intersect(R1Zkn(:,1),R2Zkn(:,1));
        
        R2Xfkn = R2Zkn;
        R2Xfkn(1:2:(end-1),2) = XsGni(4,3) + cos(XsGni(6,3))*(R2Zkn(1:2:(end-1),2)) - sin(XsGni(6,3))*(R2Zkn(2:2:end,2));
        R2Xfkn(2:2:end,2) = XsGni(5,3) + sin(XsGni(6,3))*(R2Zkn(1:2:(end-1),2)) + cos(XsGni(6,3))*(R2Zkn(2:2:end,2));

        %% Xk00e: state estimate
        % first column: 1 -> robot posture; 2 -> feature position
        % second column: posture id or position id
        % third column: data
        Xk00e = [XsGni;
            2*ones(size(R1Xfkn,1),1),R1Xfkn;
            2*ones(size(R2Xfkn,1),1),R2Xfkn];

        %% Jacobian of Xk

        JFXk = sparse(size(Xk00e,1),size(XsGni,1));
        JFXk(1:size(XsGni,1),1:size(XsGni,1)) = eye(size(XsGni,1));
        JFXk(size(XsGni,1)+(1:2:(size(R1Xfkn,1)-1)),1:3) = [repmat([1,0],size(R1Xfkn,1)/2,1), -sin(XsGni(3,3))*R1Zkn(1:2:(end-1),2) - cos(XsGni(3,3))*R1Zkn(2:2:end,2)];
        JFXk(size(XsGni,1)+(2:2:size(R1Xfkn,1)),1:3) = [repmat([0,1],size(R1Xfkn,1)/2,1), cos(XsGni(3,3))*R1Zkn(1:2:(end-1),2) - sin(XsGni(3,3))*R1Zkn(2:2:end,2)];

        JFXk(size(XsGni,1)+size(R1Xfkn,1)+(1:2:(size(R2Xfkn,1)-1)),4:6) = [repmat([1,0],size(R2Xfkn,1)/2,1), -sin(XsGni(6,3))*R2Zkn(1:2:(end-1),2) - cos(XsGni(6,3))*R2Zkn(2:2:end,2)];
        JFXk(size(XsGni,1)+size(R1Xfkn,1)+(2:2:size(R2Xfkn,1)),4:6) = [repmat([0,1],size(R2Xfkn,1)/2,1), cos(XsGni(6,3))*R2Zkn(1:2:(end-1),2) - sin(XsGni(6,3))*R2Zkn(2:2:end,2)];
        
        
        R1nRn = [];
        R2nRn = [];
        JFWk = sparse(size(Xk00e,1),size(R1Zkn,1)+size(R2Zkn,1));
        for R1jn = 1:(size(R1Zkn,1)/2)
            R1nRn = blkdiag(R1nRn, R1R);
            JFWk(size(XsGni,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = rotationMatrix(XsGni(3,3));
        end
        for R2jn = 1:(size(R2Zkn,1)/2)
            R2nRn = blkdiag(R2nRn, R2R);
            JFWk(size(XsGni,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(XsGni(6,3));
        end
        
        nRn = blkdiag(R1nRn,R2nRn);
        Pk00 = JFXk*PsGni*JFXk'+JFWk*nRn*JFWk';

        
        
        XrR1_full = Xk00e(1:3,2:3); % save all robot postures of R1
        XrR2_full = Xk00e(4:6,2:3); % save all robot postures of R2
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
    
    %% Feature initialization using new feature observations from R1 and R2
    % find the feature observations in R1 from new features of Xk10e
    R1Zkn_lv = ~ismember(R1Obs_k(:,1),Xk10e(7:end,2));
    R1Zkn_idx = find(R1Zkn_lv);
    R1Zkn = R1Obs_k(R1Zkn_idx,:);

    % find the feature observations in R2 from new features of Xk10e
    R2Zkn_lv = ~ismember(R2Obs_k(:,1),Xk10e(7:end,2));
    R2Zkn_idx = find(R2Zkn_lv);
    R2Zkn = R2Obs_k(R2Zkn_idx,:);

    %% R1和R2都看到同一个新feature怎么办~~~~~~~~~~~~~~~~~~~~~~~~~~
    Zkns = intersect(R1Zkn(:,1),R2Zkn(:,1));
    % 直接从R2Zkn中去掉
    if ~isempty(Zkns)
        for ZknsNum = 1:size(Zkns,1)
            R2Zkn(R2Zkn(:,1)==Zkns(ZknsNum,1),:) = [];
        end
    end

    if ~isempty(R1Zkn) || ~isempty(R2Zkn)
        R1Xfn = R1Zkn;
        R1Xfn(1:2:(end-1),2) = Xk10e(1,3) + cos(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) - sin(Xk10e(3,3))*R1Zkn(2:2:end,2);
        R1Xfn(2:2:end,2) = Xk10e(2,3) + sin(Xk10e(3,3))*R1Zkn(1:2:(end-1),2) + cos(Xk10e(3,3))*R1Zkn(2:2:end,2);

        R2Xfn = R2Zkn;
        R2Xfn(1:2:end,2) = Xk10e(1,3) + cos(Xk10e(3,3))*R2Zkn(1:2:end,2) - sin(Xk10e(3,3))*R2Zkn(2:2:end,2);
        R2Xfn(2:2:end,2) = Xk10e(2,3) + sin(Xk10e(3,3))*R2Zkn(1:2:end,2) + cos(Xk10e(3,3))*R2Zkn(2:2:end,2);

        Xk10eS = [Xk10e;2*ones(size(R1Xfn,1)+size(R2Xfn,1),1),[R1Xfn;R2Xfn]];

        DeltaGX = sparse(size(Xk10eS,1),size(Xk10e,1));

        DeltaGX(1:size(Xk10e,1),1:size(Xk10e,1)) = eye(size(Xk10e,1));

        DeltaGX(size(Xk10e,1)+(1:2:(size(R1Xfn,1)-1)),1:3) = [repmat([1, 0],size(R1Xfn,1)/2,1), ...
            -sin(Xk10e(3,3))*R1Zkn(1:2:(end-1),2)-cos(Xk10e(3,3))*R1Zkn(2:2:end,2)];
        DeltaGX(size(Xk10e,1)+(2:2:size(R1Xfn,1)),1:3) = [repmat([0, 1],size(R1Xfn,1)/2,1), ...
            cos(Xk10e(3,3))*R1Zkn(1:2:(end-1),2)-sin(Xk10e(3,3))*R1Zkn(2:2:end,2)];

        DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(1:2:(size(R2Xfn,1)-1)),4:6) = [repmat([1, 0],size(R2Xfn,1)/2,1), ...
            -sin(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-cos(Xk10e(6,3))*R2Zkn(2:2:end,2)];
        DeltaGX(size(Xk10e,1)+size(R1Xfn,1)+(2:2:size(R2Xfn,1)),4:6) = [repmat([0, 1],size(R2Xfn,1)/2,1), ...
            cos(Xk10e(6,3))*R2Zkn(1:2:(end-1),2)-sin(Xk10e(6,3))*R2Zkn(2:2:end,2)];
        
        R1nRn = [];
        R2nRn = [];
        DeltaGV = sparse(size(Xk10eS,1),size(R1Xfn,1)+size(R2Zkn,1));
       for R1jn = 1:(size(R1Zkn,1)/2)
            R1nRn = blkdiag(R1nRn, R1R);
            DeltaGV(size(XsGni,1)+(R1jn-1)*2+(1:2),(R1jn-1)*2+(1:2)) = rotationMatrix(XsGni(3,3));
        end
        for R2jn = 1:(size(R2Zkn,1)/2)
            R2nRn = blkdiag(R2nRn, R2R);
            DeltaGV(size(XsGni,1)+size(R1Zkn,1)+(R2jn-1)*2+(1:2),size(R1Zkn,1)+(R2jn-1)*2+(1:2)) = rotationMatrix(XsGni(6,3));
        end
        
        nRn = blkdiag(R1nRn,R2nRn);
        Pk10S = DeltaGX*Pk10*DeltaGX'+DeltaGV*nRn*DeltaGV';
        
    else
        Xk10eS = Xk10e;
        Pk10S = Pk10;
    end

    %% Update using shared feature observations from R1 and R2
    % find the feature observations in R1 from shared features of Xk10e
    R1Zks_lv = ~R1Zkn_lv;
    R1Zks_idx = find(R1Zks_lv);
    R1Zks = R1Obs_k(R1Zks_idx,:);

    R1Xfks_lv = ismember(Xk10e(7:end,2),R1Obs_k(:,1));
    R1Xfks_idx = find(R1Xfks_lv)+6;
    R1Xfks = Xk10e(R1Xfks_idx,2:3);

    % find the feature observations in R2 from shared features of Xk10e
    R2Zks_lv = ~R2Zkn_lv;
    R2Zks_idx = find(R2Zks_lv);
    R2Zks = R2Obs_k(R2Zks_idx,:);

    R2Xfks_lv = ismember(Xk10e(7:end,2),R2Obs_k(:,1));
    R2Xfks_idx = find(R2Xfks_lv)+6;
    R2Xfks = Xk10e(R2Xfks_idx,2:3);
    
    Zks = [R1Zks;R2Zks];

    if ~isempty(R1Zks) || ~isempty(R2Zks)
    HX10e = sparse(size(R1Zks,1)+size(R2Zks,1), 1);

    HX10e(1:2:(size(R1Zks,1)-1),1) = cos(Xk10eS(3,3))*(R1Zks(1:2:(end-1),2)-Xk10eS(1,3)) + ...
        sin(Xk10eS(3,3))*(R1Zks(2:2:end,2)-Xk10eS(2,3));
    HX10e(2:2:size(R1Zks,1),1) = -sin(Xk10eS(3,3))*(R1Zks(1:2:(end-1),2)-Xk10eS(1,3)) + ...
        cos(Xk10eS(3,3))*(R1Zks(2:2:end,2)-Xk10eS(2,3));
    
    HX10e(size(R1Zks,1)+(1:2:(size(R2Zks,1)-1)),1) = cos(Xk10eS(6,3))*(R2Zks(1:2:(end-1),2)-Xk10eS(4,3))+sin(Xk10eS(6,3))*(R2Zks(2:2:end,2)-Xk10eS(5,3));
    HX10e(size(R1Zks,1)+(2:2:size(R2Zks,1)),1) = -sin(Xk10eS(6,3))*(R2Zks(1:2:(end-1),2)-Xk10eS(4,3))+cos(Xk10eS(6,3))*(R2Zks(2:2:end,2)-Xk10eS(5,3));



    JHX10e = sparse(size(R1Zks,1)+size(R2Zks,1), size(Xk10eS,1));

    JHX10e(1:2:(size(R1Zks,1)-1),1:3) = [repmat([-cos(Xk10eS(3,3)),-sin(Xk10eS(3,3))],size(R1Zks,1)/2,1), ...
        -sin(Xk10eS(3,3))*(R1Zks(1:2:(end-1),2)-Xk10eS(1,3))+cos(Xk10eS(3,3))*(R1Zks(2:2:end,2)-Xk10eS(2,3))];

    JHX10e(2:2:size(R1Zks,1),1:3) = [repmat([sin(Xk10eS(3,3)),-cos(Xk10eS(3,3))],size(R1Zks,1)/2,1), ...
        -cos(Xk10eS(3,3))*(R1Zks(1:2:(end-1),2)-Xk10eS(1,3))-sin(Xk10eS(3,3))*(R1Zks(2:2:end,2)-Xk10eS(2,3))];
    
    JHX10e(size(R1Zks,1)+(1:2:size(R2Zks,1)-1),4:6) = [repmat([-cos(Xk10eS(6,3)),-sin(Xk10eS(6,3))],size(R2Zks,1)/2,1), ...
        -sin(Xk10eS(6,3))*(R2Zks(1:2:(end-1),2)-Xk10eS(4,3))+cos(Xk10eS(6,3))*(R2Zks(2:2:end,2)-Xk10eS(5,3))];

    JHX10e(size(R1Zks,1)+(2:2:size(R2Zks,1)),4:6) = [repmat([sin(Xk10eS(6,3)),-cos(Xk10eS(6,3))],size(R2Zks,1)/2,1), ...
        -cos(Xk10eS(6,3))*(R2Zks(1:2:(end-1),2)-Xk10eS(4,3))-sin(Xk10eS(6,3))*(R2Zks(2:2:end,2)-Xk10eS(5,3))];


    R1DV = [];
    R2DV = [];
    for R1kj = 1:size(R1Zks,1)/2
        R1DV = blkdiag(R1DV,R1R);
        JHX10e((R1kj-1)*2+(1:2), R1Xfks_idx((R1kj-1)*2+(1:2),1)') = rotationMatrix(Xk10eS(3,3))';
    end
    for R2kj = 1:size(R2Zks,1)/2
        R2DV = blkdiag(R2DV,R2R);
        JHX10e(size(R1Zks,1)+(R2kj-1)*2+(1:2), R2Xfks_idx((R2kj-1)*2+(1:2),1)') = rotationMatrix(Xk10eS(6,3))';
    end
    
    % JHX10_e = full(JHX10e);
    DV = blkdiag(R1DV,R2DV);
    
    % Innovation Covariance S and Kalman Gain K
    S = JHX10e * Pk10S * JHX10e' + DV;
    K = Pk10S * JHX10e' /S;

    % Updating process using observation model
    Xk11e = Xk10eS;
    Xk11e(:,3) = Xk10eS(:,3) + K*(Zks(:,2)-HX10e);
    Pk11 = Pk10S - K*S*K';
    else
        Xk11e = Xk10eS;
        Pk11 = Pk10S;
    end
    
    XrR1_full = [XrR1_full;Xk11e(1:3,2:3)];
    XrR2_full = [XrR2_full;Xk11e(4:6,2:3)];

    Xk00e = Xk11e;
    Pk00 = Pk11;

end

figure

hold on 

plot(XrR1_full(1:3:(end-2),2),XrR1_full(2:3:(end-1),2),'--ro');
plot(XrR2_full(1:3:(end-2),2),XrR2_full(2:3:(end-1),2),'--mo');

plot(Xk11e(7:2:(end-1),3),Xk11e(8:2:end,3),'^','Color', [0.6, 0.4, 0.2])
text(Xk11e(7:2:(end-1),3),Xk11e(8:2:end,3), num2str(Xk11e(7:2:(end-1),2)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.6, 0.4, 0.2])

xlim([fea_xlb, fea_xub])
ylim([fea_ylb, fea_yub])

plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'-bo')
plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'-co')

% plot(Xf_true(:,2),Xf_true(:,3),'k^')
plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'k^')
text(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2), num2str(XfTrueAll(1:2:(end-1),1)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k')

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
