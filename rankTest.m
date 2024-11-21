clc
clear
% N1 = [-eye(2),rand(2,1),zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
%     zeros(2,2),zeros(2,1),-eye(2),rand(2,1),zeros(2,2),eye(2);
%     -eye(2),rand(2,1),zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
%     zeros(2,2),zeros(2,1),-eye(2),rand(2,1),zeros(2,2),eye(2);
%     -eye(2),rand(2,1),zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
%     zeros(2,2),zeros(2,1),-eye(2),rand(2,1),zeros(2,2),eye(2);
%     -eye(2),rand(2,1),zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
%     zeros(2,2),zeros(2,1),-eye(2),rand(2,1),zeros(2,2),eye(2)];
% 
% disp('The rank of N1 is:');
% disp(rank(N1));
% null_N1 = null(N1);
% disp('The null space of N1 is:');
% disp(null_N1);
J = [0,-1;1,0];
syms a b c d e f g h x1 x2 x3 x4 x5 x6 x7 x8 % Define Symbolic Variables
N2 = [-eye(2),-J*[a-b;c-d],zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
      -eye(2),-J*[e-b;f-d],zeros(2,2),zeros(2,1),zeros(2,2),eye(2);
      zeros(2,2),zeros(2,1),-eye(2),-J*[a-g;c-h],eye(2),zeros(2,2);
      zeros(2,2),zeros(2,1),-eye(2),-J*[e-g;f-h],zeros(2,2),eye(2)];       % Define a matrix with letters
% x = [x1; x2; x3; x4; x5; x6; x7; x8];          % Define unknown vectors
% eqns = N2*x == 0;           % Matrix equation A*x = 0
% S = solve(eqns, [x1 x2 x3 x4 x5 x6 x7 x8]);  %Solving the matrix equation
% disp(S);
null_N2 = null(N2);
disp('The null space of N2 is:')
disp(null_N2)

N3 = simplify([null_N2(:,3)+null_N2(:,1),null_N2(:,2),null_N2(:,3)]);
disp('The null space of N3 is:')
disp(N3)

N4 = [N3(:,2),N3(:,1),N3(:,3)];
disp('The null space of N4 is:')
disp(N4)

N5 = N4([1,2,3,6,4,5,7,8,9,10],:);
disp('The null space of N5 is:')
disp(N5)

N6 = [N5(:,1:2),N5(:,3)];
disp('The null space of N6 is:')
disp(N6)

N7 = N6([1,2,3,4,5,6,7,10,9,8],:);
disp('The null space of N7 is:')
disp(N7)

N8 = [-eye(2),[a;b],zeros(2,2),zeros(2,1),eye(2),zeros(2,2);
      zeros(2,2),[c;d],zeros(2,2),zeros(2,1),-eye(2),eye(2);
      zeros(2,2),zeros(2,1),-eye(2),[e;f],eye(2),zeros(2,2);
      zeros(2,2),zeros(2,1),zeros(2,2),[c;d],-eye(2),eye(2)];
disp('The rank of N8 is:')
rank(N8)
cs_N8 = colspace(N8);
disp('The column space of N8 is:')
disp(cs_N8);