%{
IRFAN YAFI PRANOTO | 13122077
Three Wheeled Omnidirectional Mobile Robot
%}

clear, clc
%% MODEL PARAMETER
d = 0.1;
r = 0.035;
l = 19/2;
Kvt = 0.0059;
R = 1.69;
La = 0.00011;
M = 1.5;
J = 0.025;
Bv = 0.94;
Bvn = 0.96;
Bw = 0.01;
Cv = 2.2;
Cvn = 1.5;
Cw = 0.099;
delta = deg2rad(30);

%% KINEMATIC MODEL
R_trans_wheel2Body = inv([0, r*sqrt(3)/3, -r*sqrt(3)/3;
                      2*r/2, r/3, r/3;
                      r/(3*d), r/(3*d), r/(3*d)]);

% R_trans_wheel2Body = 1/r * [-sin(pi/3) cos(pi/3) d;
%              0 -1 d;
%              sin(pi/3) cos(pi/3) d];

%% STATE SPACE MODEL
A11 = -3 * Kvt^2 * l^2 / (2 * r^2 * R * M) - Bv/M;
A22 = -3 * Kvt^2 * l^2 / (2 * r^2 * R * M) - Bvn/M;
A33 = -3 * d^2 * Kvt^2 * l^2 / (r^2 * R * J) - Bw/J;
A_mdl = [A11 0 0;
         0 A22 0;
         0 0 A33];

B_mdl = l*Kvt/(r*R) * [0 sqrt(3)/(2*M), -sqrt(3)/(2*M);
                   -1/M, 1/(2*M), 1/(2*M);
                   d/J, d/J, d/J];

K_mdl = [-Cv/M 0 0;
          0 -Cvn/M 0;
          0 0 -Cw/J];
C_mdl = eye(3);

inv_B_mdl = inv(B_mdl);
g1 = 8.33;
g2 = 8.23;
g3 = 8.2;
Gamma = diag([g1, g2, g3]);
K1 =80;
beta = 0.85;
K2 = beta*K1;
gainError = diag([10, 10, 10]);
gainInput = diag([7; 7; 7]);

Q = 3*[1 0 0;
    0 1 0;
    0 0 1];
R = [1 0 0;
    0 1 0;
    0 0 1];

K_lqr = lqr(zeros(3), eye(3), Q, R);

Q_smc = 0.1*diag([1, 1, 1]);
delta = 1;
k1_smc = diag([0.7206586704988028 0.846888480151037 0.848909652523])
k2_smc = diag([1.3 1.32370704517714 1.1]) * k1_smc
% k1_smc = diag([1.206586704988028 1.846888480151037 1.7848909652523]);
% k2_smc = diag([1.3 1.432370704517714 1.1]) * k1_smc;
% k1_smc = diag([45.206586704988028 40.846888480151037 40.7848909652523]);
% k2_smc = diag([1.5 0.432370704517714 5]) * k1_smc;

%% LQR
A_aug = [A_mdl eye(3); zeros(3,6)];
B_aug = [B_mdl; zeros(3,3)];
C_aug = [C_mdl zeros(3,3)];
L_obsv = [diag([5,5,5]);diag([5,5,5])];

syms s
vpa(solve(det(s*eye(6) - (A_aug - L_obsv*C_aug)) == 0))

% Define error dynamics matrices
A_e = -C_mdl * A_mdl;
B_e = -C_mdl * B_mdl;

% Define weighting matrices
Q_error = diag([40, 40, 40]); % Penalizes the error
R_error = diag([1, 1, 1]);    % Penalizes the control effort

% Compute LQR gain
K_error = lqr(A_e, B_e, Q_error, R_error);


%%

% % Define parameter bounds (e.g., [min, max] for each parameter)
% lb = [0.1, 0.1, 0.1, 0.001, 0.001, 0.001];    % Lower bounds for [Q_smc, k1_smc, k2_smc]
% ub = [500, 500, 500, 50, 50, 50];   % Upper bounds for [Q_smc, k1_smc, k2_smc]
% 
% % Number of parameters to optimize
% numParameters = 6;
% 
% % Set PSO options
% options = optimoptions('particleswarm', ...
%     'SwarmSize', 20, ...         % Number of particles in the swarm
%     'MaxIterations', 30, ...     % Maximum iterations
%     'Display', 'iter', ...       % Display iteration details
%     'UseParallel', false);        % Use parallel computation (if available)
% 
% % Run PSO to minimize the cost function
% [bestParams, bestCost] = particleswarm(@objFunc, numParameters, lb, ub, options);
% 
% % Display the results
% disp('Optimal Parameters:');
% disp(['k1_smc: ', mat2str(bestParams(1:3))]); % Display vector Q_smc
% disp(['k2_smc_gain: ', mat2str(bestParams(4:6))]);
% 
% disp(['Optimal Cost: ', num2str(bestCost)]);
% 
% %% Objective Function
% function cost = objFunc(params)
%     % Define Simulink model name
%     mdl = "ver2_control";
%     set_param(mdl, "ReturnWorkspaceOutputs", "on");
% 
%     % Assign vector Q_smc and scalar k1_smc, k2_smc to Simulink workspace
%     k1_smc = diag(params(1:3));  % Extract k1_smc
%     k2_smc = diag([params(4), params(5), params(6)])*k1_smc;  % Extract k2_smc
% 
%     assignin('base', 'k1_smc', k1_smc); 
%     assignin('base', 'k2_smc', k2_smc);
% 
%     % Run the Simulink model
%     try
%         mySimOut = sim(mdl, 'StopTime', '10');
% 
%         % Extract cost from Simulink logsout (adjust signal name as needed)
%         cost = mySimOut.logsout.getElement('costValue').Values.Data(end); % Get the final value of cost signal
%     catch ME
%         warning('Simulation failed: %s', ME.message);
%         cost = inf; % Assign a high cost if simulation fails
%     end
% end

