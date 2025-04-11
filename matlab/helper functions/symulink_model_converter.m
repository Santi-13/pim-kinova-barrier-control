[A, B, C, D] = linmod('dynamic_system_alone');
sys = ss(A, B, C, D);

%% LQR
Q_pos = 10 * eye(6);    % Penalize position errors
Q_vel = 1 * eye(6);     % Penalize velocity errors
Q = blkdiag(Q_pos, Q_vel);

R = 0.1 * eye(6); % Penalize large control efforts

[K, S, CLP] = lqr(A, B, Q, R);

% Closed-loop system with state feedback
sys_cl = ss(A - B*K, B, C, D);

% Simulate step response for all joints
t = 0:0.01:10; % Time vector
t_size = size(t);
u = zeros(t_size(2),6);
x0 = zeros(12, 1); % Initial state (zero)
[y, t, x] = lsim(sys_cl, u, t, x0);

% Plot joint positions (first 6 states)
figure;
plot(t, y(:, 1:6));
xlabel('Time (s)');
ylabel('Joint Positions');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

Co = ctrb(A, B);
if rank(Co) == length(A)
    disp('System is controllable');
else
    error('System is uncontrollable! Check actuators or linearization.');
end