%% PENDULUM MODELING & IDENTIFICATION (Motor Off)
% Load experimental free-oscillation data
pendulum_data = readmatrix('data/angle_identification.csv', 'HeaderLines', 1);
angle_deg = pendulum_data(:, 2);
time_raw  = pendulum_data(:, 1);
time      = (time_raw - time_raw(1)) / 1000; % Convert ms to s

% Plot raw angular response
figure;
plot(time, angle_deg);
hold on;
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Pendulum Free-Oscillation Response');

% Extract positive peaks to build the decay envelope
[peaks, peak_idx] = findpeaks(angle_deg);
t_peaks = time(peak_idx);
t_peaks(peaks < 0)  = [];
peak_idx(peaks < 0) = [];
peaks(peaks < 0)    = [];

n_peaks = length(peaks);
plot(t_peaks(1:n_peaks), peaks(1:n_peaks), 'o');

envelope = interp1(peak_idx(1:n_peaks), peaks(1:n_peaks), 1:length(angle_deg), 'pchip');
plot(time, envelope, 'g-');

%% SECOND-ORDER OSCILLATOR RECONSTRUCTION
% Use only the first 50% of peaks to avoid tail noise distorting the period estimate
log_peaks = log(peaks);
n_valid   = floor(length(peaks) * 0.50);

y_fit = log_peaks(1:n_valid);
x_fit = 1:n_valid;

% Linear regression on logarithmic peak decay
p_fit   = polyfit(x_fit, y_fit, 1);
log_dec = -p_fit(1); % Logarithmic decrement
xi      = log_dec / sqrt(4 * pi^2 + log_dec^2);

Td_mean = mean(diff(t_peaks));
Td_est  = median(diff(t_peaks));
wd      = (2 * pi) / Td_est;
wn      = wd / sqrt(1 - xi^2);

fprintf('Damping ratio (xi): %.4f\n', xi);
fprintf('Natural frequency (wn): %.4f rad/s\n', wn);

% Reconstruct unforced second-order response (x1 = angle, x2 = angular velocity)
A_osc = [0 1; -wn^2 -2*xi*wn];
B_osc = [0; 0];
C_osc = [1 0];
D_osc = 0;

[angle_sim, time_sim, ~] = initial(ss(A_osc, B_osc, C_osc, D_osc), [angle_deg(1); 0], linspace(0, max(time), 1000));
plot(time_sim, angle_sim);

% Simulated decay envelope
[peaks_sim, peak_idx_sim] = findpeaks(angle_sim);
t_peaks_sim = time_sim(peak_idx_sim);
t_peaks_sim(peaks_sim < 0)  = [];
peak_idx_sim(peaks_sim < 0) = [];
peaks_sim(peaks_sim < 0)    = [];

plot(t_peaks_sim, peaks_sim, 'o');
envelope_sim = interp1(peak_idx_sim, peaks_sim, 1:length(angle_sim), 'pchip');
plot(time_sim, envelope_sim, '-');
legend('Experimental response', 'Experimental peaks', 'Experimental envelope', ...
       'Identified model', 'Simulated peaks', 'Simulated envelope');
hold off;

%% PENDULUM PHYSICAL PARAMETERS (J and b_theta)
M_p = 146.4 / 1000;   % Pendulum mass [kg]
d_m = 13.5e-2;        % Center of mass distance [m]
g   = 9.81;           % Gravity acceleration [m/s^2]

J       = (M_p * d_m * g) / (wn^2); % Total moment of inertia [kg*m^2]
b_theta = 2 * J * xi * wn;          % Pivot viscous friction coefficient [N*m*s/rad]

fprintf('Total moment of inertia (J): %.4f kg*m^2\n', J);
fprintf('Pendulum pivot friction (b_theta): %.4f N*m*s/rad\n', b_theta);

%% DC MOTOR & FLYWHEEL IDENTIFICATION (Pendulum Locked)
motor_data = readmatrix('data/motor_identification.csv', 'HeaderLines', 1);

omega_filtered = movmean(motor_data(:, 2), 10);
t_motor = motor_data(:, 1);
u_motor = motor_data(:, 3);
y_motor = motor_data(:, 2);
Ts      = mean(diff(t_motor));

motor_id_data = iddata(y_motor, u_motor, Ts);
np = 1;
nz = 0;
sys_tf = tfest(motor_id_data, np, nz);
sys_ss = ssest(motor_id_data, 1);

% Plot step response
figure;
plot(t_motor, u_motor);
hold on;
plot(t_motor, y_motor);
xlabel('Time [s]');
ylabel('Amplitude');
title('DC Motor Open-Loop Step Response');
legend('PWM Input (u)', 'Angular Velocity (y)');
hold off;

%% MOTOR & FLYWHEEL PARAMETERS (b_alpha and J_w)
% Datasheet electrical parameters
k_e = 0.0086;  % Back-EMF constant [V*s/rad]
k_m = 0.00971; % Torque constant [N*m/A]
Ra  = 3.08;    % Armature resistance [Ohm]

% Identified first-order flywheel dynamics
K_w     = dcgain(sys_tf);
p_motor = pole(sys_tf);
T_w     = -1 / p_motor;

b_alpha = (k_m / (K_w * Ra)) - ((k_m * k_e) / Ra); % Flywheel viscous friction [N*m*s/rad]
J_w     = T_w * (b_alpha + ((k_m * k_e) / Ra));     % Flywheel moment of inertia [kg*m^2]

fprintf('Flywheel moment of inertia (J_w): %.4f kg*m^2\n', J_w);
fprintf('Flywheel viscous friction (b_alpha): %.4f N*m*s/rad\n', b_alpha);

%% --- LQR CONTROLLER DESIGN ---

% 1. State-Space Matrices (A and B)
% State vector: x = [theta; theta_dot; alpha; alpha_dot]
% Note: A(2,1) is negative since linearization is around the downward equilibrium
motor_damping_term = b_alpha + (k_m * k_e) / Ra;
motor_torque_term  = k_m / Ra;
J_equiv            = (J + J_w) / (J * J_w);

A = zeros(4, 4);
A(1, 2) = 1;
A(2, 1) = -(M_p * d_m * g) / J;
A(2, 2) = -b_theta / J;
A(2, 4) = (1 / J) * motor_damping_term;
A(3, 4) = 1;
A(4, 1) = (M_p * d_m * g) / J;
A(4, 2) = b_theta / J;
A(4, 4) = -J_equiv * motor_damping_term;

B = zeros(4, 1);
B(2) = -(1 / J) * motor_torque_term;
B(4) = J_equiv * motor_torque_term;

C = eye(4);
D = zeros(4, 1);

% 2. Iterative Weight Tuning (Voltage Saturation Constraint)
target_voltage = 12.0;                 % Maximum supply voltage [V]
angle_test_deg = 30.0;                 % Initial deflection angle [deg]
angle_test_rad = deg2rad(angle_test_deg);

x_test = [angle_test_rad; 0; 0; 0];

% Fixed LQR weights
R           = 1;
q_omega     = 10;
q_alpha     = 0.01;
q_alpha_dot = 0;

best_q_theta   = 100;
best_K         = zeros(1, 4);
max_volt_found = 0;

fprintf('Starting LQR weight optimization for %.1f V limit at %.1f deg...\n', target_voltage, angle_test_deg);

for q_try = 100:10:1000
    Q_try = diag([q_try, q_omega, q_alpha, q_alpha_dot]);
    [K_try, ~, ~] = lqr(A, B, Q_try, R);

    u_calc   = -K_try * x_test;
    volt_now = abs(u_calc);

    if volt_now >= target_voltage
        fprintf('Voltage limit reached at q_theta = %d (V = %.3f V). Stopping search.\n', q_try, volt_now);
        break;
    else
        best_q_theta   = q_try;
        best_K         = K_try;
        max_volt_found = volt_now;
    end
end

fprintf('Optimization complete.\n');
fprintf('Optimal q_theta: %d\n', best_q_theta);
fprintf('Initial control voltage at 30 deg: %.2f V\n', max_volt_found);
fprintf('Optimal LQR gains K (for ESP32 firmware):\n');
disp(best_K);

K = best_K;
Q = diag([best_q_theta, q_omega, q_alpha, q_alpha_dot]);

%% --- CLOSED-LOOP SIMULATION ---
K(4) = 0; % Neglect flywheel velocity feedback in hardware implementation
sys_cl = ss(A - B*K, B, C, D);

x0    = [pi/6; 0; 0; 0]; % 30 deg initial angle
t_sim = 0:0.01:45;       % 45 seconds simulation
[y_cl, t_cl, x_cl] = initial(sys_cl, x0, t_sim);

figure;
subplot(3, 1, 1);
plot(t_cl, rad2deg(y_cl(:, 1)), 'LineWidth', 1.5);
grid on;
title('Pendulum Angular Position');
ylabel('Angle [deg]');
yline(0, 'k--');

subplot(3, 1, 2);
plot(t_cl, rad2deg(y_cl(:, 2)), 'LineWidth', 1.5);
grid on;
title('Pendulum Angular Velocity');
ylabel('Velocity [deg/s]');

subplot(3, 1, 3);
u_cl = -(K * x_cl')';
plot(t_cl, u_cl, 'r', 'LineWidth', 1.5);
grid on;
title('Motor Control Voltage (u)');
xlabel('Time [s]');
ylabel('Voltage [V]');

voltage_limit = 12;
if max(abs(u_cl)) > voltage_limit
    warning('Control effort exceeds the 12 V motor supply limit!');
end

%% --- EXPERIMENTAL CLOSED-LOOP VALIDATION ---
control_data = readmatrix('data/closed_loop_lqr.csv', 'HeaderLines', 1);
angle_exp    = control_data(:, 2);
time_exp_raw = control_data(:, 1);
time_exp     = time_exp_raw - time_exp_raw(1);
voltage_exp  = control_data(:, 3);

figure;
subplot(2, 1, 1);
plot(time_exp, angle_exp, 'LineWidth', 1.2);
grid on;
ylabel('Angle [deg]');
title('Experimental LQR Closed-Loop Response');

subplot(2, 1, 2);
plot(time_exp, voltage_exp, 'r', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Control Voltage [V]');
title('Experimental PWM Control Effort');