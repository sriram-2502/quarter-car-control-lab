% ME 4030 - Fall 2025 - final project
clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','shared'));

%% ---------------- Parameters (in SI units) ----------------
% TODO: input values here
Ms  = 0;     % (sprung)
Mu  = 0;     % (unsprung)
Ks  = 0;     % suspension spring
Cs  = 0;     % suspension damper
Kt  = 0;     % tire stiffness
Ct  = 0;     % tire damper

% simulation time (s)
Tend = 10.0;  dt = 1/1000;
t = (0:dt:Tend).';

% road input type
% TODO: pick 'sine' or 'step' based on the requirements
road_input = 'step'; % sine or step

% controller type
% TODO: pick 'LQR' or 'PID' based on the requiremnts
control_mode = 'NONE';   % 'LQR' or 'PID' or 'none'

% use PID tuner GUI?
% set true for tuning PID gains
% set false for simulation after getting the desired gains
% TODO: pick true or false based on requirements
use_pid_tuner = false; % true or false

%% ---------------- Road input z_r(t) ----------------
if strcmp(road_input, 'sine')
    % sine input
    Ar = 0.1;  fr = 0.5; % amplitude (m), frequency (Hz)
    zr = Ar * sin(2*pi*fr*t);
    zr_label = sprintf('Sine: %.2f m, %.2f Hz', Ar, fr);

elseif strcmp(road_input, 'step')
    % Step input
    zr = 0.2*(t>=1.0);
    zr_label = 'Step: 0.2 m @ t=1 s';
else
     error('Invalid road_input type. Choose ''sine'' or ''step''.');
end

%% ---------------- state-space model -----------------------------
% states x = [zs; zs_dot; suspension deflection; eta]
% Y is the transformed state eta, NOT suspension velocity. See docs/clarifications.md.
% system xdot = Ax + B1u + B2r
% TODO: fill out the entires
A = [ 0,   0,  0,  0;
      0,   0,  0,  0;
      0,   0,  0,  0;
      0,   0,  0,  0 ];

B1 = [ 0;
       0;
       0;
       0 ];

B2 = [ 0;
       0;
       0;
       0 ];

C = [0 0 0 0];
assert(all([Ms Mu Ks Cs Kt Ct]>0) && any(A(:)) && any(B1) && any(B2) && any(C), ...
    'Complete the parameters and state-space model before running.');

%% --------- Augment with integral of output y_I = integral(s) dt ---
% x_a = [x; integral of suspension deflection]
B = [B1, B2];
Aa = [[A, zeros(4,1)]; [C, 0]];
Ba = [B; 0 0];
Ca = [C, 0];

Bu  = Ba(:,1);    % control channel (active force)
Bw  = Ba(:,2);    % disturbance channel (road)

%% PID tuner GUI
% use MATLAB's PID tuner for this system
% tune the gains interactively
% for system with no disturbance (zr=0)
% note down the Kp Kd and Ki values from the bottom left of the PID tunder
% window
if(use_pid_tuner)
    Gu = ss(A, B(:,1), C, 0);    % Plant from control force U to output Xs
    pidTuner(Gu, 'PID');
    return
end

%% ---------------- Controller setup -------------------
F = 0; % road term in transformed PID feedback
switch upper(control_mode)
    case 'LQR'
        % ---- LQR on augmented plant (u = -K x_a) ----
        % TODO: tune the cost matrices
        Qx   = diag([0, 0, 0, 0]);  % states: zs, zs_dot, suspension deflection, eta
        Qi   = 0;                                 % integral state weight on y_I
        Qa   = blkdiag(Qx, Qi);
        R    = 0;                                 % control penalty
        K   = lqr(Aa, Bu, Qa, R);                   % LQR gain

    case 'PID'
        % ---- Input your PID gains from PID tuner ----
        % u = -Kp*s - Kd*s_dot - Ki*yI
        % TODO: tune the PID gains
        Kp = 0;
        Kd = 0;
        Ki = 0;

        % Map proportional, physical derivative, and integral feedback into transformed coordinates:
        K = [Kp*C + Kd*(C*A), Ki];
        F = -Kd*(C*B2);

    case 'NONE'
        K = [0, 0, 0, 0, 0];

    otherwise
        error('Unknown control_mode. Use ''LQR'', ''PID'', or ''NONE''.');
end

%% ---------------- Closed-loop simulation ----------------------
% TODO: define closeed loop system Aa - Bu*K
Acl = NaN(5,5); % TODO: replace with Aa - Bu*K
assert(all(isfinite(Acl(:))), 'Complete the closed-loop matrix Acl.');
Bcl = Bw + Bu*F; % include transformed PID road term

% closed loop system
sys_cl = ss(Acl, Bcl, eye(5), zeros(5,1));

x0a = zeros(5,1);                 % initial condition
xa = lsim(sys_cl, zr, t, x0a);    % simulate system
x  = xa(:,1:4);

% Recover physical displacements for animation:
X1 = x(:,1);
Y1 = x(:,3);
X2 = X1 - Y1;

zs = X1;                 % body absolute displacement
zu = X2;                 % wheel absolute displacement

% Control history
u  = -(xa * K.').' + F*zr.';      % u = -K * x_a
u  = u(:);

%% -------- plots: states, control, and input (time domain) --------
% x = [zs; zs_dot; suspension deflection; eta]. Recover physical velocity.
X1  = x(:,1);               % z_s
X1d = x(:,2);               % \dot z_s
Y1  = x(:,3);               % z_s - z_u
Y1d = x*A(3,:).' + B2(3)*zr;               % \dot z_s - \dot z_u
X2  = X1 - Y1;              % z_u
X2d = X1d - Y1d;            % \dot z_u
tire_defl = X2 - zr;        % z_u - z_r  (tire compression)

fig = figure('Color','w','Name','Quarter-Car: Static Plots','NumberTitle','off');
tl = tiledlayout(fig, 3, 2, 'TileSpacing','compact','Padding','compact');

% Row 1, Col 1: body & wheel displacements
ax11 = nexttile(tl,1); hold(ax11,'on'); grid(ax11,'on'); box(ax11,'on');
plot(ax11, t, X1, 'LineWidth',1.6);     % z_s
plot(ax11, t, X2, 'LineWidth',1.6);     % z_u
legend(ax11, {'$z_s$ (body)', '$z_u$ (wheel)'}, 'Interpreter','latex','Location','best');
xlabel(ax11, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax11, '$\mathrm{displacement}~(\mathrm{m})$', 'Interpreter','latex');
title(ax11, '$z_s$ and $z_u$', 'Interpreter','latex');

% Row 1, Col 2: suspension deflections
ax12 = nexttile(tl,2); hold(ax12,'on'); grid(ax12,'on'); box(ax12,'on');
plot(ax12, t, Y1,         'LineWidth',1.6);  % suspension deflection
legend(ax12, {'$z_s - z_u$'}, 'Interpreter','latex','Location','best');
xlabel(ax12, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax12, '$\mathrm{deflection}~(\mathrm{m})$', 'Interpreter','latex');
title(ax12, 'Suspension Deflections', 'Interpreter','latex');

% Row 2, Col 2: tire deflections
ax22 = nexttile(tl,4); hold(ax22,'on'); grid(ax22,'on'); box(ax22,'on');
plot(ax22, t, tire_defl,  'LineWidth',1.6);  % tire deflection
legend(ax22, {'$z_u - z_r$'}, 'Interpreter','latex','Location','best');
xlabel(ax22, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax22, '$\mathrm{deflection}~(\mathrm{m})$', 'Interpreter','latex');
title(ax22, 'Tire Deflections', 'Interpreter','latex');

% Row 2, Col 1: velocities
ax21 = nexttile(tl,3); hold(ax21,'on'); grid(ax21,'on'); box(ax21,'on');
plot(ax21, t, X1d, 'LineWidth',1.6);    % \dot z_s
plot(ax21, t, X2d, 'LineWidth',1.6);    % \dot z_u
legend(ax21, {'$\dot z_s$', '$\dot z_u$'}, 'Interpreter','latex','Location','best');
xlabel(ax21, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax21, '$\mathrm{velocity}~(\mathrm{m/s})$', 'Interpreter','latex');
title(ax21, 'Velocities', 'Interpreter','latex');

% Row 2, Col 2: control input u(t)
ax23 = nexttile(tl,6); hold(ax23,'on'); grid(ax23,'on'); box(ax23,'on');
plot(ax23, t, u, 'LineWidth',1.6);
xlabel(ax23, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax23, '$u~(\mathrm{N})$', 'Interpreter','latex');
title(ax23, 'Active Suspension Force', 'Interpreter','latex');

% Row 3, Col 1: road input
ax31 = nexttile(tl,5); hold(ax31,'on'); grid(ax31,'on'); box(ax31,'on');
plot(ax31, t, zr, 'LineWidth',1.6);
xlabel(ax31, '$t~(\mathrm{s})$', 'Interpreter','latex');
ylabel(ax31, '$z_r~(\mathrm{m})$', 'Interpreter','latex');
title(ax31, 'Road Input', 'Interpreter','latex');

% LaTeX tick labels everywhere + link time axes
set([ax11,ax12,ax21,ax22,ax31], 'TickLabelInterpreter','latex');
linkaxes([ax11,ax12,ax21,ax22,ax31],'x');
xlim([t(1) t(end)]);

style_quarter_car_figure(fig);

%% ---------------- Animation  -----------------------------------
opts = struct();
opts.carSpeed = 1;

% Pass control (u) and road (zr) to the animator
uinfo = struct('t', t, 'zr', zr, 'label', zr_label, 'u', u);
animate_quarter_car(t, zs, zu, zr, opts, uinfo);

%% Evaluate performance metrics
% TODO: implement the metrics in docs/project.pdf.
% For accurate RMS values at sharp steps, refine the integration or integrate
% squared outputs over each interval as explained in the solution guide.
