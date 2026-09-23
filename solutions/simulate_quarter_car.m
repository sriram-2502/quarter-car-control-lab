function result = simulate_quarter_car(control_mode, road_input)
%SIMULATE_QUARTER_CAR Reference linear active-suspension simulation.
% result = simulate_quarter_car('LQR', 'step');
% Modes: NONE, PID, LQR. Inputs: step, sine. SI units throughout.
if nargin < 1, control_mode = 'LQR'; end
if nargin < 2, road_input = 'step'; end
p = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), 'config.json')));
Ms=p.Ms; Mu=p.Mu; Ks=p.Ks; Cs=p.Cs; Kt=p.Kt; Ct=p.Ct;
alpha = Cs/Ms + Cs/Mu + Ct/Mu;
% x = [zs; zs_dot; s; eta], s = zs-zu.
% eta = s_dot - (Ct/Mu)*zs + alpha*s + (Ct/Mu)*zr.
A = [0, 1, 0, 0;
    -Cs*Ct/(Ms*Mu), 0, Cs/Ms*alpha-Ks/Ms, -Cs/Ms;
    Ct/Mu, 0, -alpha, 1;
    Kt/Mu, 0, -(Ks/Ms+Ks/Mu+Kt/Mu), 0];
Bu = [0; 1/Ms; 0; 1/Ms+1/Mu];
Bw = [0; Cs*Ct/(Ms*Mu); -Ct/Mu; -Kt/Mu];
C = [0 0 1 0];
Aa = [A zeros(4,1); C 0];
Bua = [Bu; 0]; Bwa = [Bw; 0];
% u = -K*xa + F*zr. PID differentiates s, not eta.
F = 0;
switch upper(control_mode)
    case 'LQR'
        K = lqr(Aa, Bua, diag(p.Q), p.R);
    case 'PID'
        K = [p.Kp*C + p.Kd*(C*A), p.Ki];
        F = -p.Kd*(C*Bw);
    case 'NONE'
        K = zeros(1,5);
    otherwise
        error('Choose NONE, PID, or LQR.');
end
t = (0:p.dt:p.duration).';
switch lower(road_input)
    case 'step'
        zr = p.step_amplitude*(t >= p.step_time);
        label = sprintf('Step: %.2f m at t=%.1f s',p.step_amplitude,p.step_time);
    case 'sine'
        zr = p.sine_amplitude*sin(2*pi*p.sine_frequency*t);
        label = sprintf('Sine: %.2f m, %.2f Hz',p.sine_amplitude,p.sine_frequency);
    otherwise
        error('Choose step or sine.');
end
Acl = Aa-Bua*K; Bcl = Bwa+Bua*F;
% Exact continuous-input propagation using a road exosystem.
% The step is constant between samples; the sine is continuous, not stair-stepped.
if strcmpi(road_input,'step'), n=6; else, n=7; end
H=zeros(n); H(1:5,1:5)=Acl; H(1:5,6)=Bcl;
if n==7
    H(6,7)=1; H(7,6)=-(2*pi*p.sine_frequency)^2;
end
E=expm(H*p.dt); z=zeros(numel(t),n);
if n==7, z(1,7)=p.sine_amplitude*2*pi*p.sine_frequency; end
for j=2:numel(t)
    z(j,:)=(E*z(j-1,:).').';
    if n==6, z(j,6)=zr(j); end
end
xa=z(:,1:5);
% Integrate squared outputs exactly on each interval; coarse trapezoids can
% misrepresent the fast ideal-PID transient at a sharp road step.
hu=zeros(1,n); hu(1:5)=-K; hu(6)=F;
hs=zeros(1,n); hs(1:4)=A(3,:); hs(6)=Bw(3);
ha=(-Cs*hs+hu)/Ms; ha(3)=ha(3)-Ks/Ms;
ht=zeros(1,n); ht(1)=1; ht(3)=-1; ht(6)=-1;
outputs=[ha;ht;hu]; energies=zeros(1,3);
for j=1:3
    h=outputs(j,:);
    V=expm([-H.', h.'*h; zeros(n), H]*p.dt);
    G=E.'*V(1:n,n+1:end);
    energies(j)=sum(sum((z(1:end-1,:)*G).*z(1:end-1,:)));
end
x = xa(:,1:4);
s_dot = x*A(3,:).' + Bw(3)*zr;
u = -xa*K.' + F*zr;
zs = x(:,1); zu = zs-x(:,3);
zs_dot = x(:,2); zu_dot = zs_dot-s_dot;
zs_ddot = (-Ks*x(:,3)-Cs*s_dot+u)/Ms;
result = struct('t',t,'zr',zr,'zs',zs,'zu',zu,'zs_dot',zs_dot, ...
    'zu_dot',zu_dot,'s_dot',s_dot,'zs_ddot',zs_ddot,'u',u, ...
    'xa',xa,'K',K,'F',F,'poles',eig(Acl),'mode',upper(control_mode), ...
    'road',lower(road_input),'label',label,'parameters',p,'energies',energies);
end
