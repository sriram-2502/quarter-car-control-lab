% ME 4030: run this script from any working directory.
clear; close all; clc;
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','shared'));
control_mode = 'LQR'; % 'NONE', 'PID', or 'LQR'
road_input = 'step';  % 'step' or 'sine'
show_animation = true;
r = simulate_quarter_car(control_mode, road_input);
figure('Color','w','Name',[control_mode ' / ' road_input]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(r.t,[r.zs r.zu]); grid on;
title('Body and wheel'); ylabel('Displacement (m)'); legend('Body','Wheel');
nexttile; plot(r.t,r.zs-r.zu); grid on;
title('Suspension deflection'); ylabel('m');
nexttile; plot(r.t,[r.zs_dot r.zu_dot]); grid on;
title('Physical velocities'); ylabel('m/s'); legend('Body','Wheel');
nexttile; plot(r.t,r.zu-r.zr); grid on;
title('Tire deflection'); ylabel('m');
nexttile; plot(r.t,r.zr); grid on;
title(r.label); ylabel('Road (m)'); xlabel('Time (s)');
nexttile; plot(r.t,r.u/1000); grid on;
title('Actuator force'); ylabel('kN'); xlabel('Time (s)');
metrics = quarter_car_performance_eval(r);
disp(struct2table(metrics));
if show_animation
    opts = struct('carSpeed',1);
    uinfo = struct('t',r.t,'zr',r.zr,'label',r.label,'u',r.u);
    animate_quarter_car(r.t,r.zs,r.zu,r.zr,opts,uinfo);
end
