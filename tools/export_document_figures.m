% ME 4030: export corrected project figures directly from MATLAB.
% The original supplied images are preserved; these are separate build assets.
clear; close all; clc;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'solutions'), fullfile(root,'shared'));
outdir = fullfile(root,'latex','figures');
gendir = fullfile(root,'latex','generated');
if ~isfolder(gendir), mkdir(gendir); end
checks = struct();
for road = {'step','sine'}
    for mode = {'NONE','PID','LQR'}
        %% Simulate the corrected continuous-time system
        r = simulate_quarter_car(mode{1},road{1});
        metrics = quarter_car_performance_eval(r);
        check = struct('metrics',metrics,'K',r.K);
        if strcmp(road{1},'step')
            i10 = find(r.t>=r.parameters.step_time & r.zs>=0.1*r.parameters.step_amplitude,1);
            i90 = find(r.t>=r.parameters.step_time & r.zs>=0.9*r.parameters.step_amplitude,1);
            assert(~isempty(i10) && ~isempty(i90),'Body rise thresholds not reached.');
            check.body_rise_time_s = r.t(i90)-r.t(i10);
        end
        checks.(road{1}).(mode{1}) = check;
        %% Original six-panel MATLAB plotting layout
        fig = figure('Visible','off','Color','w','Position',[100 100 1200 620]);
        tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
        nexttile; plot(r.t,[r.zs r.zu],'LineWidth',1.1); grid on;
        title('Body and wheel'); ylabel('Displacement (m)'); legend('Body','Wheel','Location','best');
        nexttile; plot(r.t,r.zs-r.zu,'LineWidth',1.1); grid on;
        title('Suspension deflection'); ylabel('m');
        nexttile; plot(r.t,[r.zs_dot r.zu_dot],'LineWidth',1.1); grid on;
        title('Physical velocities'); ylabel('m/s'); legend('Body','Wheel','Location','best');
        nexttile; plot(r.t,r.zu-r.zr,'LineWidth',1.1); grid on;
        title('Tire deflection'); ylabel('m');
        nexttile; plot(r.t,r.zr,'LineWidth',1.1); grid on;
        title(r.label); ylabel('Road (m)'); xlabel('Time (s)');
        nexttile; plot(r.t,r.u/1000,'LineWidth',1.1); grid on;
        title('Actuator force'); ylabel('kN'); xlabel('Time (s)');
        style_quarter_car_figure(fig);
        set(findall(fig,'Type','axes'),'FontSize',12);
        label = lower(mode{1});
        if strcmp(mode{1},'NONE'), label='passive'; end
        exportgraphics(fig,fullfile(outdir,['corrected_' label '_' road{1} '.png']), ...
            'Resolution',180,'BackgroundColor','white');
        close(fig);
    end
end
fid = fopen(fullfile(gendir,'matlab-document-checks.json'),'w');
assert(fid>=0,'Cannot write native document checks.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(checks,'PrettyPrint',true));
disp('Exported all six corrected MATLAB figures and native document checks.');
