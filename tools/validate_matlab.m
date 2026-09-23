function report = validate_matlab(output_dir)
%VALIDATE_MATLAB Native numerical, graphics, and student-entry-point checks.
% Run from any directory after adding tools/ to the MATLAB path.
if nargin < 1, output_dir = fullfile(tempdir,'quarter-car-matlab-validation'); end
if ~exist(output_dir,'dir'), mkdir(output_dir); end
root = fileparts(fileparts(mfilename('fullpath')));
original_dir=pwd;
restore_dir=onCleanup(@() cd(original_dir));
cd(root); % current-folder functions otherwise shadow the repository path
addpath(fullfile(root,'solutions'),fullfile(root,'shared'));
expected = jsondecode(fileread(fullfile(root,'solutions','results.json')));
old_visibility = get(groot,'DefaultFigureVisible');
restore = onCleanup(@() set(groot,'DefaultFigureVisible',old_visibility));
set(groot,'DefaultFigureVisible','off');
report = struct('matlab_version',version,'release',version('-release'), ...
    'control_system_toolbox',ver('control'));
report.cases=cell(1,6);
modes = {'NONE','PID','LQR'}; roads = {'step','sine'};
case_index = 0;
for road_index = 1:numel(roads)
    for mode_index = 1:numel(modes)
        mode = modes{mode_index}; road = roads{road_index};
        r = simulate_quarter_car(mode,road);
        actual = quarter_car_performance_eval(r);
        reference = expected.(road).(mode);
        fields = fieldnames(reference);
        max_normalized_error = 0;
        for j = 1:numel(fields)
            field = fields{j}; wanted = reference.(field); observed = actual.(field);
            if isempty(wanted)
                assert(isnan(observed),'Expected NaN for %s/%s/%s.',mode,road,field);
            elseif ischar(wanted)
                assert(strcmp(wanted,'not settled') && isinf(observed), ...
                    'Expected not settled for %s/%s/%s.',mode,road,field);
            else
                normalized_error = abs(observed-wanted)/max(1,abs(wanted));
                max_normalized_error = max(max_normalized_error,normalized_error);
                assert(normalized_error<1e-6,'Metric mismatch: %s/%s/%s.',mode,road,field);
            end
        end
        assert(all(isfinite(r.xa(:))) && all(isfinite(r.u)),'Nonfinite simulation.');
        assert(all(r.energies>=0),'Negative output energy.');
        if strcmp(mode,'NONE')
            assert(all(r.u==0),'Passive actuator force must be zero.');
            assert(sum(real(r.poles)<-1e-8)==4,'Passive mechanical poles are not stable.');
        else
            assert(all(real(r.poles)<0),'Unstable active closed loop.');
        end
        if strcmp(mode,'PID')
            p=r.parameters;
            force=-p.Kp*(r.zs-r.zu)-p.Kd*r.s_dot-p.Ki*r.xa(:,5);
            assert(max(abs(force-r.u))<1e-6,'PID physical-force mismatch.');
        end
        % Check physical sprung-mass force balance, independently of RMS code.
        p=r.parameters;
        residual=p.Ms*r.zs_ddot+p.Ks*(r.zs-r.zu)+p.Cs*(r.zs_dot-r.zu_dot)-r.u;
        assert(max(abs(residual))<1e-6,'Body force-balance mismatch.');
        info=struct('t',r.t,'zr',r.zr,'label',r.label,'u',r.u,'zs_dot',r.zs_dot,'zu_dot',r.zu_dot);
        opts=struct('carSpeed',1,'frameSkip',500,'pausePerFrame',0);
        animate_quarter_car(r.t,r.zs,r.zu,r.zr,opts,info);
        drawnow;
        ax=findall(gcf,'Type','axes');
        for a=1:numel(ax)
            assert(isequal(get(ax(a),'Color'),[1 1 1]),'Unreadable theme background.');
        end
        exportgraphics(gcf,fullfile(output_dir,[lower(mode) '-' road '-animation.png']), ...
            'Resolution',120);
        close all;
        % Keep not-settled values distinct from N/A in the JSON report.
        serialized=actual;
        for j=1:numel(fields)
            if isinf(serialized.(fields{j}))
                serialized.(fields{j})='not settled';
            end
        end
        case_index=case_index+1;
        report.cases{case_index}=struct('mode',mode,'road',road,'metrics',serialized, ...
            'max_normalized_metric_error',max_normalized_error,'graphics_passed',true);
        fprintf('PASS native MATLAB: %s / %s (max metric error %.3g)\n', ...
            mode,road,max_normalized_error);
    end
end
% Also exercise the helper's optional-argument and zero-motion fallback.
animate_quarter_car([0;.5;1],zeros(3,1),zeros(3,1),zeros(3,1));
drawnow; close all;
report.optional_animation_arguments_passed=true;
% Exercise the shipped demonstration without modifying its settings.
run_demo(fullfile(root,'solutions','quarter_car_demo_solution.m'));
figures=findall(groot,'Type','figure');
assert(numel(figures)==2,'Demo should produce static and animation figures.');
for j=1:numel(figures)
    exportgraphics(figures(j),fullfile(output_dir,sprintf('demo-figure-%d.png',j)), ...
        'Resolution',120);
end
close all;
report.demo_passed=true;
% Student template is intentionally incomplete and must stop with its clear guard.
check_starter(fullfile(root,'starter','main_file.m'));
report.starter_guard_passed=true;
report.all_passed=true;
fid=fopen(fullfile(output_dir,'matlab-validation.json'),'w');
assert(fid>=0,'Cannot write validation report.');
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true)); fclose(fid);
fprintf('PASS native demo, all six animation paths, and starter guard.\n');
end

function run_demo(path)
run(path);
drawnow;
end

function check_starter(path)
try
    run(path);
catch exception
    assert(contains(exception.message,'Complete the parameters and state-space model'), ...
        'Starter failed for an unexpected reason: %s',exception.message);
    fprintf('PASS expected student-template guard.\n');
    return;
end
error('Incomplete student template unexpectedly ran.');
end
