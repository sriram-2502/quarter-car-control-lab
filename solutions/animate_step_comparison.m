% ANIMATE_STEP_COMPARISON Standalone passive-versus-controlled step animation.
% Run this script directly. It reuses the validated model and MATLAB animator.
% Default: LQR. To select PID, set comparison_mode='PID' before running.
if ~exist('comparison_mode','var'), comparison_mode='LQR'; end
assert(any(strcmpi(comparison_mode,{'PID','LQR'})), 'Choose PID or LQR.');
here=fileparts(mfilename('fullpath'));
repo_root=fileparts(here);
addpath(here,fullfile(repo_root,'shared'));
controlled=simulate_quarter_car(comparison_mode,'step');
passive=simulate_quarter_car('NONE','step');
assert(isequal(controlled.t,passive.t) && isequal(controlled.zr,passive.zr), ...
    'Comparison must use identical times and road inputs.');
base_name=[lower(comparison_mode) '-step-comparison'];
opts=struct('carSpeed',1,'frameSkip',100,'pausePerFrame',0, ...
    'figurePosition',[100 100 1000 750], ...
    'title',[upper(comparison_mode) ' vs passive | ' controlled.label], ...
    'gifPath',fullfile(repo_root,'media','animations',[base_name '.gif']), ...
    'screenshotPath',fullfile(here,'figures',[base_name '.png']), ...
    'frameCallback',@(fig,k) passive_overlay(fig,k,passive,controlled));
info=struct('t',controlled.t,'zr',controlled.zr,'label',controlled.label, ...
    'u',controlled.u,'zs_dot',controlled.zs_dot,'zu_dot',controlled.zu_dot);
animate_quarter_car(controlled.t,controlled.zs,controlled.zu,controlled.zr,opts,info);
% Validate the reference data actually attached to the exported figure.
references=findall(gcf,'Type','line','Tag','PassiveStepReference');
assert(numel(references)==5,'Expected passive references in all five time plots.');
fields={'zs','zs_dot','zu','zu_dot','u'};
for j=1:numel(references)
    field=get(references(j),'UserData');
    assert(any(strcmp(field,fields)),'Unexpected reference quantity.');
    assert(isequal(get(references(j),'XData'),passive.t.'), 'Reference time mismatch.');
    assert(isequal(get(references(j),'YData'),passive.(field).'), 'Reference data mismatch.');
end
metadata=imfinfo(opts.gifPath);
assert(numel(metadata)==101 && abs(sum([metadata.DelayTime])/100-11)<.02, ...
    'Unexpected comparison animation timing.');
fprintf('EXPORTED %s: five verified passive traces, 101 frames, 11 s playback.\n',base_name);

function passive_overlay(fig,~,baseline,active)
% Full passive histories are a fixed backdrop; controlled traces animate above.
if isappdata(fig,'PassiveStepReferenceAdded'), return; end
labels={'z_s (m)','dz_s/dt (m/s)','z_u (m)','dz_u/dt (m/s)','u (N)'};
fields={'zs','zs_dot','zu','zu_dot','u'};
axes_handles=findall(fig,'Type','axes');
for j=1:numel(labels)
    ax=[];
    for a=1:numel(axes_handles)
        if strcmp(get(get(axes_handles(a),'YLabel'),'String'),labels{j})
            ax=axes_handles(a); break;
        end
    end
    assert(~isempty(ax),'Missing comparison axis: %s.',labels{j});
    active_line=findobj(ax,'Type','line','LineWidth',1.4);
    h=plot(ax,baseline.t,baseline.(fields{j}),'Color',[.68 .80 .94], ...
        'LineWidth',1.6,'Tag','PassiveStepReference','UserData',fields{j});
    uistack(h,'bottom');
    combined=[baseline.(fields{j});active.(fields{j})];
    low=min(combined); high=max(combined); pad=.04*max(high-low,1e-6);
    ylim(ax,[low-pad,high+pad]);
    if j==1
        legend(ax,[active_line(1),h],{active.mode,'Passive (uncontrolled)'}, ...
            'Location','northeast','FontSize',8,'AutoUpdate','off');
    end
end
setappdata(fig,'PassiveStepReferenceAdded',true);
style_quarter_car_figure(fig);
end
