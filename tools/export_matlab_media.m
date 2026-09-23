function export_matlab_media(modes,roads)
%EXPORT_MATLAB_MEDIA Export actual MATLAB animation frames and screenshots.
% From repository root: addpath('tools'); export_matlab_media;
if nargin<1, modes={'NONE','PID','LQR'}; end
if nargin<2, roads={'step','sine'}; end
root=fileparts(fileparts(mfilename('fullpath')));
old_dir=pwd; restore_dir=onCleanup(@() cd(old_dir)); cd(root);
addpath(fullfile(root,'solutions'),fullfile(root,'shared'));
old_visibility=get(groot,'DefaultFigureVisible');
restore_visibility=onCleanup(@() set(groot,'DefaultFigureVisible',old_visibility));
set(groot,'DefaultFigureVisible','off');
for j=1:numel(roads)
    for i=1:numel(modes)
        r=simulate_quarter_car(modes{i},roads{j});
        label=lower(modes{i}); display_name=modes{i};
        if strcmpi(modes{i},'NONE'), label='passive'; display_name='Passive'; end
        basename=[label '-' roads{j}];
        opts=struct('carSpeed',1,'frameSkip',100,'pausePerFrame',0, ...
            'figurePosition',[100 100 1000 750], ...
            'title',[display_name ' | ' r.label], ...
            'gifPath',fullfile(root,'media','animations',[basename '.gif']), ...
            'screenshotPath',fullfile(root,'solutions','figures',[basename '.png']));
        info=struct('t',r.t,'zr',r.zr,'label',r.label,'u',r.u, ...
            'zs_dot',r.zs_dot,'zu_dot',r.zu_dot);
        animate_quarter_car(r.t,r.zs,r.zu,r.zr,opts,info);
        meta=imfinfo(opts.gifPath);
        expected_frames=numel(unique([1:opts.frameSkip:numel(r.t),numel(r.t)]));
        assert(numel(meta)==expected_frames,'Unexpected GIF frame count.');
        % imfinfo reports GIF delays in hundredths of a second.
        expected_duration=r.t(end)-r.t(1)+1;
        assert(abs(sum([meta.DelayTime])/100-expected_duration)<0.02, ...
            'Unexpected GIF duration.');
        assert(isfile(opts.screenshotPath),'Missing screenshot.');
        fprintf('EXPORTED %s: %d MATLAB frames, %.1f s playback\n', ...
            basename,numel(meta),sum([meta.DelayTime])/100);
        close all;
    end
end
end
