function metrics = quarter_car_performance_eval(r)
% Whole-record RMS metrics; step transients measured from disturbance onset.
% NaN marks metrics not applicable to sine inputs; Inf means not settled.
t = r.t; T = t(end)-t(1);
rms_exact = sqrt(max(r.energies,0)/T);
metrics = struct('acceleration_rms_m_s2',rms_exact(1), ...
    'tire_deflection_rms_m',rms_exact(2), ...
    'max_suspension_deflection_m',max(abs(r.zs-r.zu)), ...
    'force_rms_kN',rms_exact(3)/1000, ...
    'force_peak_kN',max(abs(r.u))/1000, ...
    'body_overshoot_percent',NaN,'body_settling_time_s',NaN,'suspension_recovery_time_s',NaN);
if strcmpi(r.road,'step')
    target = r.parameters.step_amplitude;
    post = find(t >= r.parameters.step_time);
    y = r.zs(post);
    metrics.body_overshoot_percent = max(0,100*max(sign(target)*(y-target))/abs(target));
    outside = find(abs(y-target)>0.02*abs(target),1,'last');
    if isempty(outside)
        metrics.body_settling_time_s = 0;
    elseif outside == numel(post)
        metrics.body_settling_time_s = Inf;
    else
        metrics.body_settling_time_s = t(post(outside+1))-r.parameters.step_time;
    end
    outside = find(abs(r.zs(post)-r.zu(post))>0.02*abs(target),1,'last');
    if isempty(outside)
        metrics.suspension_recovery_time_s=0;
    elseif outside==numel(post)
        metrics.suspension_recovery_time_s=Inf;
    else
        metrics.suspension_recovery_time_s=t(post(outside+1))-r.parameters.step_time;
    end
end
end
