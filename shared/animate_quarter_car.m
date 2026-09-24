function animate_quarter_car(t, zs, zu, zr, opts, uinfo)
% World-fixed road profile built directly from your input zr(t).
% Car moves forward at opts.carSpeed so wheel contact height equals road at car x.
%
% y_road(X) = yRoad0 + S*zr((X - x0_car)/v_car)  (static in world)

if nargin < 5, opts = struct(); end
if nargin < 6, uinfo = struct('t',t,'zr',zr,'label','Input'); end

% -------- defaults (kept exactly as you had) --------
opts = def(opts,'frameCallback',[]);
opts = def(opts,'gifPath',''); opts = def(opts,'screenshotPath','');
opts = def(opts,'figurePosition',[100 100 1200 850]);
opts = def(opts,'title','Quarter-car suspension');
opts = def(opts,'yRoad0',0.00);  opts = def(opts,'yUns0',0.00);  opts = def(opts,'ySpr0',1.15);
opts = def(opts,'xMass',0.00);
opts = def(opts,'xSusp',0.00);  opts = def(opts,'xTire',0.00);
opts = def(opts,'wSprung',0.80);  opts = def(opts,'hSprung',0.25);
opts = def(opts,'wUns',0.30);     opts = def(opts,'hUns',0.16);
opts = def(opts,'wheelRadius',0.20);
opts = def(opts,'springWidth',0.10);  opts = def(opts,'nCoils',10);
opts = def(opts,'nCoilsSusp',8); opts = def(opts,'nCoilsTire',4);
opts = def(opts,'dispScale',1);
opts = def(opts,'minGapSprUng',0.06);  opts = def(opts,'minGapWheelRoad',0.00);
opts = def(opts,'viewPadTop',0.25);    opts = def(opts,'viewPadBottom',0.12);
opts = def(opts,'frameSkip',8);        opts = def(opts,'pausePerFrame',1/240);
opts = def(opts,'carSpeed',0.5);       opts = def(opts,'x0_car',0.0);
opts = def(opts,'roadMarginL',0.0);    opts = def(opts,'roadMarginR',0.0);
opts = def(opts,'xSpringLine',0.00);   opts.xSusp = opts.xSpringLine; opts.xTire = opts.xSpringLine;

% aliases
S = opts.dispScale;  Rwh = opts.wheelRadius;

% -------------- prep --------------
t  = t(:); zs = zs(:); zu = zu(:); zr = zr(:);
v  = max(1e-6, opts.carSpeed);
x_car = opts.x0_car + v*t;
carHalfWidth = max([opts.wSprung/2, opts.wUns/2, Rwh]);
x_min = min(x_car) + min(0,opts.xMass) - opts.roadMarginL - carHalfWidth - 0.1;
x_max = max(x_car) + max(0,opts.xMass) + opts.roadMarginR + carHalfWidth + 0.1;

% Spatial world road from time signal
zr_of_tau = @(tau) interp1(t, zr, tau, 'linear', 'extrap');
yroad     = @(X) opts.yRoad0 + S * zr_of_tau((X - opts.x0_car)/v);  % <-- use for any world-X
roadX     = linspace(x_min, x_max, 800);
roadY     = yroad(roadX);

% Contact road height under the actual wheel world-x (includes xMass offset)
yR_contact = yroad(x_car + opts.xMass);   % <-- FIX: sync to wheel x

% Fixed schematic offsets; apply wheel clearance before body clearance.
% These offsets do not clip or otherwise alter simulated displacement changes.
yU = opts.yUns0 + S*zu;
yS = opts.ySpr0 + S*zs;
clear_WR = yU - Rwh - yR_contact;
shiftU = max(0, opts.minGapWheelRoad - min(clear_WR));
yU = yU + shiftU;
clear_SU = (yS - opts.hSprung/2) - (yU + opts.hUns/2);
shiftS = max(0, opts.minGapSprUng - min(clear_SU));
yS = yS + shiftS;

% View limits
ymin_mech = min([yS - opts.hSprung/2; yU - Rwh; yR_contact]) - opts.viewPadBottom;
ymax_mech = max([yS + opts.hSprung/2; yU + Rwh; yR_contact]) + opts.viewPadTop;

% Use exact physical velocities when supplied; estimate only as a fallback
if isfield(uinfo,'zs_dot') && isfield(uinfo,'zu_dot')
    zsd = uinfo.zs_dot(:); zud = uinfo.zu_dot(:);
else
    zsd = gradient(zs, t); zud = gradient(zu, t);
end

% Control availability
hasU = isfield(uinfo,'u') && ~isempty(uinfo.u);
if hasU, uinfo.u = uinfo.u(:); end

% ---------------- Figure layout: (4x2)
fh = figure('Color','w','Name','Quarter-car: World-fixed Road, Moving Car','NumberTitle','off', ...
    'Position',opts.figurePosition);
tl = tiledlayout(fh, 4, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl,opts.title,'FontWeight','bold','Color',[.1 .1 .1]);

% Row 1: mechanism spans both columns
axM = nexttile(tl, [1 2]); hold(axM,'on'); axis(axM,'equal'); box(axM,'on'); grid(axM,'on');
xlabel(axM,'x (m)', 'Interpreter','tex');
ylabel(axM,'z (m)', 'Interpreter','tex');
xlim(axM,[x_min, x_max]); ylim(axM,[ymin_mech, ymax_mech]);

% ---- Static world road
plot(axM, roadX, roadY, 'k-', 'LineWidth', 2);

% Initial poses use the same fixed offsets as every animation frame
xC0 = x_car(1);  yS0 = yS(1);  yU0 = yU(1);
xC0_world = xC0 + opts.xMass;
yR0 = yR_contact(1);
yU0_draw = yU0;

% Red contact dot under the wheel center
hRoadDot = plot(axM, xC0_world, yR0, 'ro', 'MarkerFaceColor','r', 'MarkerSize', 5);

% Draw car at initial pose
hSprung = draw_block(axM, xC0 + opts.xMass, yS0, opts.wSprung, opts.hSprung, [0.20 0.55 0.95]);
hUns    = draw_block(axM, xC0 + opts.xMass, yU0_draw, opts.wUns, opts.hUns, [0.90 0.70 0.30]);

% Wheel (rim, spokes, rim marker) — all use yU*_draw
theta = linspace(0,2*pi,180);
hWheel = plot(axM, xC0 + opts.xMass + Rwh*cos(theta),  yU0_draw + Rwh*sin(theta), 'k-', 'LineWidth', 2);
hSp1   = plot(axM, [xC0+opts.xMass xC0+opts.xMass], [yU0_draw-Rwh yU0_draw+Rwh], 'k-');
hSp2   = plot(axM, [xC0+opts.xMass-Rwh xC0+opts.xMass+Rwh], [yU0_draw yU0_draw], 'k-');
markerAngle0 = pi/2;
hMark = plot(axM, xC0 + opts.xMass + Rwh*cos(markerAngle0), ...
                  yU0_draw + Rwh*sin(markerAngle0), 'ko', 'MarkerFaceColor','k', 'MarkerSize',4);

% Suspension spring/damper — SAME x line
yTopSusp = yS0 - opts.hSprung/2;   yBotSusp = yU0_draw + opts.hUns/2;
hSuspSpring = draw_spring(axM, xC0 + opts.xSpringLine, yTopSusp, yBotSusp, ...
                          opts.springWidth, opts.nCoilsSusp, [0.15 0.15 0.15], 1.8);
[hDamTop, hDamBot, hDamRod] = draw_damper(axM, xC0 + opts.xSpringLine, yTopSusp, yBotSusp);

% Tire spring — SAME x line (top at unsprung center, bottom on road)
hTireSpring = draw_spring(axM, xC0 + opts.xSpringLine, yU0_draw, yR0, ...
                          0.9*opts.springWidth, opts.nCoilsTire, [0.05 0.05 0.05], 1.8);

% HUD
hTxt = text(axM, x_min + 0.02, ymax_mech - 0.05, 't = 0.00 s', 'FontSize',10, 'VerticalAlignment','top');

% ---------- Rows 2–3: states (make dynamic lines; same colors) ----------
axS1 = nexttile(tl); hold(axS1,'on'); box(axS1,'on'); grid(axS1,'on');
% ylim from full data (pad if flat)
m1 = min(zs); M1 = max(zs); if m1==M1, d=max(1e-6,abs(M1)); m1=m1-0.05*d; M1=M1+0.05*d; end
ylim(axS1,[m1 M1]); xlim(axS1,[t(1) t(end)]);
hProgS1 = plot(axS1, NaN, NaN, 'LineWidth',1.4, 'Color',[0.0 0.45 0.74]);
xlabel(axS1,'t (s)', 'Interpreter','tex');  ylabel(axS1,'z_s (m)', 'Interpreter','tex');
yl1 = ylim(axS1); hCur1 = plot(axS1, [t(1) t(1)], yl1, 'r--', 'LineWidth',1.0);

axS2 = nexttile(tl); hold(axS2,'on'); box(axS2,'on'); grid(axS2,'on');
m2 = min(zsd); M2 = max(zsd); if m2==M2, d=max(1e-6,abs(M2)); m2=m2-0.05*d; M2=M2+0.05*d; end
ylim(axS2,[m2 M2]); xlim(axS2,[t(1) t(end)]);
hProgS2 = plot(axS2, NaN, NaN, 'LineWidth',1.4, 'Color',[0.30 0.30 0.30]);
xlabel(axS2,'t (s)', 'Interpreter','tex');  ylabel(axS2,'dz_s/dt (m/s)', 'Interpreter','tex');
yl2 = ylim(axS2); hCur2 = plot(axS2, [t(1) t(1)], yl2, 'r--', 'LineWidth',1.0);

axS3 = nexttile(tl); hold(axS3,'on'); box(axS3,'on'); grid(axS3,'on');
m3 = min(zu); M3 = max(zu); if m3==M3, d=max(1e-6,abs(M3)); m3=m3-0.05*d; M3=M3+0.05*d; end
ylim(axS3,[m3 M3]); xlim(axS3,[t(1) t(end)]);
hProgS3 = plot(axS3, NaN, NaN, 'LineWidth',1.4, 'Color',[0.85 0.33 0.10]);
xlabel(axS3,'t (s)', 'Interpreter','tex');  ylabel(axS3,'z_u (m)', 'Interpreter','tex');
yl3 = ylim(axS3); hCur3 = plot(axS3, [t(1) t(1)], yl3, 'r--', 'LineWidth',1.0);

axS4 = nexttile(tl); hold(axS4,'on'); box(axS4,'on'); grid(axS4,'on');
m4 = min(zud); M4 = max(zud); if m4==M4, d=max(1e-6,abs(M4)); m4=m4-0.05*d; M4=M4+0.05*d; end
ylim(axS4,[m4 M4]); xlim(axS4,[t(1) t(end)]);
hProgS4 = plot(axS4, NaN, NaN, 'LineWidth',1.4, 'Color',[0.20 0.20 0.60]);
xlabel(axS4,'t (s)', 'Interpreter','tex');  ylabel(axS4,'dz_u/dt (m/s)', 'Interpreter','tex');
yl4 = ylim(axS4); hCur4 = plot(axS4, [t(1) t(1)], yl4, 'r--', 'LineWidth',1.0);

% Row 4: control spans both columns (dynamic up to cursor)
axC = nexttile(tl, [1 2]); hold(axC,'on'); box(axC,'on'); grid(axC,'on');
if hasU
    mC = min(uinfo.u); MC = max(uinfo.u); if mC==MC, d=max(1e-6,abs(MC)); mC=mC-0.05*d; MC=MC+0.05*d; end
    ylim(axC,[mC MC]);
    hProgC = plot(axC, NaN, NaN, 'LineWidth',1.4, 'Color',[0.2 0.2 0.2]);
else
    text(axC, 0.5, 0.5, 'u(t) not provided', 'Units','normalized', ...
         'HorizontalAlignment','center', 'FontAngle','italic');
end
xlabel(axC,'t (s)', 'Interpreter','tex');
ylabel(axC,'u (N)', 'Interpreter','tex');
xlim(axC,[t(1) t(end)]);
ylc = ylim(axC); hCurC = plot(axC, [t(1) t(1)], ylc, 'r--', 'LineWidth',1.0);

% Native tick labels render reliably in interactive and batch figures
set([axM, axS1, axS2, axS3, axS4, axC], 'TickLabelInterpreter','tex');

style_quarter_car_figure(fh);

% rolling reference for rim marker (same as your logic)
xTravel0 = xC0;

% ---------------- Animate ----------------
N = numel(t);
frame_indices = unique([1:opts.frameSkip:N, N]);
for frame_index = 1:numel(frame_indices)
    k = frame_indices(frame_index);
    tk = t(k);
    xC = x_car(k);
    ySk = yS(k);
    % road under the wheel at this frame (already precomputed vector)
    yRk = yR_contact(k);

    % ---- physical wheel displacement with a fixed drawing offset
    yUk = yU(k);
    yU_draw = yUk; % preserve the simulated wheel displacement

    % Update blocks
    update_block(hSprung, xC + opts.xMass, ySk, opts.wSprung, opts.hSprung);
    update_block(hUns,    xC + opts.xMass, yU_draw, opts.wUns, opts.hUns);

    % Rolling wheel (drawn around yU_draw)
    phi = - (xC - xTravel0) / Rwh;  c = cos(phi); s = sin(phi);
    set(hWheel, 'XData', xC + opts.xMass + Rwh*cos(theta), ...
                'YData', yU_draw + Rwh*sin(theta));

    sp1x = [0 0]; sp1y = [-Rwh Rwh];
    sp1xr =  c*sp1x - s*sp1y;   sp1yr = s*sp1x + c*sp1y;
    set(hSp1, 'XData', xC + opts.xMass + sp1xr, 'YData', yU_draw + sp1yr);

    sp2x = [-Rwh Rwh]; sp2y = [0 0];
    sp2xr =  c*sp2x - s*sp2y;   sp2yr = s*sp2x + c*sp2y;
    set(hSp2, 'XData', xC + opts.xMass + sp2xr, 'YData', yU_draw + sp2yr);

    thm = markerAngle0 + phi;
    set(hMark, 'XData', xC + opts.xMass + Rwh*cos(thm), ...
               'YData', yU_draw + Rwh*sin(thm));

    % Springs & damper (tie to yU_draw)
    yTopSusp = ySk - opts.hSprung/2;
    yBotSusp = yU_draw + opts.hUns/2;
    update_spring(hSuspSpring, xC + opts.xSpringLine, yTopSusp, yBotSusp);
    update_damper(hDamTop, hDamBot, hDamRod,  xC + opts.xSpringLine, yTopSusp, yBotSusp);

    % Tire spring from unsprung center (top) to road (bottom)
    update_spring(hTireSpring, xC + opts.xSpringLine, yU_draw, yRk);

    % Road contact dot (time-locked with wheel)
    xWheel_world = xC + opts.xMass;
    set(hRoadDot, 'XData', xWheel_world, 'YData', yRk);

    % Time HUD
    set(hTxt, 'String', sprintf('t = %.2f s  |  x = %.2f m', tk, xC));

    % ----- Dynamic traces up to red cursor -----
    set(hProgS1, 'XData', t(1:k), 'YData', zs(1:k));
    set(hProgS2, 'XData', t(1:k), 'YData', zsd(1:k));
    set(hProgS3, 'XData', t(1:k), 'YData', zu(1:k));
    set(hProgS4, 'XData', t(1:k), 'YData', zud(1:k));
    if hasU
        set(hProgC, 'XData', t(1:k), 'YData', uinfo.u(1:k));
    end

    % Move state/control cursors
    set(hCur1,'XData',[tk tk],'YData',ylim(axS1));
    set(hCur2,'XData',[tk tk],'YData',ylim(axS2));
    set(hCur3,'XData',[tk tk],'YData',ylim(axS3));
    set(hCur4,'XData',[tk tk],'YData',ylim(axS4));
    set(hCurC,'XData',[tk tk],'YData',ylim(axC));

    if ~isempty(opts.frameCallback)
        opts.frameCallback(fh,k);
    end
    if ~isempty(opts.gifPath)
        drawnow;
        frame = getframe(fh);
        [indexed,map] = rgb2ind(frame.cdata,256);
        if frame_index < numel(frame_indices)
            delay = t(frame_indices(frame_index+1))-t(k);
        else
            delay = 1; % hold the completed traces before looping
        end
        if frame_index == 1
            imwrite(indexed,map,opts.gifPath,'gif','LoopCount',Inf,'DelayTime',delay);
        else
            imwrite(indexed,map,opts.gifPath,'gif','WriteMode','append','DelayTime',delay);
        end
    else
        drawnow limitrate;
    end
    pause(opts.pausePerFrame);
end
if ~isempty(opts.screenshotPath)
    drawnow;
    exportgraphics(fh,opts.screenshotPath,'Resolution',120);
end
end

% ===================== helpers =====================
function s = def(s, name, val)
if ~isfield(s,name) || isempty(s.(name)), s.(name) = val; end
end

function h = draw_block(ax, xc, yc, w, hgt, faceColor)
x = xc + [-w/2,  w/2,  w/2, -w/2];
y = yc + [-hgt/2, -hgt/2,  hgt/2,  hgt/2];
h = patch(ax, x, y, faceColor, 'EdgeColor','k', 'LineWidth',1.5);
end

function update_block(h, xc, yc, w, hgt)
x = xc + [-w/2,  w/2,  w/2, -w/2];
y = yc + [-hgt/2, -hgt/2,  hgt/2,  hgt/2];
set(h, 'XData', x, 'YData', y);
end

function h = draw_spring(ax, x0, yTop, yBot, width, nCoils, color, lw)
nCoils = max(1, round(nCoils));
yy = linspace(yTop, yBot, 2*nCoils+1);
xx = x0 + width * (-1).^(0:numel(yy)-1);
h  = plot(ax, xx, yy, '-', 'Color', color, 'LineWidth', lw);
end

function update_spring(h, x0, yTop, yBot)
xx_prev = get(h,'XData'); n = numel(xx_prev);
yy = linspace(yTop, yBot, n);
width = 0.5*(max(xx_prev) - min(xx_prev));
xx = x0 + width * (-1).^(0:n-1);
set(h,'XData',xx,'YData',yy);
end

function [hTop, hBot, hRod] = draw_damper(ax, x0, yTop, yBot)
w = 0.06; h = 0.08;
hTop = patch(ax, x0 + [-w w w -w], [yTop yTop yTop+h yTop+h], [0.7 0.7 0.7], 'EdgeColor','k');
hBot = patch(ax, x0 + [-w w w -w], [yBot-h yBot-h yBot yBot], [0.5 0.5 0.5], 'EdgeColor','k');
hRod = plot(ax, [x0 x0], [yTop yBot], 'k-', 'LineWidth', 2);
end

function update_damper(hTop, hBot, hRod, x0, yTop, yBot)
w = 0.06; h = 0.08;
set(hTop,'XData', x0 + [-w w w -w], 'YData', [yTop yTop yTop+h yTop+h]);
set(hBot,'XData', x0 + [-w w w -w], 'YData', [yBot-h yBot-h yBot yBot]);
set(hRod,'XData', [x0 x0], 'YData', [yTop yBot]);
end
