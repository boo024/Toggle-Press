%% TOGGLE PRESS — SYNTHESIS (BONUS)  run_bonus.m
% Given the requirements below, find the crank r2, coupler r3 and handle Lh.
%
% Requirements (edit R only):
%   - ram stroke >= R.strokeMin
%   - pressing force R.Fp over the last R.ws of ram travel
%   - hand force at the handle tip <= R.FhandMax
%   - crank-coupler angle gamma = th2 - th3 at the end of the stroke = R.gammaMin
%   - coupler (pressure) angle |th3| <= R.phiMax during the whole stroke
%
% Method: scan a grid of (r2, r3). For each pair:
%   1) th2End from gamma(th2End) = gammaMin,  stroke = x_B(th2End) - x_B(th2Start)
%   2) contact angle from x_B(th2c) = x_B(th2End) - ws
%   3) the largest torque is at contact (MA is lowest there):  T2 = Fp r2 sin(gamma)/cos(th3)
%   4) Lh_min = T2 / FhandMax, rounded up to R.LhStep
%   5) keep feasible pairs and pick the smallest r2 + r3 + Lh (most compact)
% Needs only base MATLAB (R2019b+ for tiledlayout, R2020a+ for exportgraphics).
% Units: mm / deg / N.

clear; clc; close all

%% 0. REQUIREMENTS AND SEARCH GRID
R.Fp        = 1000;      % [N]   pressing force
R.ws        = 2;         % [mm]  ram travel (end of stroke) with Fp applied
R.FhandMax  = 100;       % [N]   allowed hand force at the handle tip
R.strokeMin = 75;        % [mm]  required ram stroke
R.gammaMin  = 3;         % [deg] gamma = th2 - th3 at the end of the stroke
R.phiMax    = 15;        % [deg] allowed coupler angle |th3| during the stroke
R.th2Start  = 150;       % [deg] crank angle at the start of the stroke
R.e         = 0;         % [mm]  ram line offset
R.r2Grid    = 20:5:80;   % [mm]
R.r3Grid    = 60:5:300;  % [mm]
R.LhStep    = 5;         % [mm]  handle length is rounded up to this step
outDir      = fullfile('results', 'bonus');

%% 1. SCAN
n2 = numel(R.r2Grid);   n3 = numel(R.r3Grid);
stroke = nan(n2, n3);   phimax = nan(n2, n3);   T2max = nan(n2, n3);
Lh     = nan(n2, n3);   thEndG = nan(n2, n3);
for i = 1:n2
    for j = 1:n3
        r2 = R.r2Grid(i);   r3 = R.r3Grid(j);
        if r3 <= r2 + abs(R.e), continue; end                      % crank could not rotate fully
        thT = asin(R.e/(r2 + r3));                                 % extended toggle point
        th  = linspace(thT + 1e-6, deg2rad(R.th2Start), 4000);
        phi = asin((R.e - r2*sin(th))/r3);                         % coupler angle th3
        gam = rad2deg(th - phi);                                   % increases with th2
        x   = r2*cos(th) + r3*cos(phi);                            % ram position (decreases with th2)
        thEnd = interp1(gam, th, R.gammaMin);
        if isnan(thEnd), continue; end
        xEnd  = interp1(th, x, thEnd);
        inS   = th >= thEnd;
        stroke(i,j) = xEnd - x(end);
        phimax(i,j) = rad2deg(max(abs(phi(inS))));
        thc   = interp1(x(end:-1:1), th(end:-1:1), xEnd - R.ws);   % contact angle
        phic  = asin((R.e - r2*sin(thc))/r3);
        T2max(i,j) = R.Fp*(r2*1e-3)*sin(thc - phic)/cos(phic);     % [N m], static, massless
        Lh(i,j)    = ceil(T2max(i,j)/R.FhandMax*1e3/R.LhStep)*R.LhStep;
        thEndG(i,j) = rad2deg(thEnd);
    end
end
[R2, R3] = ndgrid(R.r2Grid, R.r3Grid);
feasible = stroke >= R.strokeMin & phimax <= R.phiMax & ~isnan(Lh);
cost     = R2 + R3 + Lh;   cost(~feasible) = Inf;
[cmin, idx] = min(cost(:));
if isinf(cmin), error('No feasible design in the grid. Relax the requirements or widen the grid.'); end
[ib, jb] = ind2sub(size(cost), idx);
r2b = R.r2Grid(ib);   r3b = R.r3Grid(jb);   Lhb = Lh(ib, jb);   thEndb = thEndG(ib, jb);

%% 2. VERIFY THE CHOSEN DESIGN OVER THE WHOLE WORKING STROKE
thT  = asin(R.e/(r2b + r3b));
th   = linspace(thT + 1e-6, deg2rad(R.th2Start), 20000);
phi  = asin((R.e - r2b*sin(th))/r3b);
x    = r2b*cos(th) + r3b*cos(phi);
thEn = deg2rad(thEndb);
xEn  = interp1(th, x, thEn);
inW  = x >= xEn - R.ws & th >= thEn;                               % working stroke (Fp applied)
T2w  = R.Fp*(r2b*1e-3)*sin(th(inW) - phi(inW))./cos(phi(inW));     % [N m]
Fh   = T2w/(Lhb*1e-3);                                             % [N]
MAend = cos(interp1(th, phi, thEn))/sind(R.gammaMin);              % mechanical advantage at the end
fprintf('\n================ SYNTHESIS RESULT ================\n');
fprintf('Requirements : stroke >= %g mm, Fp = %g N over %g mm, hand force <= %g N,\n', R.strokeMin, R.Fp, R.ws, R.FhandMax);
fprintf('               gamma_end = %g deg, |th3| <= %g deg\n', R.gammaMin, R.phiMax);
fprintf('Feasible     : %d of %d grid points\n', nnz(feasible), nnz(~isnan(Lh)));
fprintf('Chosen       : r2 = %g mm, r3 = %g mm, Lh = %g mm   (r2 + r3 + Lh = %g mm)\n', r2b, r3b, Lhb, cmin);
fprintf('Stroke       : %.2f mm  (th2 %.1f -> %.2f deg)\n', stroke(ib,jb), R.th2Start, thEndb);
fprintf('Coupler angle: max |th3| = %.2f deg\n', phimax(ib,jb));
fprintf('Torque       : max T2 = %.2f N m (at contact), hand force max = %.1f N\n', max(T2w), max(Fh));
fprintf('Force gain   : MA at the end of stroke = %.1f\n', MAend);
fprintf('Paste into toggle_press_symbolic.m:\n');
fprintf('   P.r2 = %g;  P.r3 = %g;  P.Lh = %g;  P.th2End = %.2f;\n', r2b, r3b, Lhb, thEndb);
fprintf('==================================================\n');

%% 3. PLOTS
C = struct('inp',[42 120 214]/255, 'outp',[235 104 52]/255, 'ink',[11 11 11]/255, 'ink2',[82 81 78]/255);
f = figure('Name', 'Synthesis', 'Color', 'w', 'Position', [60 60 1400 480]);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl);   hold(ax, 'on');
A = Lh;   A(~feasible) = NaN;
imagesc(ax, R.r3Grid, R.r2Grid, A, 'AlphaData', ~isnan(A));
set(ax, 'YDir', 'normal', 'Color', [0.93 0.93 0.91]);   colorbar(ax);
plot(ax, r3b, r2b, 'p', 'MarkerSize', 14, 'MarkerFaceColor', C.outp, 'MarkerEdgeColor', C.ink);
axis(ax, 'tight');   xlabel(ax, 'r_3 [mm]');   ylabel(ax, 'r_2 [mm]');
title(ax, 'Minimum handle L_h [mm] (feasible region only)');

ax = nexttile(tl);   hold(ax, 'on');
B = cost;   B(isinf(B)) = NaN;
imagesc(ax, R.r3Grid, R.r2Grid, B, 'AlphaData', ~isnan(B));
set(ax, 'YDir', 'normal', 'Color', [0.93 0.93 0.91]);   colorbar(ax);
plot(ax, r3b, r2b, 'p', 'MarkerSize', 14, 'MarkerFaceColor', C.outp, 'MarkerEdgeColor', C.ink);
axis(ax, 'tight');   xlabel(ax, 'r_3 [mm]');   ylabel(ax, 'r_2 [mm]');
title(ax, 'Size r_2 + r_3 + L_h [mm]  (star = chosen)');

ax = nexttile(tl);   hold(ax, 'on');
plot(ax, rad2deg(th(inW)), Fh, 'Color', C.inp, 'LineWidth', 2);
yline(ax, R.FhandMax, '--', 'Color', C.outp);
grid(ax, 'on');   set(ax, 'XDir', 'reverse');
xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'hand force [N]');
title(ax, sprintf('Chosen design: hand force in the working stroke (L_h = %g mm)', Lhb));
title(tl, sprintf('Toggle press synthesis: r_2 = %g mm, r_3 = %g mm, L_h = %g mm', r2b, r3b, Lhb));

if ~exist(outDir, 'dir'), mkdir(outDir); end
try, exportgraphics(f, fullfile(outDir, 'Render.jpg'), 'Resolution', 200); catch, end