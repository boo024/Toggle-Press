%% TOGGLE PRESS — SYMBOLIC KINEMATIC AND FORCE ANALYSIS + ANIMATION  (2103-322)
% Mechanism : offset slider-crank used as a toggle (knee) press.
%   Link 1 = frame, Link 2 = crank O2-A (input), Link 3 = coupler / toggle link A-B,
%   Link 4 = ram (slider) moving along +x on the line y = e.
%   Joints : R(O2), R(A), R(B), P(ram-frame)  ->  DoF = 3(4-1) - 2(4) = 1
%
% Toggle (dead-centre) condition: crank and coupler collinear, theta2 = theta3.
%   The ram speed -> 0 while the force amplification -> infinity.
%
% Flow : symbolic derivation -> matlabFunction -> numeric frames -> checks -> plots -> animation.
% Needs: Symbolic Math Toolbox (R2019b+ for tiledlayout).
% Units: mm / deg / rpm in the parameter block; SI (m, kg, N) internally.
% Assumption: the assembly branch is cos(theta3) > 0 (ram stays to the right of pin A).

clear; clc; close all

%% 0. PARAMETERS — ALL VALUES ARE PLACEHOLDERS (edit only this block)
% ---- geometry ----
P.r2        = 40;        % [mm]  crank O2-A
P.r3        = 160;       % [mm]  coupler A-B
P.e         = 0;         % [mm]  ram line offset from O2 (0 = in-line)
% ---- mass properties (for the dynamic force analysis) ----
P.rg2       = [];        % [mm]  crank CG distance from O2     ([] -> r2/2)
P.rg3       = [];        % [mm]  coupler CG distance from A    ([] -> r3/2)
P.m2        = 0.4;       % [kg]  crank
P.m3        = 0.8;       % [kg]  coupler
P.m4        = 3.0;       % [kg]  ram
P.I2        = [];        % [kg m^2] about CG ([] -> uniform bar m L^2/12)
P.I3        = [];        % [kg m^2] about CG ([] -> uniform bar m L^2/12)
P.vertical  = true;      % true: press stands vertically, ram pushes DOWN (gravity along the ram); false: horizontal
% ---- motion ----
P.th2Start  = 150;       % [deg] crank angle at start of the stroke
P.th2End    = [];        % [deg] crank angle at end ([] -> toggle point +/- margin)
P.margin    = 3;         % [deg] distance from the toggle point at the end of the stroke
P.rpm       = 30;        % [rpm] constant crank speed (direction follows th2Start -> th2End)
% ---- process load on the ram ----
P.Fp        = 1000;      % [N]   pressing force, acts against the ram motion (+x)
P.workStroke= 2;         % [mm]  length of ram travel (end of stroke) with Fp applied
% ---- drawing / output ----
P.Lh        = 150;       % [mm]  handle length (shorten / lengthen the handle HERE); hand force = |T2|/Lh
P.handleAng = 90;        % [deg] handle direction measured from the crank O2->A (90 = L-shaped bell crank; 180 = straight extension)
P.wS        = 50;        % [mm]  ram block length
P.hS        = 30;        % [mm]  ram block height
P.N         = 301;       % frames per stroke
P.animMode  = "loop";    % "loop" | "none"
P.gifFile   = "";        % e.g. "toggle_press.gif" to export the animation
P.aviFile    = "Mechanism.mp4";    % video of the animation window; "" = off
P.renderFile = "Render.jpg";  % still image of the animation window (mechanism + graphs); "" = off

C = struct('inp',[42 120 214]/255, 'outp',[235 104 52]/255, 'coup',[27 175 122]/255, ...
           'ground',[58 58 56]/255, 'ink',[11 11 11]/255, 'ink2',[82 81 78]/255, ...
           'band',[228 227 223]/255);

%% 1. DERIVED PARAMETERS AND CHECKS ON THE LINK LENGTHS
if isempty(P.rg2), P.rg2 = P.r2/2; end
if isempty(P.rg3), P.rg3 = P.r3/2; end
if isempty(P.I2),  P.I2  = P.m2*(P.r2*1e-3)^2/12; end
if isempty(P.I3),  P.I3  = P.m3*(P.r3*1e-3)^2/12; end
P.viewRot = pick(P.vertical, -90, 0);                  % model x axis (ram travel) -> world -y (down) when vertical
Rw  = [cosd(P.viewRot) -sind(P.viewRot); sind(P.viewRot) cosd(P.viewRot)];   % model -> world (drawing only)
P.gvec = (Rw.'*[0; -9.81]).';                          % gravity expressed in the model frame
th2T = asind(P.e/(P.r2 + P.r3));                       % extended toggle point (closed form)
if isempty(P.th2End), P.th2End = th2T + sign(P.th2Start - th2T)*P.margin; end
dirn = sign(P.th2End - P.th2Start);
if dirn == 0, error('th2End must differ from th2Start.'); end
w2num = dirn*abs(P.rpm)*2*pi/60;                       % [rad/s]
al2num = 0;                                            % constant crank speed
fprintf('Links  : r2 = %.1f, r3 = %.1f, e = %.1f mm  (r3 %s r2 + |e|: %s)\n', P.r2, P.r3, P.e, ...
        pick(P.r3 > P.r2 + abs(P.e), '>', '<='), pick(P.r3 > P.r2 + abs(P.e), 'crank can rotate fully', 'crank cannot rotate fully'));
fprintf('Toggle : extended dead centre at theta2 = %.2f deg\n', th2T);

%% 2. SYMBOLIC POSITION (loop closure:  r2 sin(th2) + r3 sin(th3) = e)
syms th2 w2 al2 real                    % input angle, speed and acceleration
syms r2 r3 e real
syms rg2 rg3 m2 m3 m4 I2 I3 gx gy Fp real
pSym = [r2 r3 e rg2 rg3 m2 m3 m4 I2 I3 gx gy Fp];

th3 = asin((e - r2*sin(th2))/r3);       % branch cos(th3) > 0
A   = [r2*cos(th2); r2*sin(th2)];
B   = [r2*cos(th2) + r3*cos(th3); e];
G2  = rg2*[cos(th2); sin(th2)];                       % crank CG
G3  = A + rg3*[cos(th3); sin(th3)];                   % coupler CG

%% 3. SYMBOLIC VELOCITY AND ACCELERATION (chain rule with th2 -> w2 -> al2)
vel = @(p) jacobian(p, th2)*w2;
acc = @(p) jacobian(vel(p), th2)*w2 + jacobian(vel(p), w2)*al2;

w3  = vel(th3);   al3 = acc(th3);                     % coupler angular velocity / acceleration
vB  = vel(B(1));  aB  = acc(B(1));                    % ram velocity / acceleration
aG2 = acc(G2);    aG3 = acc(G3);                      % CG accelerations (for Newton-Euler)

% hand-simplified closed forms (used as an independent check)
vB_cf = -r2*w2*sin(th2 - th3)/cos(th3);               % -> 0 when th2 = th3 (toggle)
w3_cf = -r2*w2*cos(th2)/(r3*cos(th3));
fprintf('\nSymbolic results (closed forms):\n');
fprintf('  theta3 = ');  pretty(th3);
fprintf('  vB     = ');  pretty(vB_cf);
fprintf('  w3     = ');  pretty(w3_cf);

%% 4. SYMBOLIC FORCE ANALYSIS (Newton-Euler, planar, 8 equations / 8 unknowns)
%   Fo = ground on crank at O2      Fa = crank on coupler at A
%   Fb = ram on coupler at B        N  = ground normal on ram     T2 = input torque
syms Fox Foy Fax Fay Fbx Fby N T2 real
Fo = [Fox; Foy];   Fa = [Fax; Fay];   Fb = [Fbx; Fby];
gv = [gx; gy];
c2 = @(r, F) r(1)*F(2) - r(2)*F(1);                   % planar cross product (z-component)
O  = [0; 0];
eq = [ Fo - Fa + m2*gv - m2*aG2;                                         % crank, force
       c2(O - G2, Fo) + c2(A - G2, -Fa) + T2 - I2*al2;                   % crank, moment about CG
       Fa + Fb + m3*gv - m3*aG3;                                         % coupler, force
       c2(A - G3, Fa) + c2(B - G3, Fb) - I3*al3;                         % coupler, moment about CG
       -Fb + [0; N] + [-Fp; 0] + m4*gv - m4*[aB; 0] ];                   % ram, force
X  = [Fox Foy Fax Fay Fbx Fby N T2];
[Msym, bsym] = equationsToMatrix(eq, X);              % linear system  M*X = b

% principle of virtual work (static, massless): T2*w2 = Fp*vB
T2_vw = Fp*vB_cf/w2;                                  % = -Fp*r2*sin(th2-th3)/cos(th3)
MA    = r2*abs(w2)/abs(vB_cf);                        % kinematic force amplification
fprintf('  T2 (virtual work) = ');   pretty(simplify(T2_vw));

%% 5. MATLAB FUNCTIONS FROM THE SYMBOLIC RESULTS
fKin = matlabFunction([B(1); vB; aB; th3; w3; al3], 'Vars', {th2, w2, al2, pSym});
fVcf = matlabFunction(vB_cf, 'Vars', {th2, w2, pSym});
fMb  = matlabFunction(Msym, bsym, 'Vars', {th2, w2, al2, pSym});

pv = [P.r2 P.r3 P.e P.rg2 P.rg3]*1e-3;                % geometry in m
pv = [pv P.m2 P.m3 P.m4 P.I2 P.I3 P.gvec 0];          % Fp is set frame by frame (index 13)

%% 6. POSITION / VELOCITY / ACCELERATION OVER THE STROKE
N_   = P.N;
th2v = linspace(deg2rad(P.th2Start), deg2rad(P.th2End), N_);
sArg = (pv(3) - pv(1)*sin(th2v))/pv(2);
if any(abs(sArg) > 1)
    error('Cannot assemble somewhere in the stroke: |(e - r2 sin(th2))/r3| > 1.');
end
K = zeros(6, N_);
for i = 1:N_
    K(:,i) = real(fKin(th2v(i), w2num, al2num, pv));  % [xB; vB; aB; th3; w3; al3]
end
xB = K(1,:);   vBn = K(2,:);   aBn = K(3,:);   th3v = K(4,:);   w3n = K(5,:);   al3n = K(6,:);
Av = pv(1)*[cos(th2v); sin(th2v)];
Bv = [xB; pv(3)*ones(1, N_)];
gam = th2v - th3v;                                    % crank-coupler angle (0 at toggle)
tv  = (th2v - th2v(1))/w2num;                         % time [s]
if any(sign(gam) ~= sign(gam(1)))
    error('The stroke crosses the toggle point (theta2 = theta3). Move th2End.');
end
xContact = xB(end) - P.workStroke*1e-3;               % ram position where Fp starts

%% 7. FORCE ANALYSIS OVER THE STROKE
Xd = zeros(8, N_);   Xs = zeros(8, N_);   Fpv = zeros(1, N_);
for i = 1:N_
    pvi = pv;   Fpv(i) = P.Fp*(xB(i) >= xContact);   pvi(13) = Fpv(i);
    [M, b] = fMb(th2v(i), w2num, al2num, pvi);
    Xd(:,i) = M\b;                                    % dynamic (masses + gravity + process load)
    pvs = pvi;   pvs(6:10) = 0;                       % static, massless
    [M, b] = fMb(th2v(i), w2num, al2num, pvs);
    Xs(:,i) = M\b;
end
T2dyn = Xd(8,:);   T2st = Xs(8,:);   T2vw = Fpv.*vBn/w2num;
Fo_n = vecnorm(Xd(1:2,:));   Fa_n = vecnorm(Xd(3:4,:));   Fb_n = vecnorm(Xd(5:6,:));
MAn  = pv(1)*abs(w2num)./abs(vBn);

%% 8. VALIDATION
errL3 = max(abs(vecnorm(Bv - Av) - pv(2)));                       % |AB| = r3
dt    = tv(2) - tv(1);
errV  = max(abs(gradient(xB, dt) - vBn));   errV = errV/max(abs(vBn));   % finite difference vs symbolic v
errA  = max(abs(gradient(vBn, dt) - aBn)); errA = errA/max(abs(aBn));
errCF = max(abs(arrayfun(@(i) fVcf(th2v(i), w2num, pv), 1:N_) - vBn));
errVW = max(abs(T2st - T2vw));
fprintf('\n================ SUMMARY ================\n');
fprintf('Stroke   : theta2 %.1f -> %.1f deg (%s), ram travel %.2f mm, t = %.3f s\n', ...
        P.th2Start, P.th2End, pick(dirn < 0, 'CW', 'CCW'), 1e3*(xB(end) - xB(1)), tv(end));
fprintf('Closure  : max | |AB| - r3 | = %.1e m\n', errL3);
fprintf('Kinematic: v  (finite diff vs symbolic) rel. err = %.1e (interior points ~0, ends are 1st order)\n', errV);
fprintf('           a  (finite diff vs symbolic) rel. err = %.1e\n', errA);
fprintf('           v  (direct vs closed form)   abs. err = %.1e m/s\n', errCF);
fprintf('Force    : static Newton-Euler vs virtual work, max |dT2| = %.1e N m\n', errVW);
fprintf('Result   : max |T2| = %.2f N m (dynamic), %.2f N m (static)\n', max(abs(T2dyn)), max(abs(T2st)));
fprintf('           mechanical advantage at the end of stroke = %.1f\n', MAn(end));
fprintf('           hand force at the handle tip (Lh = %.0f mm): max %.1f N (dynamic), %.1f N at the end\n', ...
        P.Lh, max(abs(T2dyn))/(P.Lh*1e-3), abs(T2dyn(end))/(P.Lh*1e-3));
fprintf('           max pin force |Fa| = %.0f N, |Fo| = %.0f N, max ram normal N = %.0f N\n', ...
        max(Fa_n), max(Fo_n), max(abs(Xd(7,:))));
fprintf('=========================================\n');

%% 9. STATIC POSES
iShow = [1 ceil(N_/2) N_];   alph = [0.3 0.55 1];
names = ["Start", "Mid-stroke", "End (near toggle)"];
f1  = figure('Name', 'Poses', 'Color', 'w', 'Position', [40 80 1500 420]);
tl1 = tiledlayout(f1, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
Rh   = [cosd(P.handleAng) -sind(P.handleAng); sind(P.handleAng) cosd(P.handleAng)];
Hv   = P.Lh*1e-3*Rh*[cos(th2v); sin(th2v)];          % handle tip path (rigid with the crank, at handleAng)
Wd   = @(X) 1e3*(Rw*X);                              % model [m] -> world [mm] for drawing
allP = [Wd(Hv) Wd(Av) Wd(Bv) Wd([xB(end) + 0.06; P.e*1e-3]) Wd([xB(1) - 0.04; P.e*1e-3])];
lims = [min(allP(1,:)) - 40, max(allP(1,:)) + 40, min(allP(2,:)) - 40, max(allP(2,:)) + 40];
dieC = Rw*[xContact*1e3 + P.wS/2 + [0 25 25 0]; P.e + 40*[-1 -1 1 1]];     % die / workpiece corners (world, mm)
gdR  = Rw*[-10 1e3*xB(end) + 60; (P.e - P.hS/2 - 3)*[1 1]];                % guide rails (world, mm)
gdL  = Rw*[-10 1e3*xB(end) + 60; (P.e + P.hS/2 + 3)*[1 1]];
for k = 1:3
    ax = nexttile(tl1);   hold(ax, 'on');
    i = iShow(k);
    drawPose(ax, 1e3*Av(:,i), 1e3*Bv(:,i), P.e, P.wS, P.hS, P.Lh, P.handleAng, Rw, C, 1);
    hp = Wd(Hv);   plot(ax, hp(1,:), hp(2,:), ':', 'Color', C.inp);
    ap = Wd(Av);   plot(ax, ap(1,:), ap(2,:), ':', 'Color', C.inp);
    bp = Wd([xB; P.e*1e-3*ones(1, N_)]);   plot(ax, bp(1,:), bp(2,:), ':', 'Color', C.outp);
    patch(ax, dieC(1,:), dieC(2,:), C.band, 'EdgeColor', C.ink2);          % die / workpiece
    axis(ax, 'equal');   grid(ax, 'on');   xlim(ax, lims(1:2));   ylim(ax, lims(3:4));
    xlabel(ax, 'x [mm]');   ylabel(ax, 'y [mm]');
    title(ax, sprintf('%s:  \\theta_2 = %.1f^{\\circ},  \\gamma = %.1f^{\\circ}', ...
          names(k), rad2deg(th2v(i)), rad2deg(gam(i))));
end
title(tl1, 'Toggle press (PLACEHOLDER dimensions)');

%% 10. KINEMATIC PLOTS
th2d = rad2deg(th2v);   xDir = pick(dirn < 0, 'reverse', 'normal');
f2  = figure('Name', 'Kinematics', 'Color', 'w', 'Position', [60 60 1100 700]);
tl2 = tiledlayout(f2, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl2);   plot(ax, th2d, 1e3*xB, 'Color', C.outp, 'LineWidth', 2);
yline(ax, 1e3*xContact, '--', 'Color', C.ink2);
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'x_B [mm]');   title(ax, 'Ram position');   set(ax, 'XDir', xDir);
ax = nexttile(tl2);   plot(ax, th2d, 1e3*vBn, 'Color', C.outp, 'LineWidth', 2);
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'v_B [mm/s]');   title(ax, 'Ram velocity (\rightarrow 0 at toggle)');   set(ax, 'XDir', xDir);
ax = nexttile(tl2);   plot(ax, th2d, aBn, 'Color', C.outp, 'LineWidth', 2);
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'a_B [m/s^2]');   title(ax, 'Ram acceleration');   set(ax, 'XDir', xDir);
ax = nexttile(tl2);   hold(ax, 'on');
yyaxis(ax, 'left');    plot(ax, th2d, rad2deg(th3v), 'Color', C.coup, 'LineWidth', 2);   ylabel(ax, '\theta_3 [deg]');
yyaxis(ax, 'right');   plot(ax, th2d, w3n, 'Color', C.inp, 'LineWidth', 1.5);            ylabel(ax, '\omega_3 [rad/s]');
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   title(ax, 'Coupler angle and angular velocity');   set(ax, 'XDir', xDir);

%% 11. FORCE PLOTS
f3  = figure('Name', 'Forces', 'Color', 'w', 'Position', [80 60 1100 700]);
tl3 = tiledlayout(f3, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl3);   hold(ax, 'on');
plot(ax, th2d, T2dyn, 'Color', C.outp, 'LineWidth', 2);
plot(ax, th2d, T2st, 'Color', C.coup, 'LineWidth', 1.5);
plot(ax, th2d, T2vw, '--', 'Color', C.ink, 'LineWidth', 1);
legend(ax, 'dynamic', 'static (Newton-Euler)', 'virtual work', 'Location', 'best');
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'T_2 [N m]');   title(ax, 'Input torque');   set(ax, 'XDir', xDir);
ax = nexttile(tl3);   semilogy(ax, th2d, MAn, 'Color', C.inp, 'LineWidth', 2);
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'MA = r_2|\omega_2| / |v_B|');   title(ax, 'Mechanical advantage');   set(ax, 'XDir', xDir);
ax = nexttile(tl3);   hold(ax, 'on');
plot(ax, th2d, Fo_n, 'Color', C.ground, 'LineWidth', 1.8);
plot(ax, th2d, Fa_n, 'Color', C.coup, 'LineWidth', 1.8);
plot(ax, th2d, Fb_n, '--', 'Color', C.outp, 'LineWidth', 1.2);
legend(ax, '|F_{O2}|', '|F_A|', '|F_B|', 'Location', 'best');
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, 'pin force [N]');   title(ax, 'Joint forces');   set(ax, 'XDir', xDir);
ax = nexttile(tl3);   hold(ax, 'on');
plot(ax, th2d, rad2deg(gam), 'Color', C.inp, 'LineWidth', 2);
patch(ax, th2d([1 end end 1]), [0 0 5 5], C.band, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot(ax, th2d, abs(Xd(7,:))/10, 'Color', C.ground, 'LineWidth', 1);
legend(ax, '\gamma = \theta_2 - \theta_3', 'toggle zone (<5^{\circ})', '|N|/10 [N]', 'Location', 'best');
grid(ax, 'on');   xlabel(ax, '\theta_2 [deg]');   ylabel(ax, '[deg] / [N/10]');   title(ax, 'Distance from toggle and ram side force');   set(ax, 'XDir', xDir);

%% 12. ANIMATION (start -> end -> start) WITH s / v / a GRAPHS
if P.animMode == "loop"
    f4  = figure('Name', 'Animation', 'Color', 'w', 'Position', [60 60 1300 650]);
    tl4 = tiledlayout(f4, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax  = nexttile(tl4, 1, [3 1]);   hold(ax, 'on');                       % left: mechanism
    ap = Wd(Av);   plot(ax, ap(1,:), ap(2,:), ':', 'Color', C.inp);
    hp = Wd(Hv);   plot(ax, hp(1,:), hp(2,:), ':', 'Color', C.inp);
    patch(ax, dieC(1,:), dieC(2,:), C.band, 'EdgeColor', C.ink2);          % die / workpiece
    plot(ax, gdR(1,:), gdR(2,:), 'Color', C.ground, 'LineWidth', 2);       % ram guides
    plot(ax, gdL(1,:), gdL(2,:), 'Color', C.ground, 'LineWidth', 2);
    plot(ax, 0, 0, '^', 'MarkerSize', 10, 'MarkerFaceColor', C.ground, 'Color', C.ground);
    axis(ax, 'equal');   grid(ax, 'on');   xlim(ax, lims(1:2));   ylim(ax, lims(3:4));
    xlabel(ax, 'x [mm]');   ylabel(ax, 'y [mm]');

    % right: ram displacement / velocity / acceleration against the crank angle (red dot = now)
    gdat = {1e3*(xB - xB(1)), 1e3*vBn, aBn};
    glab = {'s [mm]', 'v [mm/s]', 'a [m/s^2]'};
    gtit = {'Ram displacement', 'Ram velocity', 'Ram acceleration'};
    dots = gobjects(1, 3);
    for q = 1:3
        axg = nexttile(tl4, 2*q);   hold(axg, 'on');
        plot(axg, th2d, gdat{q}, 'Color', C.inp, 'LineWidth', 1.8);
        if q == 1, yline(axg, 1e3*(xContact - xB(1)), '--', 'Color', C.ink2); end   % work stroke starts
        dots(q) = plot(axg, th2d(1), gdat{q}(1), 'o', 'MarkerSize', 7, ...
                       'MarkerFaceColor', [0.88 0.1 0.1], 'MarkerEdgeColor', [0.88 0.1 0.1]);
        grid(axg, 'on');   ylabel(axg, glab{q});   title(axg, gtit{q});   set(axg, 'XDir', xDir);
        if q == 3, xlabel(axg, 'Crank angle \theta_2 [deg]'); end
    end

    vw = [];                                                   % video file (Motion JPEG AVI)
    if strlength(P.aviFile) > 0
        vw = VideoWriter(char(P.aviFile), 'MPEG-4');
        vw.FrameRate = 30;   vw.Quality = 80;   open(vw);
    end
    h = gobjects(0);   firstFrame = true;
    for k = [1:N_, N_-1:-1:1]
        if ~isvalid(f4), break; end
        delete(h);
        h = drawPose(ax, 1e3*Av(:,k), 1e3*Bv(:,k), P.e, P.wS, P.hS, P.Lh, P.handleAng, Rw, C, 1);
        for q = 1:3, set(dots(q), 'XData', th2d(k), 'YData', gdat{q}(k)); end
        title(ax, sprintf('\\theta_2 = %.1f^{\\circ},  v_B = %.1f mm/s,  T_2 = %.2f N m,  hand force = %.1f N', ...
              rad2deg(th2v(k)), 1e3*vBn(k), T2dyn(k), abs(T2dyn(k))/(P.Lh*1e-3)));
        drawnow;
        if ~isempty(vw), writeVideo(vw, getframe(f4)); end
        if strlength(P.renderFile) > 0 && k == round(0.6*N_)          % one still image for the report
            try, exportgraphics(f4, P.renderFile, 'Resolution', 200); P.renderFile = ""; catch, end
        end
        if strlength(P.gifFile) > 0
            [im, map] = rgb2ind(frame2im(getframe(f4)), 256);
            if firstFrame
                imwrite(im, map, P.gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 1/30);
                firstFrame = false;
            else
                imwrite(im, map, P.gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/30);
            end
        else
            pause(1/30);
        end
    end
    if ~isempty(vw), close(vw); end
end

%% ===================== LOCAL FUNCTIONS =====================
function h = drawPose(ax, A, B, e, wS, hS, Lh, hAng, Rw, C, a)
% Draws the L-shaped input link (crank O2-A + handle), coupler, ram block and pins (mm).
% Model-frame points are rotated by Rw into the world frame (vertical press) before plotting.
h  = gobjects(0, 1);
Rh = [cosd(hAng) -sind(hAng); sind(hAng) cosd(hAng)];
Hd = Lh*Rh*A/norm(A);                                 % handle tip, rigid with the crank at hAng
W  = @(X) Rw*X;
O  = W([0; 0]);   Aw = W(A);   Bw = W(B);   Hw = W(Hd);
blk = W([B(1) + wS/2*[-1 1 1 -1]; e + hS/2*[-1 -1 1 1]]);
h(end+1,1) = plot(ax, [Hw(1) O(1) Aw(1)], [Hw(2) O(2) Aw(2)], '-', 'Color', tint(C.inp, a), 'LineWidth', 3);
h(end+1,1) = plot(ax, Hw(1), Hw(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', tint(C.inp, a), ...
                  'MarkerEdgeColor', C.ink);          % grip / ball knob
h(end+1,1) = plot(ax, [Aw(1) Bw(1)], [Aw(2) Bw(2)], '-', 'Color', tint(C.coup, a), 'LineWidth', 3);
h(end+1,1) = patch(ax, blk(1,:), blk(2,:), C.outp, 'FaceAlpha', 0.5*a, 'EdgeColor', C.outp);
h(end+1,1) = plot(ax, [O(1) Aw(1) Bw(1)], [O(2) Aw(2) Bw(2)], 'o', 'MarkerFaceColor', 'w', ...
                  'Color', C.ink, 'MarkerSize', 6);
end

function c = tint(c, a)
c = 1 - a*(1 - c);
end

function out = pick(cond, a, b)
if cond, out = a; else, out = b; end
end