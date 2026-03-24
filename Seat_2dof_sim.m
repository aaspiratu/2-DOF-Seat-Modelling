function seat_2dof_cranklink_app_id()
%% ===== PARAMETER GEOMETRI (mm, rad) =====
r = 43.91;
l = 91.02;
d = 447.94;
p = 225.30;
th_min = deg2rad(-70);
th_max = deg2rad( 70);

motor_offset_y = 80;
motor_offset_x = 0;

th0 = find_mid_theta(r,l,motor_hyp(motor_offset_x,motor_offset_y), th_min, th_max);

%% ===== BATAS FISIK PITCH-ROLL =====
h   = motor_hyp(motor_offset_x,motor_offset_y);
y0  = y_of_theta(th0, r, l, h);
dzup = y_of_theta(th_max, r, l, h) - y0;
dzdn = y0 - y_of_theta(th_min, r, l, h);
dzmax     = min(dzup, dzdn);
pitch_max = dzmax/p;
roll_max  = (2*dzmax)/d; %#ok<NASGU>

%% ===== LIMIT MOTOR & SAFETY =====
safety_mm = 5;
th_min_mech_R = deg2rad(-55);  th_max_mech_R = deg2rad(+70);
th_min_mech_L = deg2rad(-50);  th_max_mech_L = deg2rad(+70);

th_lo_L = max(th_min, th_min_mech_L);
th_hi_L = min(th_max, th_max_mech_L);
th_lo_R = max(th_min, th_min_mech_R);
th_hi_R = min(th_max, th_max_mech_R);

y_min_allowed_L = y_of_theta(th_lo_L, r, l, h) + safety_mm;
y_max_allowed_L = y_of_theta(th_hi_L, r, l, h) - safety_mm;
y_min_allowed_R = y_of_theta(th_lo_R, r, l, h) + safety_mm;
y_max_allowed_R = y_of_theta(th_hi_R, r, l, h) - safety_mm;

alpha_plate_deg = 2.84;  delta_min_deg = 5.91;  tol_deg = 0.50;
theta_down_stop = deg2rad(alpha_plate_deg - (delta_min_deg + tol_deg)); % ≈ -3.57°
motor_rate_max = deg2rad(360);

%% ======== UI ========
fig = figure('Name','Simulasi Kursi 2-DOF (Crank-Link)', ...
    'Units','normalized','Position',[0.05 0.05 0.90 0.85], 'Color','w');

L.x3d=0.07; L.w3d=0.46; L.yTop=0.34; L.h3d=0.67;
L.xR =0.56; L.wR =0.38; L.hCtrl=0.36; L.yCtrl=0.60;
L.hInfo=0.20; L.yInfo=0.38; L.hAxB=0.25; L.yBtm=0.05;

ax3 = axes('Parent',fig,'Units','normalized','Position',[L.x3d L.yTop L.w3d L.h3d]);
hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal'); view(ax3,35,20);
xlabel(ax3,'X (mm)'); ylabel(ax3,'Y (mm)'); zlabel(ax3,'Z (mm)'); title(ax3,'Visualisasi 3D'); ax3.FontSize=12;

pCtrl = uipanel('Parent',fig,'Title','Skenario Jalan','FontSize',12,'Units','normalized','Position',[L.xR L.yCtrl L.wR L.hCtrl]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','Skenario:','FontSize',10.5,'Position',[0.04 0.78 0.26 0.14],'HorizontalAlignment','left');
ddScenario = uicontrol(pCtrl,'Style','popupmenu','Units','normalized','String',{'Polisi Tidur','Rem Mendadak','Tikungan','Jalan Tidak Rata','Gabungan'},'FontSize',10.5,'Value',1,'Position',[0.32 0.78 0.40 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','Waktu Awal (s):','FontSize',10.5,'Position',[0.04 0.58 0.26 0.14],'HorizontalAlignment','left');
efT0 = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.32 0.58 0.18 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','Amplitudo (mm):','FontSize',10.5,'Position',[0.52 0.58 0.26 0.14],'HorizontalAlignment','left');
efAmp = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.80 0.58 0.16 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','Waktu Akhir (s):','FontSize',10.5,'Position',[0.04 0.40 0.26 0.14],'HorizontalAlignment','left');
efT1 = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.32 0.40 0.18 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','σ Kasar (mm):','FontSize',10.5,'Position',[0.52 0.40 0.26 0.14],'HorizontalAlignment','left');
efSigma = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.80 0.40 0.16 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','fc Kasar (Hz):','FontSize',10.5,'Position',[0.04 0.22 0.26 0.14],'HorizontalAlignment','left');
efFc = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.32 0.22 0.18 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','Durasi Total (s):','FontSize',10.5,'Position',[0.52 0.22 0.26 0.14],'HorizontalAlignment','left');
efT = uicontrol(pCtrl,'Style','edit','Units','normalized','String','','FontSize',10.5,'Position',[0.80 0.22 0.16 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','θx (deg):','FontSize',10.5,'Position',[0.37 0.02 0.12 0.14],'HorizontalAlignment','left');
efThx = uicontrol(pCtrl,'Style','edit','Units','normalized','String','', 'FontSize',10.5,'Position',[0.50 0.04 0.15 0.16]);
uicontrol(pCtrl,'Style','text','Units','normalized','String','θy (deg):','FontSize',10.5,'Position',[0.67 0.02 0.12 0.14],'HorizontalAlignment','left');
efThy = uicontrol(pCtrl,'Style','edit','Units','normalized','String','', 'FontSize',10.5,'Position',[0.82 0.04 0.14 0.16]);
btnRun  = uicontrol(pCtrl,'Style','pushbutton','Units','normalized','FontSize',10.5,'String','Run','FontWeight','bold','Position',[0.04 0.04 0.14 0.14],'Callback',@onRun);
btnStop = uicontrol(pCtrl,'Style','pushbutton','Units','normalized','FontSize',10.5,'String','Stop','Position',[0.20 0.04 0.14 0.14],'Callback',@onStop,'BackgroundColor',[0.96 0.96 0.96]);

pInfo = uipanel('Parent',fig,'Title','Info','FontSize',12,'Units','normalized','Position',[L.xR L.yInfo L.wR L.hInfo]);
uicontrol(pInfo,'Style','text','Units','normalized','String','t (s):','FontWeight','bold','FontSize',12,'Position',[0.04 0.82 0.10 0.15],'HorizontalAlignment','left');
hInfo_t = uicontrol(pInfo,'Style','text','Units','normalized','String','0.00','FontSize',12,'Position',[0.15 0.82 0.18 0.15],'HorizontalAlignment','left');
uicontrol(pInfo,'Style','text','Units','normalized','String','pitch (°):','FontWeight','bold','FontSize',12,'Position',[0.40 0.82 0.18 0.15],'HorizontalAlignment','left');
hInfo_pitch = uicontrol(pInfo,'Style','text','Units','normalized','String','0.00','FontSize',12,'Position',[0.60 0.82 0.12 0.15],'HorizontalAlignment','left');
uicontrol(pInfo,'Style','text','Units','normalized','String','roll (°):','FontWeight','bold','FontSize',12,'Position',[0.74 0.82 0.16 0.15],'HorizontalAlignment','left');
hInfo_roll = uicontrol(pInfo,'Style','text','Units','normalized','String','0.00','FontSize',12,'Position',[0.90 0.82 0.06 0.15],'HorizontalAlignment','left');
uicontrol(pInfo,'Style','text','Units','normalized','String','θ_m kiri (°):','FontWeight','bold','FontSize',12,'Position',[0.04 0.50 0.22 0.15],'HorizontalAlignment','left');
hInfo_thL = uicontrol(pInfo,'Style','text','Units','normalized','String','0.00','FontSize',12,'Position',[0.28 0.50 0.12 0.15],'HorizontalAlignment','left','ForegroundColor','r');
uicontrol(pInfo,'Style','text','Units','normalized','String','y kiri (mm):','FontWeight','bold','FontSize',12,'Position',[0.44 0.50 0.20 0.15],'HorizontalAlignment','left');
hInfo_yL = uicontrol(pInfo,'Style','text','Units','normalized','String','0.0','FontSize',12,'Position',[0.66 0.50 0.12 0.15],'HorizontalAlignment','left','ForegroundColor','r');
uicontrol(pInfo,'Style','text','Units','normalized','String','θ_m kanan (°):','FontWeight','bold','FontSize',12,'Position',[0.04 0.22 0.22 0.15],'HorizontalAlignment','left');
hInfo_thR = uicontrol(pInfo,'Style','text','Units','normalized','String','0.00','FontSize',12,'Position',[0.28 0.22 0.12 0.15],'HorizontalAlignment','left','ForegroundColor','b');
uicontrol(pInfo,'Style','text','Units','normalized','String','y kanan (mm):','FontWeight','bold','FontSize',12,'Position',[0.44 0.22 0.30 0.15],'HorizontalAlignment','left');
hInfo_yR = uicontrol(pInfo,'Style','text','Units','normalized','String','0.0','FontSize',12,'Position',[0.66 0.22 0.12 0.15],'HorizontalAlignment','left','ForegroundColor','b');
uicontrol('Parent',pInfo,'Style','text','Units','normalized','String','', 'Position',[0.04 0.12 0.92 0.20],'HorizontalAlignment','left','Visible','off');

ax_th = axes('Parent',fig,'Units','normalized','Position',[L.x3d L.yBtm L.w3d L.hAxB]);
hold(ax_th,'on'); grid(ax_th,'on'); xlabel(ax_th,'t (s)'); ylabel(ax_th,'Sudut motor (deg)'); title(ax_th,'Grafik Sudut Motor');
ax_y  = axes('Parent',fig,'Units','normalized','Position',[L.xR L.yBtm L.wR L.hAxB]);
hold(ax_y,'on'); grid(ax_y,'on'); xlabel(ax_y,'t (s)'); ylabel(ax_y,'Ketinggian y (mm)');   title(ax_y,'Grafik Ketinggian Kursi');
hStop = yline(ax_th, rad2deg(theta_down_stop),'--','down-stop (plat)','LabelHorizontalAlignment','left'); hStop.Annotation.LegendInformation.IconDisplayStyle = 'off';

%% ======== Objek 3D statis ========
A = d/2 + 60; B = p + 60; base_z = -40;
patch(ax3,[-A A A -A],[-B -B B B],[base_z base_z base_z base_z],'FaceColor',[0.92 0.92 0.95],'FaceAlpha',0.7,'EdgeColor',[0.7 0.7 0.7]);
plot3(ax3,0,0,0,'ko','MarkerFaceColor','k','MarkerSize',5);

BL_loc = [ +d/2, p, 0];   % kiri
BR_loc = [ -d/2, p, 0];   % kanan
O_L = [BL_loc(1)+motor_offset_x, BL_loc(2)-motor_offset_y, 0];
O_R = [BR_loc(1)+motor_offset_x, BR_loc(2)-motor_offset_y, 0];

pltSeat  = patch(ax3,'FaceColor',[0.2 0.6 1],'FaceAlpha',0.25,'EdgeColor','b');
pltCrankL = plot3(ax3,[0 0],[0 0],[0 0],'r-','LineWidth',4);
pltLinkL  = plot3(ax3,[0 0],[0 0],[0 0],'r-','LineWidth',4);
pltCrankR = plot3(ax3,[0 0],[0 0],[0 0],'b-','LineWidth',4);
pltLinkR  = plot3(ax3,[0 0],[0 0],[0 0],'b-','LineWidth',4);
plot3(ax3,O_L(1),O_L(2),O_L(3),'ro','MarkerFaceColor','r','MarkerSize',6);
plot3(ax3,O_R(1),O_R(2),O_R(3),'bo','MarkerFaceColor','b','MarkerSize',6);

dotAL = plot3(ax3,0,0,0,'o','Color',[0.8 0 0],   'MarkerFaceColor',[0.8 0 0],'MarkerSize',6);
dotAR = plot3(ax3,0,0,0,'o','Color',[0 0 0.8],   'MarkerFaceColor',[0 0 0.8],'MarkerSize',6);
dotBL = plot3(ax3,0,0,0,'o','Color',[1 0.4 0.1], 'MarkerFaceColor',[1 0.4 0.1],'MarkerSize',6);
dotBR = plot3(ax3,0,0,0,'o','Color',[0.1 0.6 1], 'MarkerFaceColor',[0.1 0.6 1],'MarkerSize',6);

axis(ax3, [-A-100, A+100, -B-100, B+100, base_z-60, y0+140]); ax3.Box='on';

thL_plot = plot(ax_th, NaN, NaN, 'r-', 'DisplayName','Kiri θ_m');
thR_plot = plot(ax_th, NaN, NaN, 'b-', 'DisplayName','Kanan θ_m'); legend(ax_th,'Location','northeast');
yL_plot  = plot(ax_y , NaN, NaN, 'r-', 'DisplayName','Kiri y');
yR_plot  = plot(ax_y , NaN, NaN, 'b-', 'DisplayName','Kanan y');   legend(ax_y ,'Location','northeast');

%% ======== STATE & CALLBACKS ========
is_running = false;

function onRun(~,~)
    if is_running, return; end
    is_running = true;  cleaner = onCleanup(@() set_running_false()); %#ok<NASGU>
    try
        % Parameter UI
        strs = get(ddScenario,'String'); idx  = get(ddScenario,'Value'); scenario = strs{idx};
        t0_ui = str2double(get(efT0,'String'));  t1_ui = str2double(get(efT1,'String'));
        amp   = str2double(get(efAmp,'String')); sigma = str2double(get(efSigma,'String'));
        fc    = str2double(get(efFc,'String'));  T     = str2double(get(efT,'String'));
        thx_man_deg = str2double(get(efThx,'String')); thy_man_deg = str2double(get(efThy,'String'));
        manual_mode = isfinite(thx_man_deg) || isfinite(thy_man_deg);

        if ~isfinite(T) || T<=0, warndlg('Durasi total (T) harus diisi dan > 0.'); is_running=false; return; end
        wtime_missing = ~isfinite(t0_ui) || ~isfinite(t1_ui) || (t1_ui<=t0_ui);
        if ~manual_mode && wtime_missing, warndlg('Isi Waktu Awal & Waktu Akhir dengan benar, atau isi θx/θy manual.'); is_running=false; return; end
        needsAmp = ismember(lower(char(scenario)), {'polisi tidur','speed hump','speed bump','rem mendadak','braking','emergency braking','tikungan','cornering','belok'});
        if ~manual_mode && needsAmp && (~isfinite(amp) || amp<=0), warndlg('Amplitudo harus diisi (>0) untuk skenario ini, atau gunakan θx/θy manual.'); is_running=false; return; end

        if manual_mode
            is_thx_filled = ~isnan(thx_man_deg);
            is_thy_filled = ~isnan(thy_man_deg);
            both_filled_zero = is_thx_filled && is_thy_filled && (abs(thx_man_deg) < 1e-9) && (abs(thy_man_deg) < 1e-9);
            % Manual mode dianggap aktif jika salah satu kolom θ terisi angka (termasuk 0).
            % θx=0 & θy=0 VALID → artinya platform datar (equilibrium).
            if manual_mode
                if ~isfinite(thx_man_deg), thx_man_deg = 0; end
                if ~isfinite(thy_man_deg), thy_man_deg = 0; end
            end

        end

        if ~wtime_missing, duration = t1_ui - t0_ui; center = 0.5*(t0_ui + t1_ui); else, duration=0; center=0; end

        dt = 0.05; t = 0:dt:T;

        % ===== Referensi gerak =====
        if manual_mode
            if ~isfinite(thx_man_deg), thx_man_deg = 0; end
            if ~isfinite(thy_man_deg), thy_man_deg = 0; end
            thx_const = deg2rad(thx_man_deg);  thy_const = deg2rad(thy_man_deg);
            if ~wtime_missing
                s = mj_gate(t, t0_ui, t1_ui);  thx_ref = thx_const*s;  thy_ref = thy_const*s;
            else
                thx_ref = thx_const*ones(size(t));  thy_ref = thy_const*ones(size(t));
            end
        else
            % >>> UPDATED: pass t0 & t1 ke gen_road agar skenario 'jalan tidak rata'
            % dan 'gabungan' aktif hanya pada [t0, t1]
            [wL,wR] = gen_road(scenario, t, t0_ui, t1_ui, duration, amp, center, sigma, fc);
            wL = wL(:).'; wR = wR(:).';
            thx_ref = (wL + wR)./(2*p);
            thy_ref = (wL - wR)./d;
        end

        set(thL_plot,'XData',NaN,'YData',NaN); set(thR_plot,'XData',NaN,'YData',NaN);
        set(yL_plot ,'XData',NaN,'YData',NaN); set(yR_plot ,'XData',NaN,'YData',NaN);
        xlim(ax_th,[0 max(T,dt)]); xlim(ax_y,[0 max(T,dt)]);

        thL_prev = th0; thR_prev = th0;
        thL_log = zeros(size(t)); thR_log = zeros(size(t));
        yL_log  = zeros(size(t)); yR_log  = zeros(size(t));

        for k = 1:numel(t)
            if ~is_running || ~isvalid(fig), break; end

            thx = thx_ref(k);  thy = thy_ref(k);

            S = abs(p*thx) + (d/2)*abs(thy);
            if S > dzmax, s = dzmax/S; thx = thx*s; thy = thy*s; end

            Rm  = rotX(thx) * rotY(-thy);
            BLw = (Rm*BL_loc.').';  BRw = (Rm*BR_loc.').';
            BLw(3)=BLw(3)+y0;       BRw(3)=BRw(3)+y0;

            dzL = p*thx + (d/2)*thy;   yL_tgt = soft_clamp(y0 + dzL, y_min_allowed_L, y_max_allowed_L, 4);
            dzR = p*thx - (d/2)*thy;   yR_tgt = soft_clamp(y0 + dzR, y_min_allowed_R, y_max_allowed_R, 4);

            thL = theta_from_y(yL_tgt, r, l, h, th_lo_L, th_hi_L);
            thR = theta_from_y(yR_tgt, r, l, h, th_lo_R, th_hi_R);

            thL = clamp_down_plate(thL, dzL, theta_down_stop, th_lo_L, th_hi_L);
            thR = clamp_down_plate(thR, dzR, theta_down_stop, th_lo_R, th_hi_R);
            dth_max = motor_rate_max*dt;
            thL = clamp(thL, thL_prev - dth_max, thL_prev + dth_max);
            thR = clamp(thR, thR_prev - dth_max, thR_prev + dth_max);
            thL_prev = thL; thR_prev = thR;

            yL = y_of_theta(thL, r, l, h);
            yR = y_of_theta(thR, r, l, h);

            uL = unit_vec(BL_loc - O_L);  uR = unit_vec(BR_loc - O_R);
            AL = O_L + r*(uL*cos(thL) + [0 0 1]*sin(thL));
            AR = O_R + r*(uR*cos(thR) + [0 0 1]*sin(thR));

            BLvis = fixed_len_link(AL, O_L, uL, [BLw(1) BLw(2) yL], l);
            BRvis = fixed_len_link(AR, O_R, uR, [BRw(1) BRw(2) yR], l);

            seat_local = [-A -B 0;  A -B 0;  A  B 0; -A  B 0];
            seat_world = (Rm*seat_local.').';  seat_world(:,3)=seat_world(:,3)+y0;
            set(pltSeat,'XData',seat_world(:,1),'YData',seat_world(:,2),'ZData',seat_world(:,3));
            set(pltCrankL,'XData',[O_L(1) AL(1)],'YData',[O_L(2) AL(2)],'ZData',[O_L(3) AL(3)]);
            set(pltLinkL , 'XData',[AL(1) BLvis(1)],'YData',[AL(2) BLvis(2)],'ZData',[AL(3) BLvis(3)]);
            set(pltCrankR,'XData',[O_R(1) AR(1)],'YData',[O_R(2) AR(2)],'ZData',[O_R(3) AR(3)]);
            set(pltLinkR , 'XData',[AR(1) BRvis(1)],'YData',[AR(2) BRvis(2)],'ZData',[AR(3) BRvis(3)]);
            set(dotAL,'XData',AL(1),'YData',AL(2),'ZData',AL(3));
            set(dotAR,'XData',AR(1),'YData',AR(2),'ZData',AR(3));
            set(dotBL,'XData',BLvis(1),'YData',BLvis(2),'ZData',BLvis(3));
            set(dotBR,'XData',BRvis(1),'YData',BRvis(2),'ZData',BRvis(3));

            thL_log(k)=thL; thR_log(k)=thR; yL_log(k)=yL; yR_log(k)=yR;
            set(thL_plot,'XData',t(1:k),'YData',rad2deg(thL_log(1:k)));
            set(thR_plot,'XData',t(1:k),'YData',rad2deg(thR_log(1:k)));
            set(yL_plot ,'XData',t(1:k),'YData',yL_log(1:k));
            set(yR_plot ,'XData',t(1:k),'YData',yR_log(1:k));

            yy = [yL_log(1:k) yR_log(1:k)];
            if ~isempty(yy)
                pad = max(1,0.05*range(yy)); ylim(ax_y,[min(yy)-pad, max(yy)+pad]);
            end

            update_info(t(k), thx, thy, thL, thR, yL, yR);
            if mod(k,3)==1, drawnow; pause(dt); end
        end
    catch ME
        warning('Run error (%s): %s', ME.identifier, ME.message);
        rethrow(ME);
    end
end

function onStop(~,~), is_running = false; end
function set_running_false(), is_running = false; end

function update_info(tnow, thx, thy, thL, thR, yL, yR)
    set(hInfo_t   ,'String', sprintf('%.2f', tnow));
    set(hInfo_pitch,'String', sprintf('%.2f', rad2deg(thx)));
    set(hInfo_roll ,'String', sprintf('%.2f', rad2deg(thy)));
    set(hInfo_thL ,'String', sprintf('%.2f', rad2deg(thL)));
    set(hInfo_thR ,'String', sprintf('%.2f', rad2deg(thR)));
    set(hInfo_yL  ,'String', sprintf('%.1f', yL));
    set(hInfo_yR  ,'String', sprintf('%.1f', yR));
end

%% ======== FUNGSI BANTU ========
function h = motor_hyp(dx,dy), h = hypot(dx,dy); end
function y = y_of_theta(th, r, l, h)
    y = r*sin(th) + sqrt(l^2 - (h - r*cos(th)).^2);
end
function th = theta_from_y(y_tgt, r, l, h, th_lo, th_hi)
    f = @(th) y_of_theta(th,r,l,h) - y_tgt;
    y_lo = y_of_theta(th_lo,r,l,h); y_hi = y_of_theta(th_hi,r,l,h);
    y_tgt = min(max(y_tgt, min(y_lo,y_hi)), max(y_lo,y_hi));
    for it=1:60
        th_mid = 0.5*(th_lo+th_hi);
        if f(th_mid) > 0, th_hi = th_mid; else, th_lo = th_mid; end
    end
    th = 0.5*(th_lo+th_hi);
end
function th_mid = find_mid_theta(r,l,h, th_lo, th_hi)
    y_lo = y_of_theta(th_lo,r,l,h); y_hi = y_of_theta(th_hi,r,l,h);
    y_mid = 0.5*(y_lo + y_hi);
    th_mid = fzero(@(th) y_of_theta(th,r,l,h)-y_mid, 0.0);
end
function u = unit_vec(v)
    v(3)=0; n = norm(v(1:2)); if n<1e-9, u=[0 1 0]; else, u=[v(1)/n, v(2)/n, 0]; end
end
function R = rotX(a), ca=cos(a); sa=sin(a); R=[1 0 0; 0 ca -sa; 0 sa ca]; end
function R = rotY(b), cb=cos(b); sb=sin(b); R=[cb 0 sb; 0 1 0; -sb 0 cb]; end
function yout = soft_clamp(y, lo, hi, band)
    if nargin<4, band = 4; end; yout = y;
    if y < lo, yout = lo + band*tanh((y-lo)/band);
    elseif y > hi, yout = hi + band*tanh((y-hi)/band); end
end
function th_out = clamp_down_plate(th_in, dz_dir, th_down_stop, th_lo, th_hi)
    th_out = th_in;
    if dz_dir < 0, th_out = min(th_out, th_down_stop); end
    th_out = min(max(th_out, th_lo), th_hi);
end
function x = clamp(x,a,b), x = max(min(x,b),a); end
function s = mj_gate(t, t0, t1)
    s = zeros(size(t));
    if ~isfinite(t0) || ~isfinite(t1) || t1<=t0, s(:) = 1; return; end
    Tr = max(0.05, 0.15*(t1 - t0));
    t_up0=t0; t_up1=min(t0+Tr, t1); t_dn0=max(t1-Tr, t_up1); t_dn1=t1;
    idx=(t>=t_up0)&(t<=t_up1);
    if any(idx), tau=(t(idx)-t_up0)/(t_up1-t_up0); s(idx)=10*tau.^3-15*tau.^4+6*tau.^5; end
    s((t>t_up1)&(t<t_dn0))=1;
    idx=(t>=t_dn0)&(t<=t_dn1);
    if any(idx), tau=(t(idx)-t_dn0)/(t_dn1-t_dn0); s(idx)=1-(10*tau.^3-15*tau.^4+6*tau.^5); end
end

function Bvis = fixed_len_link(A, O, u, Bworld_target, l)
    z = [0 0 1]; w = cross(z,u); w = w/max(1e-12,norm(w));
    vOB = Bworld_target - O;
    Bp  = Bworld_target - dot(vOB,w)*w;
    dir = Bp - A;
    dir = dir - dot(dir,w)*w;
    nrm = norm(dir); if nrm < 1e-9, dir = u; else, dir = dir/nrm; end
    Bvis = A + l*dir;
end

%% ================== SKENARIO JALAN ==================
function [wL,wR] = gen_road(scenario, t, t0, t1, duration, amp, center, sigma, fc)
    key = char(lower(string(scenario)));
    wL = zeros(size(t)); wR = zeros(size(t));

    % Gate 0→1→0 sesuai waktu mulai/akhir; jika tidak valid, s=1
    s_gate = mj_gate(t, t0, t1);

    switch key
        case {'polisi tidur','speed hump','speed bump'}
            w = bump_half_sine(t, center, duration, amp);  
            wL = w; wR = w;

        case {'rem mendadak','braking','emergency braking'}
            w = braking_pitch_profile(t, center, duration, amp, p); 
            wL = w; wR = w;

        case {'tikungan','cornering','belok'}
            [wL,wR] = cornering_roll_profile(t, center, duration, amp);

        case {'jalan tidak rata','rough','road roughness'}
            % aktif HANYA pada [t0,t1]
            [wL,wR] = rough_pair_enveloped(t, 0.05, sigma, fc, 1, 2);
            wL = wL .* s_gate;
            wR = wR .* s_gate;

        case {'gabungan','combined'}
            % kombinasi efek → tetap digate oleh [t0,t1]
            Tsim = t(end);
            dur_bump = max(0.20, duration); ctr_bump = 0.20*Tsim;
            dur_brake= max(0.20, duration); ctr_brake=0.50*Tsim;
            dur_turn = max(0.30, duration); ctr_turn = 0.75*Tsim;

            w_bump  = bump_half_sine(t, ctr_bump,  dur_bump,  amp);
            w_brake = braking_pitch_profile(t, ctr_brake, dur_brake, amp, p);
            [w_turn_L,w_turn_R] = cornering_roll_profile(t, ctr_turn, dur_turn, amp);
            [wRgh_L,wRgh_R]     = rough_pair_enveloped(t, 0.05, sigma, fc, 3, 4);

            wL = (w_bump + w_brake + w_turn_L + wRgh_L) .* s_gate;
            wR = (w_bump + w_brake + w_turn_R + wRgh_R) .* s_gate;

        otherwise
            % tetap nol
    end
end

function [wL, wR] = cornering_roll_profile(t, center, Tramp, amp_mm)
    T = t(end); Tramp = max(0.05, Tramp);
    t1 = min(center + Tramp/2, T - Tramp);
    t0 = max(0, t1 - Tramp);
    t2 = max(t1, T - Tramp);
    t3 = T;
    s = zeros(size(t));
    idx=(t>=t0)&(t<=t1);
    if any(idx), tau=(t(idx)-t0)/Tramp; s(idx)=10*tau.^3-15*tau.^4+6*tau.^5; end
    s((t>t1)&(t<t2))=1;
    idx=(t>=t2)&(t<=t3);
    if any(idx), tau=(t(idx)-t2)/Tramp; s(idx)=1-(10*tau.^3-15*tau.^4+6*tau.^5); end
    wL = +amp_mm*s;  wR = -amp_mm*s;
end

function w = bump_half_sine(t, center, duration, amp)
    w = zeros(size(t)); t0=center - duration/2;
    idx=(t>=t0)&(t<=t0+duration);
    if any(idx), tt=(t(idx)-t0)/duration; w(idx) = amp * sin(pi*tt); end
end

function w = rough_road(t, dt, sigma, fc, seed)
    if nargin < 5, seed = []; end
    if ~isempty(seed), rng(seed); end
    N = numel(t); white = randn(1,N);
    [b,a] = butter(4, min(0.999, fc*(2*dt)));
    w = filtfilt(b,a,white); w = w / std(w) * sigma;
end

function [wL, wR] = rough_pair(t, dt, sigma, fc, seed_common, seed_slope)
    w_common = rough_road(t, dt, sigma, fc, seed_common);
    slope_frac = 0.35;
    w_slope = rough_road(t, dt, 1.0, fc, seed_slope);
    w_slope = (w_slope / std(w_slope)) * (slope_frac * sigma);
    wL = w_common - w_slope;  wR = w_common + w_slope;
end

function [wL, wR] = rough_pair_enveloped(t, dt, sigma, fc, seed_common, seed_slope)
    [wL_raw, wR_raw] = rough_pair(t, dt, sigma, fc, seed_common, seed_slope);
    T  = t(end) - t(1);
    Tr = max(0.20, 0.20*T);
    s  = ones(size(t));
    idx = t <= (t(1)+Tr);
    if any(idx)
        tau = (t(idx)-t(1))/Tr;
        s(idx) = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    end
    idx = t >= (t(end)-Tr);
    if any(idx)
        tau = (t(idx)-(t(end)-Tr))/Tr;
        s(idx) = 1 - (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    end
    wL = (wL_raw - linspace(wL_raw(1), wL_raw(end), numel(t))) .* s;
    wR = (wR_raw - linspace(wR_raw(1), wR_raw(end), numel(t))) .* s;
    wL(1)=0; wL(end)=0; wR(1)=0; wR(end)=0;
end

function w = braking_pitch_profile(t, center, Trise, w_peak_mm, p)
    if Trise <= 0, Trise = 0.2; end
    t0 = center - Trise/2; t1 = t0 + Trise;
    eps_target = 0.005; t_end = t(end);
    remTime = max(1e-3, t_end - t1);
    tau = remTime / max(1e-6, log(1/eps_target));
    theta_peak = w_peak_mm / p;
    w = zeros(size(t));
    idx_on=(t>=t0)&(t<=t1);
    if any(idx_on), s=(t(idx_on)-t0)/Trise; sMJ=10*s.^3-15*s.^4+6*s.^5; w(idx_on)=p*(theta_peak*sMJ); end
    idx_wash=(t>t1);
    if any(idx_wash), tw=t(idx_wash)-t1; w(idx_wash)=p*(theta_peak*exp(-tw/tau)); end
end
end
