function StewartMousePointControl
% MOUSE "VIRTUAL BALL" TEST FOR THE STEWART PLATFORM
%
% Click on the plate drawing to place a virtual ball. The platform tilts
% exactly as the ball balancer would if the camera had seen the ball
% there: PD control pushes the ball back to the middle.
%
%   Left click / drag   place or move the virtual ball (drag speed feeds Kd)
%   Right click, Space  remove the ball -> platform returns level
%   Q or Esc            return level, HOLD and quit (closing the window too)
%
% The drawing uses the camera view (X right, Y down, same as the tracker
% window). If calibration_config.json from the tracker is found, the plate
% size and the camera rotation (zero azimuth) are taken from it, so a click
% at a spot gives the same tilt as the real ball at that spot.
%
% Start-up: the RUMBA homes and glides to its built-in Z height (see
% StewartFirmwareZ.m); this script reads that height back and checks it.
%
% Requires StewartSmoothModel.m and the RUMBA firmware with NEUTRAL.
% TEST WITH RUN_HARDWARE = false FIRST. Keep a hardwired E-stop at hand.

%% SETTINGS
RUN_HARDWARE = true;           % true = drive the motors
COM_PORT = 'COM5';
BAUD = 250000;

% Same controller as StewartBallControl_USB_upd
Kp = 0.9;                       % deg per cm
Kd = 0.12;                      % deg per (cm/s)
MAX_TILT_DEG = 5;
AZIMUTH_OFFSET_DEG = 0;
MAX_TILT_RATE_DEG_S = 20;       % smooth response to sudden clicks

% Plate size / camera rotation (used if no tracker calibration is found)
TRACKER_FOLDER = '';            % '' = folder of this file
PLATE_R_CM_DEFAULT = 10;        % cyan reference radius R
ZERO_AZIMUTH_DEFAULT = 0;
TRACKING_RADIUS_SCALE = 1.5;    % clicks allowed out to 1.5 R (magenta)

% Mouse velocity for the D term
VEL_FILTER_ALPHA = 0.3;
MOUSE_STILL_S = 0.08;           % no motion this long -> velocity decays

% Motor timing (must match firmware)
TIMER1_HZ = 2000000;
MOTOR_STEP_DELAY_TICKS = 2222;
COMMAND_HZ = 200;
STREAM_HEARTBEAT_HZ = 50;
STATUS_HZ = 10;
DISPLAY_HZ = 20;
MAX_COMMAND_LEAD_STEPS = 200;
STATUS_TIMEOUT_S = 1.0;
NEUTRAL_TIMEOUT_S = 15;
HOME_TIMEOUT_S = 120;
EXIT_LEVEL_TIMEOUT_S = 3;

% Geometry / motor limits (same as the other scripts)
SWITCH_DOWN = 21*ones(1,6);
HOME_DIR_HIGH = [true false true false true false];
AWAY_SIGN = 1-2*double(HOME_DIR_HIGH);
MICROSTEPS_PER_REV = 3200;
POSITION_SCALE = 256;
MAX_AWAY_STEPS = 1000;
Z_OFFLINE_CM = 20.00;           % offline only; hardware uses firmware Z
Z_CLEARANCE_CM = 0.15;
TILT_CHECK_STEP_DEG = 2;
AZ_CHECK_STEP_DEG = 30;
NEUTRAL_MATCH_TOL_USTEPS = 2;

%% STATE
geo = StewartSmoothModel('geometry');
geo.switchAngleDownDeg = SWITCH_DOWN;

port = [];
fig = [];
ax = [];
seq = 0;
rxBuffer = '';
streaming = false;
lastStatusTime = 0;

fixedZ = Z_OFFLINE_CM;
firmwareNeutral = [];
cmdU = [0 0];
cmdQ = zeros(1,6);
goalU = [0 0];

ballActive = false;
dragging = false;
ballPos = [0 0];                % cm, camera frame (like BALL message)
ballVel = [0 0];                % cm/s
lastMotionTime = -inf;
quitRequested = false;
clk = tic;

[plateR,zeroAzimuthDeg,redDir] = loadTrackerCalibration();
trackR = TRACKING_RADIUS_SCALE*plateR;

try
    %% RUMBA: home + built-in Z
    if RUN_HARDWARE
        fprintf('Connecting RUMBA on %s at %d baud.\n',COM_PORT,BAUD);
        port = serialport(COM_PORT,BAUD,'Timeout',0.05);
        configureTerminator(port,'LF');
        pause(2.2);
        flush(port);
        sayHello();     % firmware refuses to move without this handshake
        [fixedZ,firmwareNeutral] = homeToBuiltInZ();
    else
        fprintf('OFFLINE MODE: no motors. Z = %.2f cm.\n',fixedZ);
    end

    fprintf('Validating IK coverage at Z = %.2f cm...\n',fixedZ);
    verifyZ(fixedZ);
    cmdQ = poseToQ(checkedPose(0,0,fixedZ));
    if RUN_HARDWARE
        checkNeutralMatches(firmwareNeutral,cmdQ);
        startStream();
        lastStatusTime = toc(clk);
    end
    fprintf('Ready. Click on the plate to place the virtual ball.\n');

    %% FIGURE
    fig = figure('Name','Stewart Mouse Virtual Ball','NumberTitle','off', ...
        'Color','w','WindowButtonDownFcn',@onDown, ...
        'WindowButtonMotionFcn',@onMove,'WindowButtonUpFcn',@onUp, ...
        'WindowKeyPressFcn',@onKey,'CloseRequestFcn',@onClose);

    ax = subplot(1,3,[1 2],'Parent',fig);
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    set(ax,'YDir','reverse');                 % camera view: Y down
    lim = 1.15*trackR;
    xlim(ax,[-lim lim]); ylim(ax,[-lim lim]);
    xlabel(ax,'Camera X (cm)'); ylabel(ax,'Camera Y (cm, down)');
    a = linspace(0,2*pi,181);
    fill(ax,plateR*cos(a),plateR*sin(a),[0.93 0.96 1],'EdgeColor','c','LineWidth',2, ...
        'HitTest','off');
    plot(ax,trackR*cos(a),trackR*sin(a),'m--','LineWidth',1,'HitTest','off');
    plot(ax,0,0,'g+','MarkerSize',16,'LineWidth',2,'HitTest','off');
    if ~isempty(redDir)
        plot(ax,redDir(1),redDir(2),'r+','MarkerSize',14,'LineWidth',2,'HitTest','off');
    end
    pushHandle = quiver(ax,0,0,0,0,0,'Color',[0.85 0.33 0.1],'LineWidth',2, ...
        'MaxHeadSize',0.6,'HitTest','off');
    ballHandle = plot(ax,nan,nan,'o','MarkerSize',18,'LineWidth',2, ...
        'MarkerFaceColor',[1 0.55 0],'MarkerEdgeColor',[0.6 0.3 0],'HitTest','off');
    set(ax,'ButtonDownFcn',[]);

    axM = subplot(1,3,3,'Parent',fig);
    motorBars = bar(axM,motorAnglesDeg(cmdQ),'FaceColor',[0.3 0.5 0.8]);
    set(axM,'XTickLabel',{'X','Y','Z','E0','E1','E2'});
    ylabel(axM,'Motor angle from vertical (deg)');
    ylim(axM,[SWITCH_DOWN(1)-2, SWITCH_DOWN(1)+MAX_AWAY_STEPS*360/MICROSTEPS_PER_REV]);
    grid(axM,'on');
    title(axM,sprintf('Z = %.2f cm',fixedZ));

    %% MAIN LOOP
    lastCommandTime = 0;
    lastSetfTime = -inf;
    lastPollTime = 0;
    lastDisplayTime = 0;
    exitStart = NaN;

    while true
        now = toc(clk);

        % Begin exit: ramp back to level first.
        if quitRequested && isnan(exitStart)
            exitStart = now;
            ballActive = false;
            dragging = false;
            fprintf('Returning platform to level...\n');
        end
        if ~isnan(exitStart) && ...
                (norm(cmdU)<1e-3 || now-exitStart>EXIT_LEVEL_TIMEOUT_S)
            break;
        end

        % Mouse velocity decays when the mouse is still.
        if now-lastMotionTime>MOUSE_STILL_S
            ballVel = (1-VEL_FILTER_ALPHA)*ballVel;
        end

        % PD "ball to the middle", same as the ball balancer.
        if ballActive
            correction = Kp*ballPos + Kd*ballVel;
            goalU = limitNorm([-correction(2),correction(1)],MAX_TILT_DEG);
        else
            goalU = [0 0];
        end

        % Restart stream after a watchdog stop (e.g. MATLAB was busy).
        if RUN_HARDWARE && ~streaming
            startStream();
            lastStatusTime = now;
            lastSetfTime = -inf;
        end

        % New IK target.
        if now-lastCommandTime>=1/COMMAND_HZ
            dt = min(max(now-lastCommandTime,0.001),0.10);
            if lastCommandTime==0, dt = 1/COMMAND_HZ; end
            lastCommandTime = now;
            qOld = cmdQ;
            [cmdU,cmdQ] = boundedTiltMove(cmdU,cmdQ,goalU,dt);
            if RUN_HARDWARE && streaming && ...
                    (any(cmdQ~=qOld) || now-lastSetfTime>=1/STREAM_HEARTBEAT_HZ)
                transmit(cmdQ);
                lastSetfTime = now;
            end
        end

        % RUMBA status / errors.
        if RUN_HARDWARE
            if now-lastPollTime>=1/STATUS_HZ
                writeline(port,'STATUS');
                lastPollTime = now;
            end
            lines = receiveLines();
            for j = 1:numel(lines)
                msg = lines{j};
                if strcmp(msg,'WARN STREAM_WATCHDOG_HOLD')
                    fprintf('RUMBA watchdog HOLD - restarting stream.\n');
                    streaming = false;
                    continue;
                end
                if startsWith(msg,'ERR') || contains(msg,'STOPPED')
                    error('RUMBA: %s',msg);
                end
                if startsWith(msg,'STATUS ')
                    f = strsplit(msg);
                    if numel(f)==8
                        steps = str2double(f(3:8));
                        if all(isfinite(steps))
                            lastStatusTime = now;
                            lag = max(abs(round(cmdQ/POSITION_SCALE)-steps));
                            if streaming && lag>MAX_COMMAND_LEAD_STEPS
                                error('Target leads step counters by %.0f steps.',lag);
                            end
                        end
                    end
                end
            end
            if streaming && now-lastStatusTime>STATUS_TIMEOUT_S
                error('No valid RUMBA STATUS response.');
            end
        end

        % Display.
        if now-lastDisplayTime>=1/DISPLAY_HZ
            if ballActive
                set(ballHandle,'XData',ballPos(1),'YData',ballPos(2));
                push = -(Kp*ballPos + Kd*ballVel);      % toward the middle
                if norm(push)>1e-9
                    push = push/norm(push)*min(norm(ballPos),plateR)* ...
                        (norm(cmdU)/MAX_TILT_DEG);
                end
                set(pushHandle,'XData',ballPos(1),'YData',ballPos(2), ...
                    'UData',push(1),'VData',push(2));
                title(ax,sprintf(['Ball [%+.2f, %+.2f] cm | speed %.1f cm/s | ' ...
                    'tilt %.2f deg toward middle'], ...
                    ballPos(1),ballPos(2),norm(ballVel),norm(cmdU)));
            else
                set(ballHandle,'XData',nan,'YData',nan);
                set(pushHandle,'UData',0,'VData',0);
                title(ax,sprintf('No ball - level (tilt %.2f deg). Click on the plate.', ...
                    norm(cmdU)));
            end
            set(motorBars,'YData',motorAnglesDeg(cmdQ));
            drawnow limitrate;
            lastDisplayTime = now;
        end

        pause(0.002);
    end

catch ME
    fprintf(2,'[STOPPED] %s\n',ME.message);
    if RUN_HARDWARE && ~isempty(port)
        try, writeline(port,'e'); catch, end
        streaming = false;
    end
end

%% CLEANUP
if RUN_HARDWARE && ~isempty(port) && streaming
    try, writeline(port,'HOLD'); catch, end
    streaming = false;
end
if ~isempty(fig) && isgraphics(fig)
    delete(fig);
end
clear port;
fprintf('Mouse control ended.\n');

%% ============================================================
% MOUSE / KEYBOARD
% =============================================================
    function onDown(~,~)
        if quitRequested, return; end
        if strcmp(get(fig,'SelectionType'),'alt')      % right click
            removeBall();
            return;
        end
        p = mousePoint();
        if isempty(p) || norm(p)>trackR, return; end
        if ~ballActive
            ballVel = [0 0];                % no derivative kick on placement
        end
        ballPos = p;
        ballActive = true;
        dragging = true;
        lastMotionTime = toc(clk);
    end

    function onMove(~,~)
        if ~dragging || quitRequested, return; end
        p = mousePoint();
        if isempty(p), return; end
        p = limitNorm(p,trackR);
        t = toc(clk);
        dtm = t-lastMotionTime;
        if dtm>1e-3 && dtm<0.3
            ballVel = (1-VEL_FILTER_ALPHA)*ballVel + VEL_FILTER_ALPHA*(p-ballPos)/dtm;
        end
        ballPos = p;
        lastMotionTime = t;
    end

    function onUp(~,~)
        dragging = false;
    end

    function onKey(~,event)
        switch lower(event.Key)
            case {'q','escape'}
                quitRequested = true;
            case 'space'
                removeBall();
        end
    end

    function onClose(~,~)
        quitRequested = true;       % level first, then the loop closes it
    end

    function removeBall()
        ballActive = false;
        dragging = false;
        ballVel = [0 0];
    end

    function p = mousePoint()
        p = [];
        cp = get(ax,'CurrentPoint');
        xl = xlim(ax); yl = ylim(ax);
        x = cp(1,1); y = cp(1,2);
        if x<xl(1) || x>xl(2) || y<yl(1) || y>yl(2), return; end
        p = [x y];
    end

%% ============================================================
% CALIBRATION FROM THE TRACKER
% =============================================================
    function [R,zeroAz,redRel] = loadTrackerCalibration()
        R = PLATE_R_CM_DEFAULT;
        zeroAz = ZERO_AZIMUTH_DEFAULT;
        redRel = [];
        folder = TRACKER_FOLDER;
        if isempty(folder), folder = fileparts(mfilename('fullpath')); end
        f = fullfile(folder,'calibration_config.json');
        if ~isfile(f)
            fprintf(['No tracker calibration found (%s).\n' ...
                'Using R = %.1f cm, zero azimuth = %.0f deg.\n'],f,R,zeroAz);
            return;
        end
        try
            c = jsondecode(fileread(f));
            s = c.scale_px_to_cm;
            zeroAz = c.zero_azimuth_deg;
            if isfield(c,'red_marker_x') && isfield(c,'red_marker_y')
                redRel = [c.red_marker_x-c.center_x, c.red_marker_y-c.center_y]*s;
                R = norm(redRel);
            else
                R = c.plate_radius_px*s;
            end
            fprintf('Tracker calibration: R = %.2f cm, zero azimuth = %.1f deg.\n',R,zeroAz);
        catch
            warning('Could not read %s; using defaults.',f);
            R = PLATE_R_CM_DEFAULT;
            zeroAz = ZERO_AZIMUTH_DEFAULT;
            redRel = [];
        end
    end

%% ============================================================
% MOTION
% =============================================================
    function [uSafe,qSafe] = boundedTiltMove(uStart,qStart,uWanted,dt)
        % Tilt-rate and motor step-rate limited move with IK checks.
        uWanted = uStart + limitNorm(uWanted-uStart,MAX_TILT_RATE_DEG_S*dt);
        maxDeltaSteps = (TIMER1_HZ/MOTOR_STEP_DELAY_TICKS)*dt;
        alpha = 1;
        for attempt = 1:20
            uSafe = uStart + alpha*(uWanted-uStart);
            try
                qSafe = poseToQ(poseForU(uSafe));
            catch ME
                if attempt==20, rethrow(ME); end
                alpha = alpha*0.5;
                continue;
            end
            deltaSteps = max(abs((qSafe-qStart)/POSITION_SCALE));
            if deltaSteps<=maxDeltaSteps+1e-6
                return;
            end
            alpha = alpha*max(0.1,min(0.9,0.9*maxDeltaSteps/max(deltaSteps,eps)));
        end
        error('Motor rate limit cannot be satisfied.');
    end

    function p = poseForU(u)
        tilt = norm(u);
        if tilt<1e-9
            az = 0;
        else
            az = mod(atan2d(u(2),u(1))+zeroAzimuthDeg+AZIMUTH_OFFSET_DEG,360);
        end
        p = checkedPose(tilt,az,fixedZ);
    end

    function deg = motorAnglesDeg(q)
        deg = SWITCH_DOWN + abs(q)/POSITION_SCALE*360/MICROSTEPS_PER_REV;
    end

    function x = limitNorm(x,limit)
        m = norm(x);
        if m>limit, x = x*(limit/m); end
    end

    function verifyZ(z)
        tGrid = unique([0:TILT_CHECK_STEP_DEG:MAX_TILT_DEG, MAX_TILT_DEG]);
        for zCheck = [z, z-Z_CLEARANCE_CM]
            for tilt = tGrid
                for az = 0:AZ_CHECK_STEP_DEG:359.999
                    p = StewartSmoothModel('pose',geo,tilt,az,zCheck);
                    if ~isTravelValid(p)
                        error(['Z = %.2f cm cannot reach %.1f deg tilt at azimuth %d deg. ' ...
                            'Pick another height with StewartFirmwareZ.'], ...
                            zCheck,tilt,round(az));
                    end
                end
            end
        end
    end

    function okay = isTravelValid(p)
        okay = false;
        if ~p.valid, return; end
        away = (p.thetaDownDeg-SWITCH_DOWN)*MICROSTEPS_PER_REV/360;
        okay = all(isfinite(away)) && all(away>=0) && all(away<=MAX_AWAY_STEPS);
    end

    function p = checkedPose(tilt,az,z)
        p = StewartSmoothModel('pose',geo,tilt,az,z);
        if ~p.valid, error('IK rejected pose: %s',p.reason); end
        if ~isTravelValid(p), error('Pose exceeds motor travel limits.'); end
    end

    function q = poseToQ(p)
        away = (p.thetaDownDeg-SWITCH_DOWN)*MICROSTEPS_PER_REV/360;
        assert(all(isfinite(away)) && all(away>=0) && all(away<=MAX_AWAY_STEPS), ...
            'Motor travel exceeded.');
        q = round(away*POSITION_SCALE).*AWAY_SIGN;
    end

%% ============================================================
% RUMBA COMMUNICATION
% =============================================================
    function [z,neutral] = homeToBuiltInZ()
        state = queryState(2.0);
        switch state
            case 'STOPPED'
                error(['RUMBA is in a fault state. Fix the cause, then ' ...
                    'press reset on the board (or send RESET) and run again.']);
            case 'HOMING'
                fprintf('RUMBA is homing by itself...\n');
                awaitToken('HOMED_ALL',HOME_TIMEOUT_S);
            case {'STREAMING','STOPPING'}
                writeline(port,'HOLD');
                pause(1.0);
            case 'HOMED'
            otherwise
                fprintf('Homing all six motors...\n');
                writeline(port,'HOME');
                awaitToken('HOMED_ALL',HOME_TIMEOUT_S);
        end
        [z,neutral] = queryNeutral(1.0);
        if z<=0
            error('%s\n%s', ...
                'Z height is not set in the RUMBA firmware (Z_HEIGHT_CM_X100 = 0).', ...
                'Run StewartFirmwareZ, paste its two lines into the firmware, upload, run again.');
        end
        fprintf('RUMBA built-in Z height: %.2f cm.\n',z);
        [ok,steps] = readHoldStatus(1.0);
        if ok && any(abs(steps-neutral)>NEUTRAL_MATCH_TOL_USTEPS)
            fprintf('Gliding back to the built-in Z height...\n');
            startStream();
            commandNeutral(neutral*POSITION_SCALE);
            holdStream();
        end
    end

    function checkNeutralMatches(neutral,q)
        target = round(q/POSITION_SCALE);
        if any(abs(neutral-target)>NEUTRAL_MATCH_TOL_USTEPS)
            away = round(abs(q/POSITION_SCALE));
            error(['Firmware NEUTRAL [%s] does not match MATLAB IK [%s] at Z = %.2f cm.\n' ...
                'Paste these lines into the firmware and upload again:\n' ...
                '#define Z_HEIGHT_CM_X100  %d\n#define NEUTRAL_AWAY_LIST   %s'], ...
                num2str(neutral),num2str(target),fixedZ, ...
                round(fixedZ*100),strjoin(compose('%d',away),', '));
        end
    end

    function state = queryState(timeoutS)
        writeline(port,'STATUS');
        t = tic;
        while toc(t)<timeoutS
            replies = receiveLines();
            for n = 1:numel(replies)
                f = strsplit(replies{n});
                if numel(f)==8 && strcmp(f{1},'STATUS')
                    state = f{2};
                    return;
                end
            end
            pause(0.005);
        end
        error('RUMBA did not answer STATUS. Check COM port and firmware.');
    end

    function [z,neutral] = queryNeutral(timeoutS)
        writeline(port,'NEUTRAL');
        t = tic;
        while toc(t)<timeoutS
            replies = receiveLines();
            for n = 1:numel(replies)
                f = strsplit(replies{n});
                if numel(f)==8 && strcmp(f{1},'NEUTRAL')
                    v = str2double(f(2:8));
                    if all(isfinite(v))
                        z = v(1)/100;
                        neutral = v(2:7);
                        return;
                    end
                end
                if startsWith(replies{n},'ERR')
                    error('RUMBA: %s (upload the new firmware with NEUTRAL).',replies{n});
                end
            end
            pause(0.005);
        end
        error('RUMBA did not answer NEUTRAL. Upload the new firmware.');
    end

    function [ok,steps] = readHoldStatus(timeoutS)
        ok = false;
        steps = nan(1,6);
        writeline(port,'STATUS');
        t = tic;
        while toc(t)<timeoutS
            replies = receiveLines();
            for n = 1:numel(replies)
                f = strsplit(replies{n});
                if numel(f)==8 && strcmp(f{1},'STATUS') && strcmp(f{2},'HOMED')
                    trial = str2double(f(3:8));
                    if all(isfinite(trial))
                        steps = trial;
                        ok = true;
                        return;
                    end
                end
            end
            pause(0.002);
        end
    end

    function startStream()
        sayHello();
        writeline(port,'START');
        awaitToken('STREAM_READY',3);
        streaming = true;
    end

    function holdStream()
        if streaming
            writeline(port,'HOLD');
            streaming = false;
        end
    end

    function transmit(q)
        seq = seq+1;
        writeline(port,sprintf('SETF %d %d %d %d %d %d %d',seq,q));
    end

    function commandNeutral(q)
        target = round(q/POSITION_SCALE);
        c = tic;
        lastSend = -inf;
        lastPoll = -inf;
        while toc(c)<NEUTRAL_TIMEOUT_S
            t = toc(c);
            if t-lastSend>=1/COMMAND_HZ
                transmit(q);
                lastSend = t;
            end
            if t-lastPoll>=1/STATUS_HZ
                writeline(port,'STATUS');
                lastPoll = t;
            end
            replies = receiveLines();
            for k = 1:numel(replies)
                msg = replies{k};
                if startsWith(msg,'ERR') || contains(msg,'STOPPED')
                    error('RUMBA: %s',msg);
                end
                if startsWith(msg,'STATUS ')
                    f = strsplit(msg);
                    if numel(f)==8
                        steps = str2double(f(3:8));
                        if all(isfinite(steps)) && all(abs(steps-target)<=2)
                            return;
                        end
                    end
                end
            end
            pause(0.002);
        end
        error('Neutral pose timeout.');
    end

    function sayHello()
        % MATLAB handshake: without it the RUMBA refuses every motion command.
        writeline(port,'HELLO STEWART');
        awaitToken('HELLO_OK',2);
    end

    function awaitToken(token,timeout)
        c = tic;
        lastPing = 0;
        while toc(c)<timeout
            if toc(c)-lastPing>=0.5
                writeline(port,'PING');   % keep the MATLAB link alive
                lastPing = toc(c);
            end
            replies = receiveLines();
            for k = 1:numel(replies)
                if strcmp(replies{k},token), return; end
                if startsWith(replies{k},'ERR')
                    error('RUMBA: %s',replies{k});
                end
            end
            pause(0.005);
        end
        error('Timeout waiting for %s.',token);
    end

    function lines = receiveLines()
        lines = {};
        if isempty(port), return; end
        n = port.NumBytesAvailable;
        if n>0
            bytes = read(port,n,'uint8');
            rxBuffer = [rxBuffer char(bytes(:).')];
        end
        if numel(rxBuffer)>4096
            error('RUMBA serial buffer overflow.');
        end
        lf = find(rxBuffer==char(10));
        if isempty(lf), return; end
        start = 1;
        for k = 1:numel(lf)
            lines{end+1} = strtrim(rxBuffer(start:lf(k)-1)); %#ok<AGROW>
            start = lf(k)+1;
        end
        rxBuffer = rxBuffer(start:end);
    end
end
