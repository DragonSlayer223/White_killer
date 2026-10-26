function StewartBallBalance
% ONLINE AZIMUTH IK + FAST PULSE-PACED RUMBA DRIVER + DIGITAL TWIN.
%
% HOME   : all 6 steppers seek switches, back off and approach slowly.
%          Switch at 21 deg from DOWNWARD vertical = 0 motor steps.
% START  : track orange ball and stream motor IK targets at CONTROL_HZ.
%          Z ascent and descent are limited; pose change may WAIT for Z.
%          RUMBA continuously replaces previous target (no per-point DONE).
%          Tilt slider sets the maximum tilt. Camera/PID controls azimuth.
% E-STOP : software stop latches; RUMBA reset and new HOME required.
%
% ANIMATION: commanded model pose, not a measured physical pose.
% The RUMBA counts commanded microsteps, not actual motor encoder movement.
% TEST FIRST UNLOADED (connecting rods detached, plate supported).
% Connected-platform homing/startup is NOT mechanically validated.

%% SETTINGS
RUN_HARDWARE = true; % SAFETY: enable only after unloaded validation           % start OFFLINE; set true only after verification
COM_PORT = 'COM5';              % check serialportlist("available")
BAUD = 250000;             % MUST match RUMBA firmware
CONTROL_HZ = 200;                % 5 ms TARGET update interval (not guaranteed)
ANIMATION_HZ = 5;            % 5, slower than control loop
ANIMATE_DURING_HARDWARE = false; % avoids drawing-related serial jitter
SWEEP_AZ_RATE_DEG_S = 60.0;    % 10, 45, gentler diagnostic speed (~36 s sweep)
SWEEP_AZ_ACCEL_DEG_S2 = 60.0; % avoid sudden azimuth starts/stops
MAX_TILT_DEG = 15.0;           % slider maximum for test
DEFAULT_TILT_DEG = 10.0;
TILT_SLEW_DEG_S = 10.0;
TILT_ACCEL_DEG_S2 = 40.0;    % smooth start/stop of changing tilt
Z_MIN_CM = 12.13;
Z_MAX_CM = 22.13;
Z_SEARCH_STEP_CM = 0.05;
Z_LOOKAHEAD_DEG = 12.0;        % 8, 12, small online lookahead; no stored poses
Z_CLEARANCE_CM = 0.15;
MAX_Z_DESCENT_CM_S = 3;     % 0.8, 2, bounded lowering (cm/s)
MAX_Z_ASCENT_CM_S = 3;      % 0.8, 2, ceiling for Z upward velocity (cm/s)
MAX_Z_ACCEL_CM_S2 = 6;      % 0.35, 3.5, NEW Z acceleration limit (cm/s^2)
FRACTIONAL_STEPS_PER_MICROSTEP = 256; % must match firmware SETF scale
MICROSTEPS_PER_REV = 3200;    % verify 1/16 microstepping and direct drive
HOME_SWITCH_DOWN_DEG = 21*ones(1,6);
HOME_DIR_HIGH = [true false true false true false];
AWAY_SIGN = 1 - 2*double(HOME_DIR_HIGH);
MAX_AWAY_STEPS = 1000;        % firmware software limit (not a measured stop)
MAX_TARGET_JUMP_STEPS = 40;  % reject a discontinuous computed IK branch
MAX_Z_JUMP_CM = 0.20;       % max height-target change per control update

lastSentCommand = '';

% DO NOT mark a condition true merely to bypass it.
SWITCHES_PHYSICALLY_AT_21_DEG = true;
DIRECTIONS_AND_MICROSTEPPING_VERIFIED = true;
MAX_136_DOWN_VERIFIED = true;
RODS_DETACHED_AND_PLATE_SUPPORTED = true;
HARDWIRED_EMERGENCY_STOP_AVAILABLE = true;

%% CAMERA / BALL CONTROL SETTINGS (from tracker_test.py and BallBalancerPID.py)
% Camera acquisition is synchronous and can interrupt the 200 Hz stream.
% FIRST RUN OFFLINE. A camera preview does NOT establish hardware safety.
CAMERA_URL = 'http://10.98.211.17:4747/video';
CAMERA_HZ = 15;                 % camera FPS is separate from CONTROL_HZ
CAMERA_PREVIEW_HZ = 5;
CALIBRATE_CAMERA = true;        % ball at exact plate center; click red tape
TRACKER.hMin=5; TRACKER.hMax=25; TRACKER.sMin=100; TRACKER.sMax=255;
TRACKER.vMin=80; TRACKER.vMax=255;
TRACKER.minArea=30; TRACKER.minRadius=4; TRACKER.minCircularity=0.45;
TRACKER.ballDiameterCm=4.0;
TRACKER.calibrationFile=fullfile(fileparts(mfilename('fullpath')),'calibration_config.json');
TRACKER.Kp=0.2; TRACKER.Ki=0.0; TRACKER.Kd=0.4;
TRACKER.maxTiltDeg=5.0;       % conservative prototype (Python used 12 deg)
TRACKER.integralLimit=10;
TRACKER.derivativeAlpha=0.20;
TRACKER.maxLostSeconds=1.0;
TRACKER.maxStaleSeconds=0.35;
TRACKER.maxBallRadiusRatio=0.95; % stop when ball approaches edge
% Empirical motor/camera axis alignment MUST be verified before live use.
% Never invert signs with power on or while rods are connected.

%% MODEL AND UI
geo = StewartSmoothModel('geometry');
geo.switchAngleDownDeg = HOME_SWITCH_DOWN_DEG;
if RUN_HARDWARE
    checks = [SWITCHES_PHYSICALLY_AT_21_DEG,...
        DIRECTIONS_AND_MICROSTEPPING_VERIFIED,MAX_136_DOWN_VERIFIED,...
        RODS_DETACHED_AND_PLATE_SUPPORTED,HARDWIRED_EMERGENCY_STOP_AVAILABLE];
    assert(all(checks),['Hardware mode requires five verified safety checks. ' ...
        'DO NOT HOME WITH THE SIX CONNECTING RODS ATTACHED.']);
end

port = [];
rxBuffer = '';
stopRequested = false;
allHomed = false;
streaming = false;
closed = false;
fig = figure('Name','Stewart - BALL TRACKER + IK (experimental)',...
    'NumberTitle','off','MenuBar','none','ToolBar','none',...
    'Position',[100 120 1170 670], 'Color','w',...
    'CloseRequestFcn',@onClose, 'WindowKeyPressFcn',@onKey);

% Graphics objects are created ONCE; no clf() in the feedback loop.
ax = axes('Parent',fig,'Units','normalized','Position',[.07 .15 .54 .77]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlim(ax,[-15 15]); ylim(ax,[-15 15]); zlim(ax,[-5 25]);
xlabel(ax,'X (cm)'); ylabel(ax,'Y (cm)'); zlabel(ax,'Z (cm)');
title(ax,'Commanded pose (not measured motor feedback)');
scatter3(ax,geo.base(1,:),geo.base(2,:),geo.base(3,:),...
    70,[.82 .17 .12],'filled');
for i=1:6
    text(ax,geo.base(1,i)+.4,geo.base(2,i)+.4,geo.base(3,i),...
        sprintf('%d',i),'FontWeight','bold','Color',[.72 .20 .12]);
end
jointDots = scatter3(ax,nan(1,6),nan(1,6),nan(1,6),...
    62,[.08 .19 .90],'filled');
elbowDots = scatter3(ax,nan(1,6),nan(1,6),nan(1,6),...
    32,[1 .48 .05],'filled');
armLines = gobjects(1,6);
rodLines = gobjects(1,6);
for i=1:6
    armLines(i) = plot3(ax,nan,nan,nan,'Color',[1 .50 .02],'LineWidth',3);
    rodLines(i) = plot3(ax,nan,nan,nan,'Color',[.25 .68 .20],'LineWidth',2.2);
end
plateFill = fill3(ax,nan,nan,nan,[.15 .82 .93],...
    'FaceAlpha',.22,'EdgeColor',[.09 .28 .86],'LineWidth',2);
peakArrow = quiver3(ax,0,0,0,0,0,0,0,'Color',[.84 .19 .82],...
    'LineWidth',2.5,'MaxHeadSize',.5);
plateOutlineLocal = geo.rPlate*[cosd(0:5:360);sind(0:5:360);zeros(1,73)];

uicontrol(fig,'Style','text','String',...
    sprintf('Ball tracking %.0f Hz | IK %.0f Hz',CAMERA_HZ,CONTROL_HZ),...
    'Units','normalized','Position',[.65 .88 .32 .07],...
    'FontSize',11,'FontWeight','bold','BackgroundColor','w');
uicontrol(fig,'Style','text','String','Max tilt limit:',...
    'Units','normalized','Position',[.65 .73 .14 .06],...
    'BackgroundColor','w','HorizontalAlignment','left');
slider = uicontrol(fig,'Style','slider','Min',0,'Max',MAX_TILT_DEG,...
    'Value',TRACKER.maxTiltDeg,'Units','normalized',...
    'Position',[.65 .67 .25 .06],'Callback',@onSlider);
valueLabel = uicontrol(fig,'Style','text','String',sprintf('%.1f deg',TRACKER.maxTiltDeg),...
    'Units','normalized','Position',[.90 .67 .075 .06],'BackgroundColor','w');
homeBtn = uicontrol(fig,'Style','pushbutton','String','HOME',...
    'Units','normalized','Position',[.65 .50 .10 .12],...
    'FontSize',11,'Callback',@onHome);
startBtn = uicontrol(fig,'Style','pushbutton','String','START',...
    'Units','normalized','Position',[.77 .50 .10 .12],...
    'Enable','off','FontSize',11,'Callback',@onStart);
uicontrol(fig,'Style','pushbutton','String','E-STOP (E)',...
    'Units','normalized','Position',[.89 .50 .10 .12],...
    'BackgroundColor',[1 .60 .60],'FontSize',11,'Callback',@onStop);
statusLabel = uicontrol(fig,'Style','text','String','Press HOME.',...
    'Units','normalized','Position',[.65 .12 .33 .30],...
    'BackgroundColor','w','HorizontalAlignment','left','FontSize',10);

if RUN_HARDWARE
    try
        port = serialport(COM_PORT,BAUD,'Timeout',0.1);
        configureTerminator(port,'LF');
        pause(2.2); % USB connection may reset ATmega2560
        flush(port);
        setStatus(['Connected to ' COM_PORT '. Press HOME.']);
    catch ME
        if isgraphics(fig),delete(fig);end
        rethrow(ME);
    end
else
    fprintf('OFFLINE MODE: computes fresh IK; NO RUMBA connection.\n');
end

% Immediately show a MATHEMATICALLY VALID neutral pose so the 3D view is
% not blank while the user waits to press HOME and then START. No motors
% are commanded by this preview, even when RUN_HARDWARE=true.
try
    previewMinZ = StewartStreamIKFast(geo,0,0,Z_MIN_CM,Z_MAX_CM,Z_SEARCH_STEP_CM);
    if isfinite(previewMinZ)
        previewZ = min(Z_MAX_CM, previewMinZ + Z_CLEARANCE_CM);
        previewPose = StewartSmoothModel('pose',geo,0,0,previewZ);
        if previewPose.valid
            renderPose(previewPose);
            if RUN_HARDWARE
                setStatus('Preview only (no motor movement). Click HOME first.');
            else
                setStatus('OFFLINE 3D preview. Click HOME, then START to simulate.');
            end
        else
            warning('StewartLiveSmoother:PreviewInvalid', ...
                'Neutral preview is not feasible: %s',previewPose.reason);
        end
    else
        warning('StewartLiveSmoother:PreviewNoZ', ...
            'No feasible Z height for neutral preview.');
    end
catch previewError
    warning('StewartLiveSmoother:PreviewFailure', ...
        'Unable to render neutral preview: %s',previewError.message);
end

uiwait(fig);

%% CALLBACKS
    function setStatus(msg)
        if isgraphics(statusLabel),set(statusLabel,'String',msg);end
    end

    function renderPose(p)
        % Never solve IK again for graphics. Reuse the already computed
        % validated pose: p.plate, p.elbow, p.R, p.heightCm.
        if closed || ~isgraphics(ax) || ~p.valid,return;end
        P = p.plate;
        E = p.elbow;
        for j=1:6
            B = geo.base(:,j);
            set(armLines(j),'XData',[B(1) E(1,j)],...
                'YData',[B(2) E(2,j)],'ZData',[B(3) E(3,j)]);
            set(rodLines(j),'XData',[E(1,j) P(1,j)],...
                'YData',[E(2,j) P(2,j)],'ZData',[E(3,j) P(3,j)]);
        end
        set(jointDots,'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
        set(elbowDots,'XData',E(1,:),'YData',E(2,:),'ZData',E(3,:));
        origin=[0;0;p.heightCm];
        rim=p.R*plateOutlineLocal+origin;
        set(plateFill,'XData',rim(1,:),'YData',rim(2,:),'ZData',rim(3,:));
        peak=p.R*(geo.rPlate*[cosd(p.azimuthDeg);sind(p.azimuthDeg);0]);
        set(peakArrow,'XData',origin(1),'YData',origin(2),...
            'ZData',origin(3),'UData',peak(1),'VData',peak(2),'WData',peak(3));
        title(ax,sprintf('Target tilt %.1f deg | az %.1f deg | Z %.2f cm',...
            p.tiltDeg,p.azimuthDeg,p.heightCm));
        % This intentionally redraws at a rate LOWER than the control loop.
        drawnow limitrate;
    end

    function onSlider(~,~)
        if isgraphics(valueLabel)
            set(valueLabel,'String',sprintf('%.1f deg',get(slider,'Value')));
        end
    end

    function onHome(~,~)
        if streaming || stopRequested || allHomed,return;end
        set(homeBtn,'Enable','off');
        setStatus('Homing all six motors simultaneously...');
        try
            if RUN_HARDWARE
                writeline(port,'HOME');
                awaitToken('HOMED_ALL',120);
            else
                fprintf('DRY RUN: simulated all six motors homed.\n');
            end
            if stopRequested,error('Software E-stop active');end
            allHomed=true;
            set(startBtn,'Enable','on');
            setStatus('HOMED_ALL: physical switches are 0 steps = 21 deg DOWN. Press START.');
        catch ME
            sendEStop();
        
            fprintf(2, '\n========== STREAM ERROR ==========\n');
            fprintf(2, '%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
            fprintf(2, '==================================\n');
        
            setStatus(['Stream stopped: ' ME.message]);
        end
    end

    function onStart(~,~)
        if ~allHomed || streaming || stopRequested,return;end
        streaming=true;
        set(homeBtn,'Enable','off');set(startBtn,'Enable','off');
        setStatus('Starting live IK stream (no 73-point plan)...');
        try
            executeBallStream();
            if ~stopRequested && ~closed
                setStatus('Ball tracking ended. Motors holding (if connected).');
                set(startBtn,'Enable','on');
            end
        catch ME
            sendEStop();
            fprintf(2,'BALL STREAM ERROR: %s\n',getReport(ME,'extended','hyperlinks','off'));
            setStatus(['Stream stopped: ' ME.message]);
        end
        streaming=false;
    end

    function onStop(~,~)
        sendEStop();
        setStatus('SOFTWARE E-STOP requested. Reset RUMBA and re-home.');
    end

    function onKey(~,event)
        if strcmpi(event.Key,'e'),onStop([],[]);end
    end

    function onClose(~,~)
        closed=true;
        if streaming && ~stopRequested,sendEStop();end
        if ~isempty(port),port=[];end
        if isgraphics(fig),delete(fig);end
    end

    function sendEStop()
        if stopRequested,return;end
        stopRequested=true;
        if RUN_HARDWARE && ~isempty(port)
            try,writeline(port,'e');catch,end
        end
    end

    function awaitToken(token,timeoutSec)
        watch=tic;
        while toc(watch)<timeoutSec
            drawnow;
            if stopRequested||closed,error('Stopped by user');end
            while port.NumBytesAvailable>0
                line=strtrim(char(readline(port)));
                if strcmp(line,token),return;end
                if startsWith(line,'ERR')
                    error('RUMBA error: %s',line);
                end
                if ~isempty(line),fprintf('RUMBA: %s\n',line);end
            end
            pause(.01);
        end
        error('Timed out waiting for %s',token);
    end

    function [p,z,zVelocity,ikMs,calcMs,limited] = solveLive(tilt,az,zPrevious,zVelPrevious,dt,tiltPrev,azPrev)
        totalClock=tic;
        % Search a feasible Z for each new pose, not a stored 73-point path.
        lowestCurrent=StewartStreamIKFast(geo,tilt,az,Z_MIN_CM,Z_MAX_CM,Z_SEARCH_STEP_CM);
        lowestAhead=StewartStreamIKFast(geo,tilt,az+Z_LOOKAHEAD_DEG,...
            Z_MIN_CM,Z_MAX_CM,Z_SEARCH_STEP_CM);
        if ~isfinite(lowestCurrent)||~isfinite(lowestAhead)
            error('No reachable height for tilt %.2f, azimuth %.2f',tilt,az);
        end
        zRequested=max(lowestCurrent,lowestAhead)+Z_CLEARANCE_CM;
        if zRequested>Z_MAX_CM
            error('Required Z %.3f cm exceeds upper search limit.',zRequested);
        end
        limited=false;
        if isnan(zPrevious)
            z=zRequested; zVelocity=0; % plan neutral pose, no physical motion
        else
            % Velocity PLUS acceleration limited online Z height.
            distance=zRequested-zPrevious;
            speedLimit=(distance>=0)*MAX_Z_ASCENT_CM_S+ ...
                       (distance<0)*MAX_Z_DESCENT_CM_S;
            brakingSpeed=sqrt(2*MAX_Z_ACCEL_CM_S2*abs(distance));
            velDesired=sign(distance)*min(speedLimit,brakingSpeed);
            dv=max(-MAX_Z_ACCEL_CM_S2*dt,...
                min(MAX_Z_ACCEL_CM_S2*dt,velDesired-zVelPrevious));
            zVelocity=zVelPrevious+dv;
            z=zPrevious+zVelocity*dt;
            if distance~=0 && sign(zRequested-z)~=sign(distance)
                z=zRequested;zVelocity=0;
            end
        end
        ikClock=tic;
        p=StewartSmoothModel('pose',geo,tilt,az,z);
        if ~p.valid && isfinite(zPrevious)
            % Instead of all-or-nothing freezing of azimuth, find the
            % greatest feasible fraction of this small pose increment.
            % The actual command is whatever this solver returns.
            pOld=StewartSmoothModel('pose',geo,tiltPrev,azPrev,z);
            if ~pOld.valid
                % Cannot even execute Z motion at the last safe orientation.
                z=zPrevious;zVelocity=0;
                pOld=StewartSmoothModel('pose',geo,tiltPrev,azPrev,z);
            end
            if ~pOld.valid
                error('Previous commanded pose is no longer feasible.');
            end
            lo=0; hi=1;
            p=pOld;
            for j=1:9
                mid=0.5*(lo+hi);
                probe=StewartSmoothModel('pose',geo,...
                    tiltPrev+mid*(tilt-tiltPrev),...
                    azPrev+mid*(az-azPrev),z);
                if probe.valid
                    lo=mid; p=probe;
                else
                    hi=mid;
                end
            end
            limited=true;
        end
        ikMs=toc(ikClock)*1000;
        if ~p.valid,error('No valid smooth transition: %s',p.reason);end
        % Caller must use p.tiltDeg/p.azimuthDeg as the ACTUALLY commanded
        % pose. It must not advance its internal pose when a lift is needed.
        calcMs=toc(totalClock)*1000;
    end

    function positionsQ=relativeSteps(p)
        rel=p.thetaDownDeg-HOME_SWITCH_DOWN_DEG;
        away=rel*MICROSTEPS_PER_REV/360; % NOT rounded to whole microstep
        if any(~isfinite(away))||any(away<0)||any(away>MAX_AWAY_STEPS)
            error('Angle outside homed switch-relative motor travel.');
        end
        positionsQ=round(away*FRACTIONAL_STEPS_PER_MICROSTEP).*AWAY_SIGN;
    end

    function transmit(seq,positionsQ)
        if ~RUN_HARDWARE
            return;
        end
    
        if stopRequested
            error('E-stop requested');
        end
    
        if numel(positionsQ) ~= 6 || ...
           any(~isfinite(positionsQ)) || ...
           any(positionsQ ~= round(positionsQ))
            error('Invalid motor targets calculated in MATLAB.');
        end
    
        message = sprintf('SETF %d %d %d %d %d %d %d', ...
            seq, positionsQ);
    
        % Save the exact message sent to RUMBA
        lastSentCommand = message;
    
        writeline(port, message);
    end

    function inspectMessages()
        if ~RUN_HARDWARE,return;end
        % Nonblocking reception. readline() could previously block 100 ms
        % if even one byte was present without a complete LF-terminated line.
        available=port.NumBytesAvailable;
        if available==0,return;end
        newBytes=read(port,available,'uint8');
        rxBuffer=[rxBuffer char(newBytes(:).')];
        if numel(rxBuffer)>4096
            error('Serial diagnostic buffer overflow.');
        end
        LF=char(10);
        while true
            newlineIndex=find(rxBuffer==LF,1);
            if isempty(newlineIndex),break;end
            line=strtrim(rxBuffer(1:newlineIndex-1));
            rxBuffer=rxBuffer(newlineIndex+1:end);
            if startsWith(line,'ERR')
                fprintf(2, 'Last transmitted command: %s\n', ...
                    lastSentCommand);
                error('RUMBA: %s', line);
            end            
            if startsWith(line,'STATUS')||startsWith(line,'RX ')
                % Optional low-rate diagnostics.
            elseif ~isempty(line)
                fprintf('RUMBA: %s\n',line);
            end
        end
    end

    function reached = waitForPosition(steps, seq, timeoutS)
        reached = false;
    
        if ~RUN_HARDWARE
            return;
        end
    
        timer = tic;
        lastTx = -inf;
        lastStatus = -inf;
        lastGUI = -inf;
    
        % Buffer for incomplete serial messages
        localRxBuffer = '';
    
        % Transmit the neutral target every 20 ms
        TX_INTERVAL = 0.020;
    
        while toc(timer) < timeoutS
    
            elapsed = toc(timer);
    
            if stopRequested || closed
                error('Stopped during initial positioning');
            end
    
            % 1. Keep the motor stream alive
            if elapsed - lastTx >= TX_INTERVAL
                transmit(seq, steps);
                lastTx = toc(timer);
            end
    
            % 2. Request motor positions every 200 ms
            if elapsed - lastStatus >= 0.200
                writeline(port, 'STATUS');
                lastStatus = toc(timer);
            end
    
            % 3. Read only bytes that are already available
            available = port.NumBytesAvailable;
    
            if available > 0
                bytes = read(port, available, 'uint8');
    
                localRxBuffer = [localRxBuffer, char(bytes(:).')];
    
                if numel(localRxBuffer) > 4096
                    error('RUMBA serial receive buffer overflow');
                end
            end
    
            % 4. Process complete newline-terminated messages
            newlineIndex = find(localRxBuffer == char(10), 1);
    
            while ~isempty(newlineIndex)
    
                line = strtrim(localRxBuffer(1:newlineIndex-1));
    
                localRxBuffer = localRxBuffer(newlineIndex+1:end);
    
                if startsWith(line,'ERR')
                    fprintf(2,'[RUMBA FAULT] %s\n',line);
                    fprintf(2,'Motor motion stopped by firmware.\n');
                    fprintf(2,'Reset and re-home required.\n');
                
                    stopRequested = true;
                    return;
                end
    
                if startsWith(line, 'STATUS ')
    
                    pieces = strsplit(line);
    
                    if numel(pieces) == 8
    
                        motorSteps = str2double(pieces(3:8));
    
                        targetSteps = ...
                            round(steps / FRACTIONAL_STEPS_PER_MICROSTEP);
    
                        if all(isfinite(motorSteps)) && ...
                           all(abs(motorSteps - targetSteps) <= 2)
    
                            fprintf('All six motors reached neutral position.\n');
                            return;
                        end
                    end
                end
    
                newlineIndex = find(localRxBuffer == char(10), 1);
            end
    
            % 5. Keep the MATLAB UI responsive without updating constantly
            if elapsed - lastGUI >= 0.100
                drawnow limitrate;
                lastGUI = toc(timer);
            end
    
            pause(0.001);
        end
    
        fprintf(2, ...
            '[POSITION TIMEOUT] Neutral position not reached in %.1f s.\n', ...
            timeoutS);
        
        stopRequested = true;
        end

    function executeBallStream()
        % The CAMERA supplies pitch/roll, replacing the fixed 360-degree sweep.
        % All six motors still use the original IK, SETF protocol, and RUMBA.
        cam=StewartBallTracker('open',CAMERA_URL);
        if CALIBRATE_CAMERA
            cal=StewartBallTracker('calibrate',cam,TRACKER);
        else
            cal=StewartBallTracker('loadcalibration',TRACKER.calibrationFile);
        end
        pid=StewartBallTracker('pidinit');
        videoFig=figure('Name','Ball tracking: green = ball, red = center', ...
            'NumberTitle','off');
        imgHandle=imshow(zeros(360,640,3,'uint8'));
        fprintf('CAMERA READY | scale=%.5f cm/px | az offset=%.1f deg\n', ...
        cal.scale_px_to_cm,cal.zero_azimuth_deg);

        % Camera calibration is now finished.
        % Only now activate RUMBA streaming.
        if RUN_HARDWARE
            writeline(port, 'START');
            awaitToken('STREAM_READY', 3);
        end

        [neutral,zLast,zVelLast,~,~,~]=solveLive(0,0,NaN,0,1/CONTROL_HZ,0,0);        seq=1; target=relativeSteps(neutral);
        transmit(seq,target);
        reached = waitForPosition(target,seq,15);

        if ~reached
            fprintf(2, ...
                '[STREAM STOPPED] Initial positioning failed.\n');
            return;
        end
        if ~RUN_HARDWARE,renderPose(neutral);end
        fprintf('NEUTRAL READY. Ball control begins.\n');

        tilt=0; tiltVel=0; az=0; azVel=0;
        cmdTilt=0; cmdAz=0;
        lastTilt=0;lastAz=0;lastTarget=target;
        lastCamera=-inf;lastSeen=-inf;lastFrame=-inf;
        lastReport=0;previousTime=0;lastDetectionTime=NaN;
        samples=0;overruns=0;maxGap=0;
        ballErr=[NaN NaN];
        started=tic;
        while ~stopRequested && ~closed
            wall=toc(started);
            if wall-previousTime<1/CONTROL_HZ
                continue;
            end
            dt=min(0.08,max(0.001,wall-previousTime));
            maxGap=max(maxGap,dt);
            if dt>1.5/CONTROL_HZ,overruns=overruns+1;end
            previousTime=wall;
            drawnow limitrate;
            if stopRequested || closed,break;end
            if ~isgraphics(videoFig)
                error('BallTracker:Closed','Camera preview closed; control stopped.');
            end

            if wall-lastCamera>=1/CAMERA_HZ
                frame=snapshot(cam); % ipcam returns RGB; may block
                if isempty(frame)
                    error('BallTracker:Camera','Camera supplied an empty image');
                end
                lastCamera=wall;
                ball=StewartBallTracker('detect',frame,TRACKER);
                if ~isempty(ball)
                    radiusDistance=hypot(ball(1)-cal.center_x,ball(2)-cal.center_y);
                    if radiusDistance>TRACKER.maxBallRadiusRatio*cal.plate_radius_px
                        error('BallTracker:Edge','Ball is too close to the edge.');
                    end
                    frameDt=1/CAMERA_HZ;
                    if ~isnan(lastDetectionTime)
                        frameDt=max(0.001,wall-lastDetectionTime);
                    end
                    [pid,computedTilt,computedAz,ballErr,~]= ...
                        StewartBallTracker('pid',pid,ball,cal,TRACKER,frameDt);
                    cmdTilt=min(computedTilt,get(slider,'Value'));
                    cmdAz=computedAz;
                    lastDetectionTime=wall;
                    lastSeen=wall;
                else
                    cmdTilt=0;
                    pid=StewartBallTracker('pidinit');
                end
                if wall-lastFrame>=1/CAMERA_PREVIEW_HZ
                    set(imgHandle,'CData',frame);
                    % Avoid repeatedly creating graphics objects during control.
                    title(get(imgHandle,'Parent'),sprintf('Ball error: [%.2f, %.2f] cm | Tilt %.2f deg', ...
                        ballErr(1),ballErr(2),cmdTilt));
                    lastFrame=wall;
                end
            end
            if wall-lastCamera>TRACKER.maxStaleSeconds
                cmdTilt=0;
            end
            if wall-lastSeen>TRACKER.maxLostSeconds && wall>TRACKER.maxLostSeconds
                error('BallTracker:Lost','Ball not detected for %.2f seconds.',wall-lastSeen);
            end

            % Slew/acceleration limits on tilt.
            tiltError=cmdTilt-tilt;
            desiredTiltRate=sign(tiltError)*min(TILT_SLEW_DEG_S, ...
                sqrt(2*TILT_ACCEL_DEG_S2*abs(tiltError)));
            tiltVel=tiltVel+max(-TILT_ACCEL_DEG_S2*dt, ...
                min(TILT_ACCEL_DEG_S2*dt,desiredTiltRate-tiltVel));
            tiltNext=tilt+tiltVel*dt;
            if tiltError~=0 && sign(cmdTilt-tiltNext)~=sign(tiltError)
                tiltNext=cmdTilt;tiltVel=0;
            end
            tilt=tiltNext;

            % Shortest angular path to new azimuth target. At zero tilt
            % azimuth is geometrically irrelevant, but we retain continuity.
            azError=mod(cmdAz-az+180,360)-180;
            desiredAzRate=sign(azError)*min(SWEEP_AZ_RATE_DEG_S, ...
                sqrt(2*SWEEP_AZ_ACCEL_DEG_S2*abs(azError)));
            azVel=azVel+max(-SWEEP_AZ_ACCEL_DEG_S2*dt, ...
                min(SWEEP_AZ_ACCEL_DEG_S2*dt,desiredAzRate-azVel));
            azNext=az+azVel*dt;
            if azError~=0 && sign(mod(cmdAz-azNext+180,360)-180)~=sign(azError)
                azNext=az+azError;azVel=0;
            end
            az=azNext;
            oldZ=zLast;
            [pose,zLast,zVelLast,~,~,limited]=solveLive( ...
                tilt,az,zLast,zVelLast,dt,lastTilt,lastAz);
            tilt=pose.tiltDeg;az=pose.azimuthDeg;
            if limited
                tiltVel=(tilt-lastTilt)/dt;
                azVel=(az-lastAz)/dt;
            end
            steps=relativeSteps(pose);
            if any(abs(steps-lastTarget)> ...
                    MAX_TARGET_JUMP_STEPS*FRACTIONAL_STEPS_PER_MICROSTEP) || ...
                    abs(zLast-oldZ)>MAX_Z_JUMP_CM
                error('BallTracker:Jump','Motor or Z target discontinuity');
            end
            for fraction=[0.25 0.5 0.75]
                mid=StewartSmoothModel('pose',geo, ...
                    (1-fraction)*lastTilt+fraction*tilt, ...
                    (1-fraction)*lastAz+fraction*az, ...
                    (1-fraction)*oldZ+fraction*zLast);
                if ~mid.valid
                    error('BallTracker:IK','Unreachable intermediate pose');
                end
            end
            lastTilt=tilt;lastAz=az;lastTarget=steps;
            seq=seq+1;transmit(seq,steps);inspectMessages();
            samples=samples+1;
            if (~RUN_HARDWARE||ANIMATE_DURING_HARDWARE) && ...
                    wall-lastReport>=1/ANIMATION_HZ
                renderPose(pose);
            end
            if wall-lastReport>=1
                fprintf('Ball balance: tilt=%.2f az=%.1f Z=%.2f | samples=%d overruns=%d max dt=%.1f ms\n', ...
                    tilt,mod(az,360),zLast,samples,overruns,1000*maxGap);
                setStatus(sprintf('Tracking: tilt %.2f az %.1f | samples %d overruns %d', ...
                    tilt,mod(az,360),samples,overruns));
                lastReport=wall;
            end
        end
        if isgraphics(videoFig),close(videoFig);end
        if stopRequested||closed,error('User stopped ball tracking');end
        if RUN_HARDWARE,writeline(port,'HOLD');end
    end
end
