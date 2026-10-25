function TiltTest_Matlab
% ONLINE AZIMUTH IK + FAST PULSE-PACED RUMBA DRIVER + DIGITAL TWIN.
%
% HOME   : all 6 steppers seek switches, back off and approach slowly.
%          Switch at 21 deg from DOWNWARD vertical = 0 motor steps.
% START  : stream newest MATLAB IK target directly to RUMBA at <=40 Hz.
%          Z ascent and descent are limited; pose change may WAIT for Z.
%          RUMBA continuously replaces previous target (no per-point DONE).
%          Tilt slider can change in REAL TIME. Azimuth sweeps one revolution.
% E-STOP : software stop latches; RUMBA reset and new HOME required.
%
% ANIMATION: commanded model pose, not a measured physical pose.
% The RUMBA counts commanded microsteps, not actual motor encoder movement.
% TEST FIRST UNLOADED (connecting rods detached, plate supported).
% Connected-platform homing/startup is NOT mechanically validated.

%% SETTINGS
RUN_HARDWARE = true;           % start OFFLINE; set true only after verification
COM_PORT = 'COM5';              % check serialportlist("available")
BAUD = 2000000;             % MUST match RUMBA firmware
CONTROL_HZ = 200;                % 5 ms TARGET update interval (not guaranteed)
ANIMATION_HZ = 5;            % 5, slower than control loop
ANIMATE_DURING_HARDWARE = false; % avoids drawing-related serial jitter
SWEEP_AZ_RATE_DEG_S = 45.0;    % 10, 45, gentler diagnostic speed (~36 s sweep)
SWEEP_AZ_ACCEL_DEG_S2 = 20.0; % avoid sudden azimuth starts/stops
MAX_TILT_DEG = 15.0;           % slider maximum for test
DEFAULT_TILT_DEG = 10.0;
TILT_SLEW_DEG_S = 3.0;
TILT_ACCEL_DEG_S2 = 12.0;    % smooth start/stop of changing tilt
Z_MIN_CM = 12.13;
Z_MAX_CM = 22.13;
Z_SEARCH_STEP_CM = 0.05;
Z_LOOKAHEAD_DEG = 12.0;        % 8, 12, small online lookahead; no stored poses
Z_CLEARANCE_CM = 0.15;
MAX_Z_DESCENT_CM_S = 2;     % 0.8, 2, bounded lowering (cm/s)
MAX_Z_ASCENT_CM_S = 2;      % 0.8, 2, ceiling for Z upward velocity (cm/s)
MAX_Z_ACCEL_CM_S2 = 3.5;      % 0.35, 3.5, NEW Z acceleration limit (cm/s^2)
FRACTIONAL_STEPS_PER_MICROSTEP = 256; % must match firmware SETF scale
MICROSTEPS_PER_REV = 3200;    % verify 1/16 microstepping and direct drive
HOME_SWITCH_DOWN_DEG = 21*ones(1,6);
HOME_DIR_HIGH = [true false true false true false];
AWAY_SIGN = 1 - 2*double(HOME_DIR_HIGH);
MAX_AWAY_STEPS = 1000;        % firmware software limit (not a measured stop)
MAX_TARGET_JUMP_STEPS = 30;  % reject a discontinuous computed IK branch
MAX_Z_JUMP_CM = 0.20;       % max height-target change per control update

% DO NOT mark a condition true merely to bypass it.
SWITCHES_PHYSICALLY_AT_21_DEG = true;
DIRECTIONS_AND_MICROSTEPPING_VERIFIED = true;
MAX_136_DOWN_VERIFIED = true;
RODS_DETACHED_AND_PLATE_SUPPORTED = true;
HARDWIRED_EMERGENCY_STOP_AVAILABLE = true;

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
fig = figure('Name','Stewart - LIVE IK + 3D COMMAND PREVIEW',...
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
    sprintf('Live IK %.0f Hz max | 3D preview %.0f Hz',CONTROL_HZ,ANIMATION_HZ),...
    'Units','normalized','Position',[.65 .88 .32 .07],...
    'FontSize',11,'FontWeight','bold','BackgroundColor','w');
uicontrol(fig,'Style','text','String','Target tilt:',...
    'Units','normalized','Position',[.65 .73 .14 .06],...
    'BackgroundColor','w','HorizontalAlignment','left');
slider = uicontrol(fig,'Style','slider','Min',0,'Max',MAX_TILT_DEG,...
    'Value',DEFAULT_TILT_DEG,'Units','normalized',...
    'Position',[.65 .67 .25 .06],'Callback',@onSlider);
valueLabel = uicontrol(fig,'Style','text','String',sprintf('%.1f deg',DEFAULT_TILT_DEG),...
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
            setStatus(['Homing failed: ' ME.message]);
        end
    end

    function onStart(~,~)
        if ~allHomed || streaming || stopRequested,return;end
        streaming=true;
        set(homeBtn,'Enable','off');set(startBtn,'Enable','off');
        setStatus('Starting live IK stream (no 73-point plan)...');
        try
            if RUN_HARDWARE
                writeline(port,'START');
                awaitToken('STREAM_READY',3);
            end
            executeLiveStream();
            if ~stopRequested && ~closed
                setStatus('360 deg complete. Motor holding; fresh START can run again.');
                set(startBtn,'Enable','on');
            end
        catch ME
            sendEStop();
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
        if ~RUN_HARDWARE,return;end
        if stopRequested,error('E-stop requested');end
        % NEW protocol: 1/256 microstep targets. Matching V2 firmware needed.
        writeline(port,sprintf('SETF %d %d %d %d %d %d %d',seq,positionsQ));
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
            if startsWith(line,'ERR'),error('RUMBA: %s',line);end
            if startsWith(line,'STATUS')||startsWith(line,'RX ')
                % Optional low-rate diagnostics.
            elseif ~isempty(line)
                fprintf('RUMBA: %s\n',line);
            end
        end
    end

    function waitForPosition(steps,seq,timeoutS)
        % Only for the initial move from switch zero to a neutral pose.
        if ~RUN_HARDWARE,return;end
        t=tic;lastTx=-inf;lastStatus=-inf;
        while toc(t)<timeoutS
            drawnow;
            if stopRequested||closed,error('Stopped during initial positioning');end
            elapsed=toc(t);
            if elapsed-lastTx>1/CONTROL_HZ
                transmit(seq,steps);lastTx=elapsed;
            end
            if elapsed-lastStatus>.25
                writeline(port,'STATUS');lastStatus=elapsed;
            end
            while port.NumBytesAvailable>0
                line=strtrim(char(readline(port)));
                if startsWith(line,'ERR'),error('RUMBA: %s',line);end
                if startsWith(line,'STATUS ')
                    pieces=strsplit(line);
                    vals=[];
                    if numel(pieces)==8
                        vals=str2double(pieces(3:8));
                    end
                    if numel(vals)==6 && all(isfinite(vals)) && ...
                            all(abs(vals(:).'-round(steps/FRACTIONAL_STEPS_PER_MICROSTEP))<=2)
                        return;
                    end
                end
            end
            pause(.004);
        end
        error('Initial pose not reached within %.1f seconds',timeoutS);
    end

    function executeLiveStream()
        % Start from a neutral 0-degree tilt; no precomputed circle.
        [neutral,zLast,zVelLast,~,~,~]=solveLive(0,0,NaN,0,1/CONTROL_HZ,0,0);
        seq=1;
        target=relativeSteps(neutral);
        transmit(seq,target);
        waitForPosition(target,seq,15);
        if ~RUN_HARDWARE || ANIMATE_DURING_HARDWARE,renderPose(neutral);end
        fprintf('Neutral pose reached/commanded. Begin real-time stream.\n');
        actualTilt=0; tiltVelocity=0; az=0; azRate=0; previousTime=0;
        lastTilt=0;lastAz=0;lastTarget=target;
        meanIK=0;meanCalc=0;maxCalc=0;samples=0;
        maxLoopDt=0; overrunCount=0; workspaceLimited=0;
        lastReport=0;lastFrame=-inf;
        runClock=tic;
        while ~stopRequested && ~closed && az<360
            wall=toc(runClock);
            %if wall-previousTime<1/CONTROL_HZ
            %    pause(min(0.002,1/CONTROL_HZ-(wall-previousTime)));
            %    continue;
            %end
            if wall-previousTime<1/CONTROL_HZ
                continue;
            end
            gap=wall-previousTime;
            maxLoopDt=max(maxLoopDt,gap);
            if gap>1.5/CONTROL_HZ,overrunCount=overrunCount+1;end
            dt=min(.08,max(1e-3,gap));
            previousTime=wall;
            drawnow limitrate; % allow STOP and slider, once per command cycle
            if stopRequested||closed,break;end
            targetTilt=get(slider,'Value'); % NEWEST tilt, read each loop
            tiltError=targetTilt-actualTilt;
            tiltVDesired=sign(tiltError)*min(TILT_SLEW_DEG_S,...
                sqrt(2*TILT_ACCEL_DEG_S2*abs(tiltError)));
            tiltVelocity=tiltVelocity+max(-TILT_ACCEL_DEG_S2*dt,...
                min(TILT_ACCEL_DEG_S2*dt,tiltVDesired-tiltVelocity));
            tiltNext=actualTilt+tiltVelocity*dt;
            if tiltError~=0 && sign(targetTilt-tiltNext)~=sign(tiltError)
                tiltNext=targetTilt;tiltVelocity=0;
            end
            actualTilt=tiltNext;
            % Rather than abruptly starting/stopping azimuth when the
            % tilt slider changes, accelerate/decelerate the azimuth rate.
            azRateGoal=0;
            if abs(actualTilt-targetTilt)<0.05
                azRateGoal=SWEEP_AZ_RATE_DEG_S;
            end
            azRate=azRate+max(-SWEEP_AZ_ACCEL_DEG_S2*dt,...
                min(SWEEP_AZ_ACCEL_DEG_S2*dt,azRateGoal-azRate));
            az=min(360,az+azRate*dt);
            oldZ=zLast;
            [pose,zLast,zVelLast,ikMs,calcMs,limited]=solveLive(...
                actualTilt,az,zLast,zVelLast,dt,lastTilt,lastAz);
            if limited,workspaceLimited=workspaceLimited+1;end
            % If more Z is necessary, the solver can temporarily hold
            % tilt and azimuth while slowly lifting the platform.
            actualTilt=pose.tiltDeg;
            az=pose.azimuthDeg;
            if limited
                % Do not retain an internally large sweep velocity while
                % the geometry prevents a pose transition.
                tiltVelocity=(actualTilt-lastTilt)/dt;
                azRate=min(azRate,max(0,(az-lastAz)/dt));
            end
            steps=relativeSteps(pose);
            if any(abs(steps-lastTarget)> ...
                    MAX_TARGET_JUMP_STEPS*FRACTIONAL_STEPS_PER_MICROSTEP) || ...
                    abs(zLast-oldZ)>MAX_Z_JUMP_CM
                error('Discontinuous IK/Z target: motor or Z jump too large.');
            end
            for t=[0.25 0.50 0.75]
                middle=StewartSmoothModel('pose',geo,...
                    (1-t)*lastTilt+t*actualTilt,...
                    (1-t)*lastAz+t*az,(1-t)*oldZ+t*zLast);
                if ~middle.valid
                    error('Unreachable interpolated Cartesian command.');
                end
            end
            lastTilt=actualTilt; lastAz=az; lastTarget=steps;
            seq=seq+1;
            transmit(seq,steps);   % IMMEDIATE target replacement, no DONE wait
            samples=samples+1;
            meanIK=meanIK+(ikMs-meanIK)/samples;
            meanCalc=meanCalc+(calcMs-meanCalc)/samples;
            maxCalc=max(maxCalc,calcMs);
            inspectMessages();
            % Decimated display: never wait for a frame to finish moving.
            if (~RUN_HARDWARE || ANIMATE_DURING_HARDWARE) && ...
                    wall-lastFrame>=1/ANIMATION_HZ
                renderPose(pose);
                lastFrame=wall;
            end
            if wall-lastReport>=1
                fprintf('az=%6.1f deg | tilt=%5.2f deg | Z=%6.3f cm | ',...
                    az,actualTilt,zLast);
                fprintf(['IK=%6.3f ms | Z+IK=%6.3f ms | samples=%d | ' ...
                    'max loop dt=%.1f ms | overruns=%d | workspace limited=%d\n'],...
                    ikMs,calcMs,samples,1000*maxLoopDt,overrunCount,workspaceLimited);
                setStatus(sprintf(['Streaming: az %.1f/360 deg, tilt %.2f deg, Z %.2f cm. ' ...
                    'Mean IK %.3f ms | mean Z+IK %.3f ms'],...
                    az,actualTilt,zLast,meanIK,meanCalc));
                lastReport=wall;
            end
        end
        if stopRequested||closed,error('Stopped');end
        % Keep sending final target until reached; otherwise a timeout would
        % fault the RUMBA while it is still moving toward final steps.
        if ~RUN_HARDWARE || ANIMATE_DURING_HARDWARE,renderPose(pose);end
        finalTarget=steps;
        waitForPosition(finalTarget,seq,15);
        if RUN_HARDWARE,writeline(port,'HOLD');end
        fprintf('\nLIVE STREAM FINISHED: %d freshly computed poses\n',samples);
        fprintf('Mean IK (six angles): %.4f ms; mean Z optimization + IK: %.4f ms\n',...
            meanIK,meanCalc);
        fprintf('Worst Z optimization + IK: %.4f ms; duration: %.2f s\n',...
            maxCalc,toc(runClock));
        fprintf('Worst control interval: %.2f ms; overruns: %d; limited poses: %d\n',...
            1000*maxLoopDt,overrunCount,workspaceLimited);
    end
end
