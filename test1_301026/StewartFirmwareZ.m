function [zCm,neutralAway] = StewartFirmwareZ(zCm,maxTiltDeg)
% STEWARTFIRMWAREZ  Lines that build the platform Z height into the RUMBA.
%
%   StewartFirmwareZ(20.5)   use Z = 20.5 cm
%   StewartFirmwareZ         lowest Z that reaches the full tilt range
%   StewartFirmwareZ(z,3)    also check up to 3 deg tilt (default 2)
%
% Prints two lines. Paste them over the matching lines in
% RUMBA_Stewart_Firmware.ino (section "Built-in Z height") and upload.
% After homing the RUMBA then glides to this height on its own, and
% StewartBallControl_USB_upd / StewartMousePointControl read it back.
%
% The constants below must match the control scripts and the firmware.

SWITCH_DOWN = 21*ones(1,6);          % switch angle from vertical (deg)
MICROSTEPS_PER_REV = 3200;
MAX_AWAY_STEPS = 1000;               % firmware MAX_AWAY_STEPS
Z_MIN_CM = 16.13;
Z_MAX_CM = 24.13;
Z_STEP_CM = 0.05;
Z_CLEARANCE_CM = 0.15;               % Z must also work this much lower
TILT_CHECK_STEP_DEG = 1;
AZ_CHECK_STEP_DEG = 30;

if nargin<2 || isempty(maxTiltDeg), maxTiltDeg = 2; end

geo = StewartSmoothModel('geometry');
geo.switchAngleDownDeg = SWITCH_DOWN;

if nargin<1 || isempty(zCm)
    zCm = NaN;
    for z = Z_MIN_CM+Z_CLEARANCE_CM : Z_STEP_CM : Z_MAX_CM
        zr = round(z*100)/100;
        if coverageOk(zr), zCm = zr; break; end
    end
    if isnan(zCm)
        error('No Z between %.2f and %.2f cm reaches %.1f deg tilt.', ...
            Z_MIN_CM,Z_MAX_CM,maxTiltDeg);
    end
    fprintf('Lowest Z with full %.1f deg tilt coverage: %.2f cm\n',maxTiltDeg,zCm);
else
    zCm = round(zCm*100)/100;        % firmware stores cm*100 exactly
    if ~coverageOk(zCm)
        error(['Z = %.2f cm cannot reach %.1f deg tilt in every direction ' ...
            '(or Z - %.2f cm cannot). Choose another height.'], ...
            zCm,maxTiltDeg,Z_CLEARANCE_CM);
    end
end

p = StewartSmoothModel('pose',geo,0,0,zCm);
if ~p.valid
    error('Level pose at Z = %.2f cm rejected: %s',zCm,p.reason);
end
away = (p.thetaDownDeg-SWITCH_DOWN)*MICROSTEPS_PER_REV/360;
neutralAway = round(away);
if any(~isfinite(away)) || any(neutralAway<1) || any(neutralAway>MAX_AWAY_STEPS)
    error('Level pose at Z = %.2f cm is outside motor travel.',zCm);
end

fprintf('\nMotor angles at Z = %.2f cm (deg from vertical): %s\n', ...
    zCm,num2str(p.thetaDownDeg,'%8.2f'));
fprintf('\nPaste into RUMBA_Stewart_Firmware.ino and upload:\n\n');
fprintf('#define Z_HEIGHT_CM_X100  %d\n',round(zCm*100));
fprintf('#define NEUTRAL_AWAY_LIST   %s\n\n',strjoin(compose('%d',neutralAway),', '));
fprintf('Offline mode: set Z_OFFLINE_CM = %.2f in the control scripts.\n',zCm);

    function ok = coverageOk(z)
        ok = false;
        tGrid = unique([0:TILT_CHECK_STEP_DEG:maxTiltDeg, maxTiltDeg]);
        for zc = [z, z-Z_CLEARANCE_CM]
            for tilt = tGrid
                for az = 0:AZ_CHECK_STEP_DEG:359.999
                    q = StewartSmoothModel('pose',geo,tilt,az,zc);
                    if ~q.valid, return; end
                    a = (q.thetaDownDeg-SWITCH_DOWN)*MICROSTEPS_PER_REV/360;
                    if any(~isfinite(a)) || any(a<0) || any(a>MAX_AWAY_STEPS)
                        return;
                    end
                end
            end
        end
        ok = true;
    end
end
