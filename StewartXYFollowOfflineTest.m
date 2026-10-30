function StewartXYFollowOfflineTest(R_cm)
% Offline D-only XY follow / combined 3D IK verification. NO HARDWARE.
% R_cm must be the physical center-to-red-marker radius obtained during
% camera calibration (not the 1.50x magenta tracking radius).
%
% Usage: StewartXYFollowOfflineTest(8.23)  % EXAMPLE ONLY, supply YOUR R

validateattributes(R_cm, {'numeric'}, {'scalar','real','finite','positive'});
g = StewartSmoothModel('geometry');
Z = 16.28;               % same fixed Z reported by last live log
MAX_TILT_DEG = 3;        % bench-test conservatively before increasing
XY_KD = 0.12;            % only XY controller tuning gain (seconds)
MAX_XY_R = 0.50;         % absolute allowed displacement relative to GREEN
TARGET_MARGIN = 0.95;   % command stays inside observed limit
microsteps = 3200;
maxAway = 1000;

% Three separate velocity cases, expressed in radius/second.
velocityRps = [0 0; 1 0; 0 1; -1 -1; 10 0];
modelAngleDeg = 0;       % offline mapping example; measured in live setup
rot = [cosd(modelAngleDeg) -sind(modelAngleDeg); ...
       sind(modelAngleDeg)  cosd(modelAngleDeg)];

fprintf('R = %.3f cm | strict displacement cap = %.3f cm (0.50 R)\n', ...
    R_cm, MAX_XY_R*R_cm);
for k=1:size(velocityRps,1)
    v = velocityRps(k,:);
    xyCamR = XY_KD*v;
    n = norm(xyCamR);
    if n>TARGET_MARGIN*MAX_XY_R
        xyCamR=xyCamR*((TARGET_MARGIN*MAX_XY_R)/n);
    end
    modelXY = (rot * [-xyCamR(2);xyCamR(1)])'*R_cm;
    assert(norm(modelXY)/R_cm<=MAX_XY_R+1e-12);
    for tilt = [0 MAX_TILT_DEG]
        for az=0:30:330
            p = StewartSmoothModel('pose',g,tilt,az,Z,modelXY);
            assert(p.valid, 'Unreachable IK for test %d at tilt %.1f az %.1f: %s', ...
                k,tilt,az,p.reason);
            away = (p.thetaDownDeg-g.switchAngleDownDeg)*microsteps/360;
            assert(all(away>=0 & away<=maxAway), 'Motor travel outside limits.');
            assert(max(abs(p.rodErrorCm))<1e-5, '3D rod length check failed.');
        end
    end
    fprintf('ball v = [%+.2f,%+.2f] R/s -> XY target = [%+.3f,%+.3f] R, norm %.3f R. IK OK.\n', ...
        v(1),v(2), modelXY(1)/R_cm, modelXY(2)/R_cm, norm(modelXY)/R_cm);
end
fprintf('All sampled cases passed offline. This is not physical motor validation.\n');
end
