function out = StewartSmoothModel(action, varargin)
% STEWARTSMOOTHMODEL - Matched IK geometry and smooth periodic Z planner.
%
% g = StewartSmoothModel('geometry')
% p = StewartSmoothModel('pose',g,tiltDeg,azimuthDeg,zCm)
% plan = StewartSmoothModel('plan',g,tiltDeg,azimuthStep,zMin,zMax)
%
% ANGLE CONVENTION (IMPORTANT):
% thetaDown = 0 deg when a motor horn points VERTICALLY DOWN.
% The user's minimum/endstop is thetaDown = 21 deg.
% thetaUp = 180 - thetaDown (angle from upward vertical).
% Both 21 and the assumed maximum of 136 are used in the *same*
% downward-vertical convention. Verify the 136-deg physical limit.
%
% The model does not energize, contact, or control any real motors.

switch lower(action)
    case 'geometry'
        g.Lshort = 5.1;           % cm
        g.Llong = 17.1;           % cm
        g.rBase = 8.36;           % cm
        g.rPlate = 8.13;          % cm
        g.faceCentersDeg = [0 120 240];
        g.baseOffsetsDeg = [-14.37 14.37];
        g.plateOffsetsDeg = [-2.33 2.33];
        g.minAngleDownDeg = 21.0;   % MINIMUM from DOWNWARD vertical
        g.maxAngleDownDeg = 136.0;  % ASSUMED maximum from DOWNWARD vertical
        g.switchAngleDownDeg = 21.0 * ones(1,6); % nominal, not calibrated
        g.angleSafetyMarginDeg = 0.5;
        g.ZsearchStepCm = 0.02;
        g.ZsafetyMarginCm = 0.08;
        g.additionalLiftCm = 0.03;
        g.ZsmoothingSigmaDeg = 15.0;
        g.maxExtraLiftIterations = 60;
        g.base = zeros(3,6);
        g.plateLocal = zeros(3,6);
        g.swing = zeros(3,6);
        for i=1:6
            f=ceil(i/2);
            j=mod(i-1,2)+1;
            face=g.faceCentersDeg(f);
            baseAngle=face+g.baseOffsetsDeg(j);
            plateAngle=face+g.plateOffsetsDeg(j);
            g.base(:,i)=g.rBase*[cosd(baseAngle); sind(baseAngle); 0];
            g.plateLocal(:,i)=g.rPlate*[cosd(plateAngle); sind(plateAngle); 0];
            tangent=[-sind(face); cosd(face); 0];
            signSide=2*j-3;  % -1 for first motor, +1 for second
            g.swing(:,i)=signSide*tangent;
        end
        out=g;

    case 'pose'
        g=varargin{1}; tilt=varargin{2}; az=varargin{3}; z=varargin{4};
        out=solvePose(g,tilt,az,z);

    case 'plan'
        g=varargin{1}; tilt=varargin{2}; azStep=varargin{3};
        zMin=varargin{4}; zMax=varargin{5};
        out=planCircle(g,tilt,azStep,zMin,zMax);

    otherwise
        error('StewartSmoothModel:UnknownAction','Unknown action: %s',action);
end
end

function R=tiltRotation(tiltDeg,azDeg)
% Tilts upward toward azimuth, without yaw rotation of the upper plate.
axis=[sind(azDeg);-cosd(azDeg);0];
K=[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
R=eye(3)+sind(tiltDeg)*K+(1-cosd(tiltDeg))*(K*K);
end

function p=solvePose(g,tiltDeg,azDeg,zCm)
p.valid=false;
p.thetaDownDeg=NaN(1,6);
p.thetaUpDeg=NaN(1,6);
p.relativeToSwitchDeg=NaN(1,6);
p.elbow=NaN(3,6);
p.rodErrorCm=NaN(1,6);
p.plate=NaN(3,6);
p.R=NaN(3,3);
p.reason='Unreachable pose';
p.heightCm=zCm;
p.azimuthDeg=azDeg;
p.tiltDeg=tiltDeg;
if ~all(isfinite([zCm tiltDeg azDeg])),return;end
R=tiltRotation(tiltDeg,azDeg);
P=R*g.plateLocal+repmat([0;0;zCm],1,6);
p.plate=P; p.R=R;
for i=1:6
    BP=P(:,i)-g.base(:,i);
    x=dot(BP,g.swing(:,i));
    vz=BP(3);
    rho=hypot(x,vz);
    if rho<1e-12, p.reason='Degenerate elbow geometry';return;end

    % Full 3D connecting-rod constraint (includes out-of-plane offset).
    C=(dot(BP,BP)+g.Lshort^2-g.Llong^2)/(2*g.Lshort);
    if abs(C)>rho+1e-8, p.reason='Rod cannot reach';return;end
    alpha=acosd(max(-1,min(1,C/rho)));
    thetaUpOptions=mod(atan2d(x,vz)+[alpha,-alpha],360);
    thetaDownOptions=180-thetaUpOptions;

    allowed=thetaDownOptions >= g.minAngleDownDeg+g.angleSafetyMarginDeg & ...
            thetaDownOptions <= g.maxAngleDownDeg-g.angleSafetyMarginDeg;
    if ~any(allowed)
        p.reason='Outside motor angle limits';return;
    end
    choice=find(allowed,1,'first');
    thetaDown=thetaDownOptions(choice);

    % EXACTLY THE SAME swing plane is used for IK and animation.
    E=g.base(:,i)+g.Lshort*( ...
        sind(thetaDown)*g.swing(:,i)-cosd(thetaDown)*[0;0;1]);
    err=norm(P(:,i)-E)-g.Llong;
    if abs(err)>1e-6
        p.reason='Linkage length mismatch';return;
    end
    p.thetaDownDeg(i)=thetaDown;
    p.thetaUpDeg(i)=180-thetaDown;
    p.relativeToSwitchDeg(i)=thetaDown-g.switchAngleDownDeg(i);
    p.elbow(:,i)=E;
    p.rodErrorCm(i)=err;
end
p.valid=true; p.reason='OK';
end

function angles=poseAtHeights(g,tiltDeg,azDeg,zValues)
% Vectorized IK feasibility used to search all candidate heights.
zValues=zValues(:);
nz=numel(zValues);
angles=NaN(nz,6);
R=tiltRotation(tiltDeg,azDeg);
d=R*g.plateLocal-g.base;
minimum=g.minAngleDownDeg+g.angleSafetyMarginDeg;
maximum=g.maxAngleDownDeg-g.angleSafetyMarginDeg;
for i=1:6
    x=dot(d(:,i),g.swing(:,i));
    vz=d(3,i)+zValues;
    rho=hypot(x,vz);
    fullSquared=d(1,i)^2+d(2,i)^2+vz.^2;
    C=(fullSquared+g.Lshort^2-g.Llong^2)/(2*g.Lshort);
    reachable=rho>1e-12 & abs(C)<=rho+1e-8;
    cosAlpha=C./max(rho,1e-12);
    alpha=acosd(max(-1,min(1,cosAlpha)));
    origin=atan2d(x,vz);
    downA=180-mod(origin+alpha,360);
    downB=180-mod(origin-alpha,360);
    goodA=reachable & downA>=minimum & downA<=maximum;
    goodB=reachable & downB>=minimum & downB<=maximum;
    angles(goodA,i)=downA(goodA);
    useB=~goodA & goodB;
    angles(useB,i)=downB(useB);
end
end

function y=periodicGaussian(x,sigmaSteps)
% Circular convolution prevents a seam at azimuth 0/360.
x=x(:).';
if sigmaSteps<0.1, y=x;return;end
halfWidth=ceil(4*sigmaSteps);
shifts=-halfWidth:halfWidth;
w=exp(-0.5*(shifts/sigmaSteps).^2);
w=w/sum(w);
y=zeros(size(x));
for q=1:numel(shifts)
    y=y+w(q)*circshift(x,[0 shifts(q)]);
end
end

function plan=planCircle(g,tiltDeg,azStep,zMin,zMax)
assert(isscalar(tiltDeg) && tiltDeg>=0 && tiltDeg<=60,...
    'Tilt must be between 0 and 60 degrees.');
assert(isscalar(azStep) && azStep>0 && ...
    abs(360/azStep-round(360/azStep))<1e-8,...
    'Azimuth step must divide 360 exactly.');
assert(zMin>0 && zMax>zMin,'Invalid Z interval.');

az=0:azStep:(360-azStep); % no duplicate point during smoothing
n=numel(az);
zGrid=zMin:g.ZsearchStepCm:zMax;
if zGrid(end)<zMax-1e-9,zGrid=[zGrid zMax];end
minimumZ=NaN(1,n);
for k=1:n
    candidates=poseAtHeights(g,tiltDeg,az(k),zGrid);
    firstValid=find(all(isfinite(candidates),2),1,'first');
    if isempty(firstValid)
        error('No valid height at azimuth %.1f degrees.',az(k));
    end
    minimumZ(k)=zGrid(firstValid);
end

% Smooth the periodic minimum-height envelope, then lift the ENTIRE curve
% until no sample is below its local minimum + clearance. This rounds
% the sharp cusps at 60, 180 and 300 degrees without invalidating IK.
zRequired=minimumZ+g.ZsafetyMarginCm;
zSmoothed=periodicGaussian(zRequired,g.ZsmoothingSigmaDeg/azStep);
missingLift=max(0,max(zRequired-zSmoothed));
zPath=zSmoothed+missingLift+g.additionalLiftCm;

% Validate the COMPLETE 360-degree path, including the wrap to 360.
% A tiny additional uniform lift is allowed if a sampled transition fails.
% Do not remove checks or skip invalid poses.
valid=false;
for attempt=1:g.maxExtraLiftIterations
    if max(zPath)>zMax+1e-10
        error('Smoothed Z requires %.3f cm, above Z_MAX %.3f cm.',...
            max(zPath),zMax);
    end
    valid=true;
    for k=1:n
        j=mod(k,n)+1;
        for t=[0 0.2 0.4 0.6 0.8 1]
            azInterp=az(k)+azStep*t;
            zInterp=(1-t)*zPath(k)+t*zPath(j);
            p=solvePose(g,tiltDeg,azInterp,zInterp);
            if ~p.valid
                valid=false;
                break;
            end
        end
        if ~valid,break;end
    end
    if valid,break;end
    zPath=zPath+0.02;
end
if ~valid
    error('Unable to obtain a valid smooth Z path under these limits.');
end

plan.azimuthDeg=[az 360];
plan.heightCm=[zPath zPath(1)];
plan.minFeasibleHeightCm=[minimumZ minimumZ(1)];
plan.tiltDeg=tiltDeg;
plan.thetaDownDeg=NaN(n+1,6);
plan.thetaUpDeg=NaN(n+1,6);
plan.relativeToSwitchDeg=NaN(n+1,6);
plan.maxRodErrorCm=0;
for k=1:n+1
    p=solvePose(g,tiltDeg,plan.azimuthDeg(k),plan.heightCm(k));
    assert(p.valid,'Invalid planned pose at waypoint %d: %s',k,p.reason);
    plan.thetaDownDeg(k,:)=p.thetaDownDeg;
    plan.thetaUpDeg(k,:)=p.thetaUpDeg;
    plan.relativeToSwitchDeg(k,:)=p.relativeToSwitchDeg;
    plan.maxRodErrorCm=max(plan.maxRodErrorCm,max(abs(p.rodErrorCm)));
end
plan.maxAdjacentHeightChangeCm=max(abs(diff(plan.heightCm)));
plan.maxAdjacentMotorAngleChangeDeg=max(max(abs(diff(plan.thetaDownDeg))));
end
