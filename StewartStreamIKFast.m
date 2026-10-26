function zMinimum = StewartStreamIKFast(g, tiltDeg, azimuthDeg, zMin, zMax, zStep)
% StewartStreamIKFast
% Vectorized six-axis Z feasibility search for the live Stewart controller.
% Same 0.05 cm (or supplied) search grid and angle conventions as baseline.
% Returns the first reachable Z or NaN if none is found.
% NO MOTOR OUTPUT. Validate before replacing any hardware controller function.

validateattributes(zMin, {'numeric'}, {'scalar','real','finite'});
validateattributes(zMax, {'numeric'}, {'scalar','real','finite','>',zMin});
validateattributes(zStep,{'numeric'}, {'scalar','real','finite','positive'});

zGrid = (zMin:zStep:zMax).';
if zGrid(end) < zMax - 1e-9
    zGrid(end+1,1) = zMax;
end

% Rotation matrix (exactly the same convention as StewartStreamIK).
u = [sind(azimuthDeg); -cosd(azimuthDeg); 0];
K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R = eye(3) + sind(tiltDeg)*K + (1-cosd(tiltDeg))*(K*K);
d = R*g.plateLocal - g.base;                 % 3 x 6

% All six motor swing-plane projections computed together (1 x 6).
x = sum(d .* g.swing, 1);
v = zGrid + d(3,:);                         % N x 6 via implicit expansion
rho = hypot(x,v);
fullSquared = sum(d(1:2,:).^2,1) + v.^2;
C = (fullSquared+g.Lshort^2-g.Llong^2)/(2*g.Lshort);

reachable = rho>1e-10 & abs(C)<=rho+1e-8;
alpha = acosd(max(-1,min(1,C./max(rho,1e-10))));
origin = atan2d(x,v);
downA = 180 - mod(origin+alpha,360);
downB = 180 - mod(origin-alpha,360);

% Per-axis safety bound works for identical 21-deg switches AND individually
% calibrated switch angles. Baseline uses switchAngleDownDeg(1) for all.
minAngle = max(g.minAngleDownDeg,g.switchAngleDownDeg(:).') + ...
    g.angleSafetyMarginDeg;
maxAngle = g.maxAngleDownDeg - g.angleSafetyMarginDeg;
valid = reachable & ((downA>=minAngle & downA<=maxAngle) | ...
                     (downB>=minAngle & downB<=maxAngle));
first = find(all(valid,2),1,'first');
if isempty(first)
    zMinimum = NaN;
else
    zMinimum = zGrid(first);
end
end
