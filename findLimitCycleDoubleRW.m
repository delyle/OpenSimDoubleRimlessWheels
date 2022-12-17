function [solution,flag] = findLimitCycleDoubleRW(fName,nSpokes,visualize)
if nargin < 3
    visualize = true;
end

% set default coordinate values to initial values of period in a planar
% forward simulation
finalAngle = 2*pi/nSpokes;

simData = DoubleRWMakePlanarGuess(fName,finalAngle);

Xvel = simData.data.Trunk_tx_speed;
Xpos = simData.data.Trunk_tx_value;
maxSpeed = max(Xvel);
strideLength = diff(Xpos([1,end]));

bounds.TimeFinal = [0.1 2]*simData.data.time(end);
bounds.Trunk_tx_speed = [0 1.2*maxSpeed];
bounds.Trunk_tx_value = [0 1.2*strideLength];

settings = struct('meshIntervals',30,'guess','planar','constraintTolerance',1e-2,'visualize',visualize);

[flag,solution] = DoubleRWPeriodicMoco(fName,-finalAngle,bounds,settings);

if flag ~= 1
    settings = struct('meshIntervals',40,'guess','3Dsolution','constraintTolerance',1e-2,'visualize',visualize);
    [flag,solution] = DoubleRWPeriodicMoco(fName,-finalAngle,bounds,settings);
end

