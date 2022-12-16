% MAINRimlessWheelPeriodic
clear
modelName = 'DoubleRW12_M0p50-0p50-0p50_RL15_FH8';
resultsDir = 'modelsAndResults';

fName = [resultsDir,'/',modelName,'/',modelName,'.osim'];


% set default coordinate values to initial values of period in a planar
% forward simulation
nSpokes = 12;
finalAngle = 2*pi/nSpokes;

simData = DoubleRWMakePlanarGuess(fName,finalAngle);

Xvel = simData.data.Trunk_tx_speed;
Xpos = simData.data.Trunk_tx_value;
maxSpeed = max(Xvel);
strideLength = diff(Xpos([1,end]));

bounds.TimeFinal = [0.1 2]*simData.data.time(end);
bounds.Trunk_tx_speed = [0 1.2*maxSpeed];
bounds.Trunk_tx_value = [0 1.2*strideLength];

settings = struct('meshIntervals',20,'guess','planar','constraintTolerance',1e-2);

[flag,solution] = DoubleRWPeriodicMoco(fName,-finalAngle,bounds,settings);

if flag ~= 1
    settings = struct('meshIntervals',40,'guess','3Dsolution');
    [flag,solution] = DoubleRWPeriodicMoco(fName,-finalAngle,bounds,settings);
end

T = solution.getFinalTime();
Trunk_tx = solution.getStateMat('/jointset/TrunkToGround/Trunk_tx/value');
D = diff(Trunk_tx([1,end]));
U = D/T