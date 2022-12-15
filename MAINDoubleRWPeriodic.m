% MAINRimlessWheelPeriodic
clear
modelName = 'DoubleRW12_M10p00-10p00-1p60_RL15_FH8';
resultsDir = 'modelsAndResults';

fName = [resultsDir,'/',modelName,'/',modelName,'.osim'];


% set default coordinate values to initial values of period in a planar
% forward simulation
finalAngle = pi/6;

simData = DoubleRWMakePlanarGuess(fName,finalAngle);

Xvel = simData.data.Trunk_tx_speed;
Xpos = simData.data.Trunk_tx_value;
maxSpeed = max(Xvel);
strideLength = diff(Xpos([1,end]));

bounds.TimeFinal = [0.1 2*simData.data.time(end)];
bounds.Trunk_tx_speed = [0 2*maxSpeed];
bounds.Trunk_tx_value = [0 2*strideLength];

DoubleRWPeriodicMoco(fName,-finalAngle,bounds)