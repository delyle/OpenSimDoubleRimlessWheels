% MAINRimlessWheelPeriodic
clear
modelName = 'DoubleRW12_M10p00-10p00-1p60_RL15_FH8';
resultsDir = 'modelsAndResults';

fName = [resultsDir,'/',modelName,'/',modelName,'.osim'];


% set default coordinate values to initial values of period in a planar
% forward simulation
finalAngle = pi/6;

DoubleRWMakePlanarGuess(fName,finalAngle);

DoubleRWPeriodicMoco(fName,-finalAngle)