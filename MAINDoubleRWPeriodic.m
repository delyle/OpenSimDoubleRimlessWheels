% MAINRimlessWheelPeriodic
clear
modelName = 'DoubleRW6_M10p00-10p00-1p60_RL30_FH15';
resultsDir = 'modelsAndResults';

fName = [resultsDir,'/',modelName,'/',modelName,'.osim'];


% set default coordinate values to initial values of period in a planar
% forward simulation
finalAngle = pi/3;

DoubleRWMakePlanarGuess(fName,finalAngle);

DoubleRWPeriodicMoco(fName,-finalAngle)