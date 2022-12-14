% run a forward simulation

fName = 'modelsAndResults/DoubleRW6_M1p00-1p00-0p80_RL30_FH15/DoubleRW6_M1p00-1p00-0p80_RL30_FH15.osim';

d2r = pi/180;

options = struct('endTime',5,'stepSize',0.001,'reportInterval',0.01,...
    'useVis',true);
initCoords = {'Trunk_tx',0,0.2};

simData = RimlessWheelForwardSimulation(fName,'test',initCoords,options);