% MAIN ParameterSweep

Murphy = [0.5 0.5 0.5];
phase = 0.25;
nSpokes = 12;
slope = -3;
options = struct('simulate',true);

fName = buildDoubleRW(Murphy,phase,nSpokes,slope,options);

%%

solution = findLimitCycleDoubleRW(fName,nSpokes);

T = solution.getFinalTime();
Trunk_tx = solution.getStateMat('/jointset/TrunkToGround/Trunk_tx/value');
D = diff(Trunk_tx([1,end]));
U = D/T