% MAIN ParameterSweep

%%% User inputs
MurphyRange_xy = [0.8,1.2];
MurphyRange_z = [0.5,1,2];
phaseRange = [0 0.25 0.5 0.75];
nSpokes = 12;
slope = -3;
modelBuildOptions = struct('simulate',false,'visualizeModel',false);
visualizeSolutions = false;

%%% Generate matrices to sweep

[MurphyXYMat,MurphyZMat,phaseMat] = meshgrid(MurphyRange_xy,MurphyRange_z,phaseRange);

szMat = size(MurphyXYMat);
nSims = prod(szMat);

[uMat,flagMat] = deal(NaN(szMat));

for i = 1:nSims
    
    Murphy = [MurphyXYMat(i)*[1 1], MurphyZMat(i)];
    phase = phaseMat(i);
    
    fName = buildDoubleRW(Murphy,phase,nSpokes,slope,modelBuildOptions);
    
    [solution,flag] = findLimitCycleDoubleRW(fName,nSpokes,visualizeSolutions);
    
    T = solution.getFinalTime();
    Trunk_tx = solution.getStateMat('/jointset/TrunkToGround/Trunk_tx/value');
    D = diff(Trunk_tx([1,end]));
    uMat(i) = D/T;
    flagMat(i) = flag;
end

save(['modelsAndResults/ParameterSweep',date_prefix('yyyymmdd-HHMM'),'.mat'],'*Mat','*Range*','nSpokes','slope')