% Quadruped Rimless compare
blankSlate

% fName1 = '3DRimlessWheelQuadruped6_M0p05-40p00-1p00_RL30_FH15_30s';
% fName2 = '3DRimlessWheelQuadruped6_M0p05-40p00-1p00_RL30_FH-15_30s';
fName1 = '3DRimlessWheelQuadruped6_M40p00-40p00-3p00_RL30_FH15_60s';
fName2 = '3DRimlessWheelQuadruped6_M40p00-40p00-3p00_RL30_FH0_60s';



A = load(fName1);
B = load(fName2);

figure('color','w')

subplot(2,1,1)
plot(A.simData.time,A.simData.Trunk_tx_speed)
i0 = find(A.simData.time > 15,1);
A.meanForwardSpeed = mean(A.simData.Trunk_tx_speed(i0:end));
hold on
plot(A.simData.time([1,end]),A.meanForwardSpeed*[1,1],'r-')
plot(A.simData.time([i0 i0]),[0 2],'k--')
text(A.simData.time(end),A.meanForwardSpeed,sprintf('%.2f m/s',A.meanForwardSpeed),'horizontalalignment','left')
ylim([0 3.0])
ylabel('Forward Speed (m/s)')
text(0.05,1,'lateral seq.','units','normalized','verticalalignment','top','horizontalalignment','left')
splt = strsplit(fName1,'_');
title(['Murphy xyz: [',strrep(strrep(splt{2}(2:end),'p','.'),'-',', '),'], ',splt{4},char(176)])

subplot(2,1,2)
plot(B.simData.time,B.simData.Trunk_tx_speed)
i0 = find(B.simData.time > 15,1);
B.meanForwardSpeed = mean(B.simData.Trunk_tx_speed(i0:end));
hold on
plot(B.simData.time([1,end]),B.meanForwardSpeed*[1,1],'r-')
plot(B.simData.time([i0 i0]),[0 2],'k--')
text(B.simData.time(end),B.meanForwardSpeed,sprintf('%.2f m/s',B.meanForwardSpeed),'horizontalalignment','left')
text(0.05,1,'diagonal seq.','units','normalized','verticalalignment','top','horizontalalignment','left')


ylim([0 3.0])
xlabel('Time (s)')
ylabel('Forward Speed (m/s)')
splt = strsplit(fName2,'_');
title(['Murphy xyz: [',strrep(strrep(splt{2}(2:end),'p','.'),'-',', '),'], ',splt{4},char(176)])
