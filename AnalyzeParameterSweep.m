% Analyze parameter sweep

% organize by phase

I0 = phaseMat == 0;
I0p25 = phaseMat == 0.25;
I0p5 = phaseMat == 0.5;
I0p75 = phaseMat == 0.75;

plotUvsMz = @(I) plot(MurphyZMat(I),uMat(I));

subplot(3,1,1)
IMx = MurphyXYMat == 0.5;
plotUvsMz(IMx & I0);
hold on
plotUvsMz(IMx & I0p25);
plotUvsMz(IMx & I0p5);
plotUvsMz(IMx & I0p75);
legend({'0','0.25','0.5','0.75'},'location','northoutside','orientation','horizontal')

subplot(3,1,2)
IMx = MurphyXYMat == 1;
plotUvsMz(IMx & I0);
hold on
plotUvsMz(IMx & I0p25);
plotUvsMz(IMx & I0p5);
plotUvsMz(IMx & I0p75);

subplot(3,1,3)
IMx = MurphyXYMat == 2;
plotUvsMz(IMx & I0);
hold on
plotUvsMz(IMx & I0p25);
plotUvsMz(IMx & I0p5);
plotUvsMz(IMx & I0p75);
