function simData = DoubleRWMakePlanarGuess(fName,targetAngle,fNameNew)
if nargin < 3 
    fNameNew = fName;
end



OPC = strcat('Trunk_',{'rx','ry','tz'}); % off-planar coordinates

% run a feedforward simulation

options = struct('endTime',5,'stepSize',0.001,'reportInterval',0.01,...
    'useVis',false,'lockedCoords',{OPC});
initCoords = {'Trunk_tx',0,1};

simData = RimlessWheelForwardSimulation(fName,false,initCoords,options);

Data = simData.data;
% find last instance of vertical leg

t = Data.time;
X = Data.lHind1_rz_value;
Y = Data.lFore1_rz_value;

modX = mod(X,targetAngle);
dmodX = diff(modX)./diff(t);
iTarget = find(abs(dmodX) > 10,2,'last'); % finds rapid changes in modX
% this rapid change indicates that Pelvis_rz has crossed the targetAngle 
% Returns the final two changes; the last full cycle in the simulation

% reset key variables such that start of cycle is 0
reset0 = @(x,i) x - x(i);
Data.time = reset0(t,iTarget(1));
Data.lHind1_rz_value = reset0(X,iTarget(1));
Data.Trunk_tx_value = reset0(Data.Trunk_tx_value,iTarget(1));

% set forelimb variables to be within 0-360 deg
% note: doing a modulo(X,2pi) often results in a 2pi step change, which we absolutely do not want
% instead, we can keep the same offset between fore and hind limbs,
% bringing the forelimb angle range closer to the hindlimb range.
FHdiff = Y-X;
Data.lFore1_rz_value = Data.lHind1_rz_value+FHdiff;

% set model defaults to values at iTarget
import org.opensim.modeling.*

model = Model(fName);
coordSet = model.updCoordinateSet;
n = coordSet.getSize;
for i = 0:n-1
    coordName = char(coordSet.get(i));
    coordSet.get(i).setDefaultValue(Data.([coordName,'_value'])(iTarget(1)))
    coordSet.get(i).setDefaultSpeedValue(Data.([coordName,'_speed'])(iTarget(1)))
end

model.initSystem();

% save the model to a file
model.print(fNameNew);
disp([fNameNew,' printed with new defaults coordinate values and speeds']);

% save data only over the cyclical guess
fields = fieldnames(Data);
for i  = 1:length(fields)
   Data.(fields{i}) = Data.(fields{i})(iTarget(1):iTarget(2));
end

% overwrite simData.data for conversion to .sto
simData.data = Data;

% get number of time points
nrows = size(Data,1);

% add lambda and gamma fields
lName = 'lambda_cid8_p0'; % multiplier
gName = 'gamma_cid8_p0'; % slack

simData.data.(lName) =zeros(nrows,1);
simData.columnLabels.(lName) = lName;
simData.data.(gName) =zeros(nrows,1);
simData.columnLabels.(gName) = gName;

% write header to be moco-compatible
mocoHeader = [...
    "inDegrees=no";
    "num_controls=0";
    "num_derivatives=0";
    "num_multipliers=1";
    "num_parameters=0";
    "num_slacks=1";
    "num_states=16";
    "DataType=double";
    "version=3";
    string(['OpenSimVersion=',char(opensimCommon.GetVersion())])];
simDataToMocoSTO(strrep(fName,'.osim','_planarCycle.sto'),simData,mocoHeader)

fprintf('Done\n')


