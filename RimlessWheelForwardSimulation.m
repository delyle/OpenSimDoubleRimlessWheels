function [simData,options] = RimlessWheelForwardSimulation(fName,saveName,initCoords,options)

if nargin < 4
    % initialize options as a blank struct. It will be filled with defaults
    options = struct();
    if nargin < 3
        initCoords = [];
        if nargin < 2 || isempty(saveName)
            saveName = [strrep(fName,'.osim',''),'_',num2str(options.endTime),'s'];
        end
    end
end

% set default options
fieldNames = fieldnames(options);
reqFieldsAndDefaults = {'endTime',5,'stepSize',0.001,'integrator',1,...
    'accuracy',0.001,'useVis',false,'reportInterval',0.01,'lockedCoords',{}};
reqFields = reqFieldsAndDefaults(1:2:end);
reqDefaults = reqFieldsAndDefaults(2:2:end);

for i = find(~ismember(reqFields,fieldNames))
   options.(reqFields{i}) = reqDefaults{i};
end

import org.opensim.modeling.*

osimModel = Model(fName);

% Set the visualizer use.
osimModel.setUseVisualizer(options.useVis);

% set up a StatesTrajectoryReporter
reporter = StatesTrajectoryReporter();
reporter.setName('reporter');
reporter.set_report_time_interval(options.reportInterval);
osimModel.addComponent(reporter);

if ~isempty(options.lockedCoords)
    coordSet = osimModel.updCoordinateSet();
    for i = 1:length(options.lockedCoords)
       coordSet.get(options.lockedCoords{i}).setDefaultLocked(true); 
    end
end

% Initialize the underlying computational system and get a reference to
% the system state.
state = osimModel.initSystem();

% initialize the coordinates

if ~isempty(initCoords)
    if exist('coordSet','var') ~= 1
        coordSet = osimModel.updCoordinateSet();
    end
    coordNames = initCoords(1:3:end);
    coordValues = initCoords(2:3:end);
    coordSpeeds = initCoords(3:3:end);
    for i = 1:length(coordNames)
        coordSet.get(coordNames{i}).setValue(state, coordValues{i});
        coordSet.get(coordNames{i}).setSpeedValue(state, coordSpeeds{i});
    end
end



% Set the Vizualizer parameters
if options.useVis
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);
    % Set the default ground height down so that the walker platform is
    % viewable.
    %sviz.setGroundHeight(-20);
    % Set the initial position of the camera
    sviz.setCameraTransform(Transform(Rotation(0.9,Vec3(0,1,0)),Vec3(16,1,8)));
end


%% Run a fwd simulation using the manager

% set up a ForceReporter
%forceReport = ForceReporter(osimModel);



% Initialize the underlying computational system and get a reference to
% the system state.
%state = osimModel.initSystem();

htic = tic;
fprintf('Running forward tool... ')
manager = Manager(osimModel);
state.setTime(0);
manager.initialize(state);
manager.setIntegratorMinimumStepSize(options.stepSize)
state = manager.integrate(options.endTime);
htoc = toc(htic);

fprintf('Took %.1f s\n',htoc);
% Get the states table from the manager and print the results.

%fTable = forceReport.getForcesTable(); % doesn't work...

tic;
fprintf('Getting states from trajectory reporter... ')
statesTraj = reporter.getStates();
sTable = statesTraj.exportToTable(osimModel);
fprintf('Took %.4f s\n',toc)

%% Write results to .sto
if saveName
stofiles = STOFileAdapter();
fprintf('Writing table to %s... ',[saveName,'.sto'])
stofiles.write(sTable, [saveName,'.sto']);

% rewrite first line from "states" to something more informative
newHeaderName = ['states',saveName];
rewriteLine([saveName,'.sto'],newHeaderName,1);
fprintf('Done\n');
end

%% Convert the Data to matlab format

% first get table column labels; these will be in a format like
% '/jointset/joint/coordname/value'
% which is a bad format for osimTableToStruct and will generate pesky warnings.
% The code below will convert the labels to 'coordname_value', and save the
% old names to the data struct

simData = struct();
nLabels = sTable.getNumColumns();
for i = 0:nLabels-1
    curLabel = char(sTable.getColumnLabel(i));
    labelParts = strsplit(curLabel,'/');
    newLabel = strjoin(labelParts(end-1:end),'_');
    sTable.setColumnLabel(i,newLabel);
    
    simData.columnLabels.(newLabel) = curLabel;
end

fprintf('Writing Table to Matlab Structure... ')
tic;
simData.data = osimTableToStruct(sTable);
fprintf('Took %.4f s\n',toc)

simData.SimulationTime = htoc;

if saveName
    % same a .mat file
    save([saveName,'.mat'],'simData')
    disp(['Output files written to ',saveName])
end



