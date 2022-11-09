function [simData,options] = SimulateQuadrupedRimlessWheel(fName,options,saveName)

% set default options
fieldNames = fieldnames(options);
neededFields = {'endTime','stepSize','integrator','accuracy','useVis'};

if nargin < 3 || isempty(saveName)
    saveName = [strrep(fName,'.osim',''),'_',num2str(options.endTime),'s'];
end



import org.opensim.modeling.*

osimModel = Model(fName);


% Set the visualizer use.
osimModel.setUseVisualizer(options.useVis);
osimModel.initSystem;
% Set the Vizualizer parameters
if options.useVis
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);
    % Set the default ground height down so that the walker platform is
    % viewable.
    sviz.setGroundHeight(-20);
    % Set the initial position of the camera
    sviz.setCameraTransform(Transform(Rotation(0.9,Vec3(0,1,0)),Vec3(16,1,8)));
end




%% Run a fwd simulation using the manager

% set up a ForceReporter
%forceReport = ForceReporter(osimModel);

% set up a StatesTrajectoryReporter
reporter = StatesTrajectoryReporter();
reporter.setName('reporter');
reporter.set_report_time_interval(0.01);
osimModel.addComponent(reporter);

% Initialize the underlying computational system and get a reference to
% the system state.
state = osimModel.initSystem();

htic = tic;
disp('Running forward tool')
manager = Manager(osimModel);
state.setTime(0);
manager.initialize(state);
manager.setIntegratorMinimumStepSize(options.stepSize)
state = manager.integrate(options.endTime);
htoc = toc(htic);

fprintf('Forward Tool Finished in %.1f s\n',htoc);
%% Get the states table from the manager and print the results.
simData = struct;
if saveName
%fTable = forceReport.getForcesTable(); % doesn't work...

tic;
disp('Getting states from trajectory reporter')
statesTraj = reporter.getStates();
sTable = statesTraj.exportToTable(osimModel);
fprintf('Took %.4f s\n',toc)


stofiles = STOFileAdapter();
fprintf('Writing table to %s\n',[saveName,'.sto'])
stofiles.write(sTable, [saveName,'.sto']);

% rewrite first line from "states" to something more informative
newHeaderName = ['states',strrep(saveName,'3DRimlessWheelQuadruped','')];
rewriteLine([saveName,'.sto'],newHeaderName,1);

%% Get the Data in matlab format
fprintf('Writing Table to Matlab Structure\n')
tic;
simData = osimTableToStruct(sTable);
fprintf('Took %.4f s\n',toc)

% rename the data to a more readable format

fieldNames = fieldnames(simData);

fieldNames(strcmp('time',fieldNames)) = [];

for i = 1:length(fieldNames)
    oldField = fieldNames{i};
    splt = strsplit(oldField,'_');
    newField = strjoin(splt(end-2:end),'_');
    simData.(newField) = simData.(oldField);
    simData = rmfield(simData,oldField);
end

simData.SimulationTime = htoc;

save([saveName,'.mat'],'simData')


disp(['Output files written to ',saveName])
end

end

