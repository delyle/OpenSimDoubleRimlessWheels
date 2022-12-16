function [flag,solution] = DoubleRWPeriodicMoco(fName,finalAngle,bounds,settings)
import org.opensim.modeling.*

% set default bounds
boundsDefault = struct('TimeFinal',[0.1 1],'Trunk_tx_value',[0 1],'Trunk_tx_speed',[0 2]);
settingsDefault = struct('meshIntervals',20,'guess','planar','constraintTolerance',1e-2);

if nargin < 4
    settings = settingsDefault;
    if nargin < 3
        bounds = boundsDefault;
    end
end
if isempty(bounds)
    bounds = boundsDefault;
end

bounds = fillDefaults(bounds,boundsDefault);
settings = fillDefaults(settings, settingsDefault);

osimModel = Model(fName);

% Create a MocoStudy
study = MocoStudy();
study.setName('PeriodicDoubleRW');
problem = study.updProblem();% Define the OCP
problem.setModel(osimModel);

% Specify bounds
% time starts at 0, can go up to set amount
problem.setTimeBounds(MocoInitialBounds(0),MocoFinalBounds(bounds.TimeFinal(1), bounds.TimeFinal(2)));

t2g = '/jointset/TrunkToGround/Trunk_';
problem.setStateInfo([t2g,'tx/value'],bounds.Trunk_tx_value,bounds.Trunk_tx_value(1),bounds.Trunk_tx_value); 
%problem.setStateInfo([t2g,'ty/value'],[0.45 1],[0.45 1],[0.45 1]); % with leg length of 0.5, this is more than reasonable
problem.setStateInfo([t2g,'tx/speed'],bounds.Trunk_tx_speed,bounds.Trunk_tx_speed,bounds.Trunk_tx_speed);
problem.setStateInfo('/jointset/lHind1ToTrunk/lHind1_rz/value',sort([0 finalAngle]),0,finalAngle);


% Put in bounds for other states...
problem.setStateInfo([t2g,'rx/value'],pi/6*[-1 1],pi/6*[-1 1],pi/6*[-1 1]);
problem.setStateInfo([t2g,'ry/value'],pi/6*[-1 1],pi/6*[-1 1],pi/6*[-1 1]);
problem.setStateInfo([t2g,'rz/value'],pi/6*[-1 1],pi/6*[-1 1],pi/6*[-1 1]);


% ensure gait periodicity
periodicityGoal = MocoPeriodicityGoal('periodicityGoal');
periodicityGoal.setMode('endpoint_constraint');
problem.addGoal(periodicityGoal);

periodicCoordList = {'rx','ry','rz','ty','tz'};
for iRange = 1:length(periodicCoordList)
    c = [t2g,periodicCoordList{iRange},'/value'];
    dc = [t2g,periodicCoordList{iRange},'/speed'];
    periodicityGoal.addStatePair(MocoPeriodicityGoalPair(c,c));
    periodicityGoal.addStatePair(MocoPeriodicityGoalPair(dc,dc));
end
% add periodicity for speed
periodicityGoal.addStatePair(MocoPeriodicityGoalPair([t2g,'tx','/speed']));
periodicityGoal.addStatePair(MocoPeriodicityGoalPair(['/jointset/lHind1ToTrunk/lHind1_rz','/speed']));
%periodicityGoal.addStatePair(MocoPeriodicityGoalPair(['/jointset/lFore1ToTrunk/lFore1_rz','/speed']));


% Configure Solver
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(settings.meshIntervals);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(settings.constraintTolerance);
fName_prefix = strrep(fName,'.osim','');
switch lower(settings.guess)
    case 'planar'
        solver.setGuessFile([fName_prefix,'_planarCycle.sto'])
    case '3dsolution'
        solver.setGuessFile([fName_prefix,'_3DCycle.sto'])
    otherwise
        error('settings.guess not recognized. Accepted values: ''planar'' or ''3Dsolution''')
end
%solver.set_optim_hessian_approximation('exact');

solution = study.solve();

solution.unseal();
solution.write([fName_prefix,'_3DCycle.sto']);
disp(['Solution written to ',fName_prefix,'_3Dcycle.sto'])

flag = solution.success();
disp(solution.getStatus());

study.visualize(solution);