function DoubleRWPeriodicMoco(fName,finalAngle)
import org.opensim.modeling.*

osimModel = Model(fName);

% Create a MocoStudy
study = MocoStudy();
study.setName('PeriodicDoubleRW');
problem = study.updProblem();% Define the OCP
problem.setModel(osimModel);

% Specify bounds
% time starts at 0, can go up to 1
problem.setTimeBounds(MocoInitialBounds(0.),MocoFinalBounds(0.1, 1));

t2g = '/jointset/TrunkToGround/Trunk_';
problem.setStateInfo([t2g,'tx/value'],[0 5],[0],[0 5]);
%problem.setStateInfo('/jointset/PelvisToGround/Pelvis_tx/speed',[0 5],[0 5],[0 5]);
problem.setStateInfo('/jointset/lHind1ToTrunk/lHind1_rz/value',sort([0 finalAngle]),0,finalAngle);


% could put in bounds for other states...
%problem.setStateInfo('/jointset/PelvisToGround/Pelvis_rx/value',pi/3*[-1 1],pi/3*[-1 1],pi/3*[-1 1]);


% Cost, minimize periodicity residuals

periodicityGoal = MocoPeriodicityGoal('periodicityGoal');
periodicityGoal.setMode('cost');
problem.addGoal(periodicityGoal);

periodicCoordList = {'rx','ry','ty','tz'};
for iRange = 1:length(periodicCoordList)
    c = [t2g,periodicCoordList{iRange},'/value'];
    dc = [t2g,periodicCoordList{iRange},'/speed'];
    periodicityGoal.addStatePair(MocoPeriodicityGoalPair(c,c));
    periodicityGoal.addStatePair(MocoPeriodicityGoalPair(dc,dc));
end
% add periodicity for speed
periodicityGoal.addStatePair(MocoPeriodicityGoalPair([t2g,'tx','/speed']));
periodicityGoal.addStatePair(MocoPeriodicityGoalPair([t2g,'rz','/speed']));


% Configure Solver
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(20);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-3);
fName_prefix = strrep(fName,'.osim','');
solver.setGuessFile([fName_prefix,'_planarCycle.sto'])

solution = study.solve();

solution.unseal();
solution.write([fName_prefix,'_3Dcycle.sto']);

study.visualize(solution);