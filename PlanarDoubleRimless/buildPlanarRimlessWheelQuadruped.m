% Quadrupedal rimless wheel

blankSlate

%% Import OSim Libraries
import org.opensim.modeling.*

%% Key Model Variables
modelNamePrefix = 'PlanarDoubleRW';
resultsDir = 'modelsAndResults';
reorientGravity = true; % if true, there is no platform joint. Gravity is reoriented according to the platform angle.

murphy_x = 10;
murphy_y = 10;
murphy_z = 1.6;
legLength = 0.50;
legWidth = 0.05;
legMass = 0.5;
trunkMass = 20;
trunkLength = 1.5;
trunkWidth = 0.5;
trunkDepth = 0.125;
contactSphereRadius = 0.025;
rampInitialAngle = -2; % negative angles point the normal force along the x axis
rampHeightOffset = 5;
trunkColor = [255,10,10]/256;

nRightLegs = 6;
nLeftLegs = 6;
angleOffsetRight = 30;
angleOffsetLeft = 0;
angleOffsetForeToHind = 15;
hipPosX = -trunkLength/2;
shoulderPosX = trunkLength/2;
constrainFHPhase = true;

trunkMOI = [murphy_x*trunkWidth^2,murphy_y*trunkWidth^2,murphy_z*trunkLength^2]*trunkMass/4;

initialSpeed = 0.2;

% Define Contact Force Parameters
stiffness           = 2e6;
dissipation         = 2.0;
staticFriction      = 2;
dynamicFriction     = 0.8;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;
% halve leg length, because of how cylinders are built in opensim
cylLength = legLength/2;

% whether to run a simulation
simulate = true;
endTime = 15;
visualizeSim = true;
visualizeModel = true;
%% intantiate an empty OpenSim Model
angleOffsetRightToLeft = angleOffsetRight - angleOffsetLeft;
modelNamePostfix = sprintf('%i_M%.2f-%.2f-%.2f_RL%.0f_FH%.0f',nRightLegs,murphy_x,murphy_y,murphy_z,angleOffsetRightToLeft,angleOffsetForeToHind);
modelNamePostfix = strrep(modelNamePostfix,'.','p');
modelName = [modelNamePrefix,modelNamePostfix];

osimModel = Model();
osimModel.setName(modelName)

% Get a reference to the ground object
ground = osimModel.getGround();

% define acceleration of gravity
if reorientGravity
    a = deg2rad(rampInitialAngle);
    gvec = -9.81*[sin(a),cos(a),0];
    osimModel.setGravity(osimVec3FromArray(gvec));
    
    % Make a contact Half space for the ground
    groundContactLocation = Vec3(0,0,0);
    groundContactOrientation = Vec3(0,0,-pi/2);
    groundContactSpace = ContactHalfSpace(groundContactLocation,groundContactOrientation,ground);
    contactSpaceName = 'GroundContact';
    groundContactSpace.setName(contactSpaceName);
    osimModel.addContactGeometry(groundContactSpace); 
else
    osimModel.setGravity(Vec3(0, -9.81,0));

    % Construct Platform

    platform = Body();
    platform.setName('Platform');
    platform.setMass(1);
    platform.setInertia( Inertia(1,1,1,0,0,0) );

    % Add geometry to the body
    platformGeometry = Brick(Vec3(10,0.01,1));
    platformColor = 153/256;
    platformGeometry.setColor(Vec3(platformColor)); % a smokey black
    platform.attachGeometry(platformGeometry);


    % Add Body to the Model
    osimModel.addBody(platform);

    % Section: Create the Platform Joint
    % Make and add a Pin joint for the Platform Body
    locationInParent    = Vec3(0,0,0);
    orientationInParent = Vec3(0,0,0);
    locationInChild     = Vec3(0,0,0);
    orientationInChild  = Vec3(0,0,0);
    platformToGround    = PinJoint('PlatformToGround',...  % Joint Name
                                    ground,...             % Parent Frame
                                    locationInParent,...   % Translation in Parent Frame
                                    orientationInParent,...% Orientation in Parent Frame
                                    platform,...           % Child Frame
                                    locationInChild,...    % Translation in Child Frame
                                    orientationInChild);   % Orientation in Child Frame

    % Update the coordinates of the new joint
    platform_rz = platformToGround.upd_coordinates(0);
    platform_rz.setRange([deg2rad(-100), deg2rad(100)]);
    platform_rz.setName('platform_rz');
    platform_rz.setDefaultValue(deg2rad(rampInitialAngle));
    platform_rz.setDefaultSpeedValue(0);
    platform_rz.setDefaultLocked(true) % important to ensure rigid surface

    % Add Joint to the Model
    osimModel.addJoint(platformToGround);

    % Make a Contact Half Space
    groundContactLocation = Vec3(0,0.025,0);
    groundContactOrientation = Vec3(0,0,-1.57);
    groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                           groundContactOrientation,...
                                           platform);
    contactSpaceName  = 'PlatformContact'; % needed for the addLegs script
    groundContactSpace.setName(contactSpaceName);

    osimModel.addContactGeometry(groundContactSpace);
end

%% Make and add a trunk Body
import org.opensim.modeling.*
trunk = Body();
trunk.setName('Trunk');
trunk.setMass(trunkMass);
trunk.setInertia(Inertia(trunkMOI(1),trunkMOI(2),trunkMOI(3),0,0,0));
% Add geometry for display
dispGeom = Brick(Vec3(trunkLength/2,trunkDepth/2,trunkWidth/2));
dispGeom.setColor(osimVec3FromArray(trunkColor));
trunk.attachGeometry(dispGeom);
% Add Body to the Model
osimModel.addBody(trunk);

% Make and add a planar joint for the Pelvis Body
import org.opensim.modeling.*
trunkToPlatform = PlanarJoint('TrunkToGround',ground,trunk);

%
% Update the coordinates of the new joint

trunk_rz = trunkToPlatform.upd_coordinates(0); % Rotation about z
trunk_rz.setRange([-pi, pi]);
trunk_rz.setDefaultValue(0);
trunk_rz.setName('Trunk_rz');

trunk_tx = trunkToPlatform.upd_coordinates(1); % Translation about x
trunk_tx.setRange([-10, 10]);
trunk_tx.setDefaultValue(-10*(~reorientGravity)); % zero if gravity is reoriented, -10 if on the ramp
trunk_tx.setDefaultSpeedValue(initialSpeed)
trunk_tx.setName('Trunk_tx'); 

trunk_ty = trunkToPlatform.upd_coordinates(2); % Translation about y
trunk_ty.setRange([0,5]);
trunk_ty.setDefaultValue(legLength+0.1);
trunk_ty.setDefaultSpeedValue(0)
trunk_ty.setName('Trunk_ty')

% Add Joint to model
osimModel.addJoint(trunkToPlatform)
%% Add Hind Legs

[LegS, ContactS, ForceS] = deal(struct);

% Make and add Left Hind legs
newLegSet = true;
nLegs = nLeftLegs;
angleOffsetToPinLeg = 0;
angleOffset = angleOffsetLeft;
legAngle = 360/nLegs;
trunkToLegPosZ = -trunkWidth/2;
trunkToLegPosX = hipPosX;
legZOffset = 0;
sidePrefix = 'lHind';
RimlessWheelQuad_addLegs

% Make and add a Right Hind legs
nLegs = nRightLegs;
angleOffsetToPinLeg = angleOffsetRight-angleOffsetLeft;
legZOffset = trunkWidth;
legAngle = 360/nLegs;
sidePrefix = 'rHind';
RimlessWheelQuad_addLegs

%% Add Fore Legs

% Make and add Left Fore legs
newLegSet = true;
nLegs = nLeftLegs;
legAngle = 360/nLegs;
angleOffset = angleOffsetLeft;
angleOffsetToPinLeg = 0;
legZOffset = 0;
trunkToLegPosZ = -trunkWidth/2;
trunkToLegPosX = shoulderPosX;
sidePrefix = 'lFore';
RimlessWheelQuad_addLegs

% Make and add Right Fore legs
nLegs = nRightLegs;
angleOffsetToPinLeg = angleOffsetRight-angleOffsetLeft;
legZOffset = trunkWidth;
legAngle = 360/nLegs;
sidePrefix = 'rFore';
RimlessWheelQuad_addLegs

%% Add coordinate coupler constraint

if constrainFHPhase
legPhaseConstraint = CoordinateCouplerConstraint();
%legPhaseConstraint = legPhaseConstraint.safeDownCast(legPhaseConstraint)

independentCoords = ArrayStr();
independentCoords.append('lHind1_rz');
legPhaseConstraint.setIndependentCoordinateNames(independentCoords);
legPhaseConstraint.setDependentCoordinateName('lFore1_rz');
coefficients = Vector(3,0);
coefficients.set(0,1);
legPhaseConstraint.setFunction(LinearFunction(1,deg2rad(angleOffsetForeToHind)))
osimModel.addConstraint(legPhaseConstraint);
end

%% Initialize the System (checks model consistency).
osimModel.initSystem();

saveDir = [resultsDir,'/',modelName,'/'];
mkdir(saveDir)

% Save the model to a file
fname = [saveDir,modelName,'.osim'];
osimModel.print(fname);
disp([fname,' printed!']);





%% Run simulation
if simulate
    options = struct('endTime',endTime,'stepSize',0.001,'reportInterval',0.01,...
    'useVis',visualizeSim);
    simData = RimlessWheelForwardSimulation(fname,'',[],options);
end

% show the model
if visualizeModel
    VisualizerUtilities().showModel(osimModel)
end
