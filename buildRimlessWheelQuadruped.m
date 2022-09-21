% Quadrupedal rimless wheel

blankSlate

%% Import OSim Libraries
import org.opensim.modeling.*

%% Key Model Variables
modelNamePrefix = '3DRimlessWheelQuadruped';
murphy_x = 40;
murphy_y = 40;
murphy_z = 3;
legLength = 0.50;
legWidth = 0.05;
trunkMass = 10;
trunkLength = 1.5;
trunkWidth = 0.5;
trunkDepth = 0.125;
contactSphereRadius = 0.025;
rampHeightOffset = 5;
trunkColor = [255,10,10]/256;

nRightLegs = 6;
nLeftLegs = 6;
angleOffsetRight = 30;
angleOffsetLeft = 0;
angleOffsetForeToHind = 15;
hipPosX = -trunkLength/2;
shoulderPosX = trunkLength/2;

trunkMOI = [murphy_x*trunkWidth^2,murphy_y*trunkWidth^2,murphy_z*trunkLength^2]*trunkMass/4;

initialSpeed = -3;
rampAngle = -8;

% Define Contact Force Parameters
stiffness           = 1000000;
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
useVis = false;

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
osimModel.setGravity(Vec3(0, -9.81,0));

%% Construct bodies and joints
%% Construct Platform

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
platform_rz.setDefaultValue(deg2rad(rampAngle));
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
groundContactSpace.setName('PlatformContact');
osimModel.addContactGeometry(groundContactSpace);

%% Make and add a trunk Body
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

% Make and add a free joint for the Pelvis Body
trunkToPlatform = FreeJoint('trunkToPlatform', platform, trunk);
%%
% Update the coordinates of the new joint
trunk_rx = trunkToPlatform.upd_coordinates(0); % Rotation about x
trunk_rx.setRange([-pi, pi]);
trunk_rx.setName('Trunk_rx');
trunk_rx.setDefaultValue(0);

trunk_ry = trunkToPlatform.upd_coordinates(1); % Rotation about y
trunk_ry.setRange([-pi, pi]);
trunk_ry.setName('Trunk_ry');
trunk_ry.setDefaultValue(0);

trunk_rz = trunkToPlatform.upd_coordinates(2); % Rotation about z
trunk_rz.setRange([-pi, pi]);
trunk_rz.setName('Trunk_rz');
trunk_rz.setDefaultValue(0);

trunk_tx = trunkToPlatform.upd_coordinates(3); % Translation about x
trunk_tx.setRange([-10, 10]);
trunk_tx.setName('Trunk_tx');
trunk_tx.setDefaultValue(-10);
trunk_tx.setDefaultSpeedValue(initialSpeed)
 
trunk_ty = trunkToPlatform.upd_coordinates(4); % Translation about y
trunk_ty.setRange([0,5]);
trunk_ty.setName('Trunk_ty');
trunk_ty.setDefaultValue(legLength+0.1);
trunk_ty.setDefaultSpeedValue(0)

trunk_tz = trunkToPlatform.upd_coordinates(5); % Translation along z
trunk_tz.setRange([-5,5]);
trunk_tz.setName('Trunk_tz');
trunk_tz.setDefaultValue(0);
trunk_tz.setDefaultSpeedValue(0)

% Add Joint to model
osimModel.addJoint(trunkToPlatform)

%% Add Hind Legs

[LegS, ContactS, ForceS] = deal(struct);

% Make and add a Right Hind legs
nLegs = nRightLegs;
angleOffset = angleOffsetRight;
angleOffsetToPinLeg = 0;
legZOffset = 0;
legAngle = 360/nLegs;
trunkToLegPosZ = trunkWidth/2;
trunkToLegPosX = hipPosX;
sidePrefix = 'rHind';
newLegSet = true;
RimlessWheelQuad_addHindLegs

% Make and add Left Hind legs
nLegs = nLeftLegs;
legAngle = 360/nLegs;
angleOffsetToPinLeg = angleOffsetLeft-angleOffsetRight;
legZOffset = -trunkWidth;
sidePrefix = 'lHind';
RimlessWheelQuad_addHindLegs

%% Add Fore Legs

% Make and add Right Fore legs
nLegs = nRightLegs;
angleOffset = angleOffsetRight;
angleOffsetToPinLeg = 0;
legZOffset = 0;
legAngle = 360/nLegs;
trunkToLegPosZ = trunkWidth/2;
trunkToLegPosX = shoulderPosX;
sidePrefix = 'rFore';
newLegSet = true;
RimlessWheelQuad_addHindLegs

% Make and add Left Fore legs
nLegs = nLeftLegs;
legAngle = 360/nLegs;
angleOffsetToPinLeg = angleOffsetLeft-angleOffsetRight;
legZOffset = -trunkWidth;
sidePrefix = 'lFore';
RimlessWheelQuad_addHindLegs

%% Add coordinate coupler constraint

legPhaseConstraint = CoordinateCouplerConstraint();
%legPhaseConstraint = legPhaseConstraint.safeDownCast(legPhaseConstraint)

independentCoords = ArrayStr();
independentCoords.append('rHind1_rz');
legPhaseConstraint.setIndependentCoordinateNames(independentCoords);
legPhaseConstraint.setDependentCoordinateName('rFore1_rz');
coefficients = Vector(3,0);
coefficients.set(0,1);
legPhaseConstraint.setFunction(LinearFunction(1,deg2rad(angleOffsetForeToHind)))
osimModel.addConstraint(legPhaseConstraint);


%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
fname = [modelName,'.osim'];
osimModel.print(fname);
disp([fname,' printed!']);

%% Run simulation
if simulate
    options = struct('endTime',endTime,'stepSize',0.001,'useVis',useVis);
    simData = SimulateQuadrupedRimlessWheel(fname,options,[]);
end

