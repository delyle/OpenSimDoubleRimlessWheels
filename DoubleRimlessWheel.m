% Build a rimless wheel model

blankSlate

%% Import OSim Libraries
import org.opensim.modeling.*

%% Define key model variables
modelName = 'RimlessWheel';
pelvisWidth = 0.20;
legLength = 0.50;
legWidth = 0.05;
torsoLength = 1.5;
contactSphereRadius = 0.05;
rampHeightOffset = 5;

nRightLegs = 3;
nLeftLegs = 4;
angleOffsetRight = 0;
angleOffsetLeft = 0;

% Define Contact Force Parameters
stiffness           = 1000000;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;


%% intantiate an empty OpenSim Model
osimModel = Model();
osimModel.setName(modelName)

% Get a reference to the ground object
ground = osimModel.getGround();

% define acceleration of gravity
osimModel.setGravity(Vec3(0, -9.81,0));
%% Construct bodies and joints
platform = Body();
platform.setName('Platform');
platform.setMass(1);
platform.setInertia( Inertia(1,1,1,0,0,0) );

% Add geometry to the body
platformGeometry = Brick(Vec3(10,0.01,1));
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
platform_rz.setDefaultValue(deg2rad(-10));
platform_rz.setDefaultSpeedValue(0);
platform_rz.setDefaultLocked(true)

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
%%
% Make and add a Pelvis Body
pelvis = Body();
pelvis.setName('Pelvis');
pelvis.setMass(1);
pelvis.setInertia(Inertia(1,1,1,0,0,0));
% Add geometry for display
pelvis.attachGeometry(Sphere(pelvisWidth));
% Add Body to the Model
osimModel.addBody(pelvis);

% Make and add a Planar joint for the Pelvis Body
pelvisToPlatform = PlanarJoint('PelvisToPlatform', platform, pelvis);
% Update the coordinates of the new joint
Pelvis_rz = pelvisToPlatform.upd_coordinates(0); % Rotation about z
Pelvis_rz.setRange([-pi, pi]);
Pelvis_rz.setName('Pelvis_rz');
Pelvis_rz.setDefaultValue(0);

Pelvis_tx = pelvisToPlatform.upd_coordinates(1); % T about x
Pelvis_tx.setRange([-10, 10]);
Pelvis_tx.setName('Pelvis_tx');
Pelvis_tx.setDefaultValue(-10);
Pelvis_tx.setDefaultSpeedValue(0)

Pelvis_ty = pelvisToPlatform.upd_coordinates(2); % Translation about y
Pelvis_ty.setRange([-5,5]);
Pelvis_ty.setName('Pelvis_ty');
Pelvis_ty.setDefaultValue(legLength);
Pelvis_ty.setDefaultSpeedValue(0)
% Add Joint to model
osimModel.addJoint(pelvisToPlatform)

%% Add Hind Legs

[LegS, ContactS, ForceS] = deal(struct);

% Make and add a Right Hind legs
nLegs = nRightLegs;
angleOffset = angleOffsetRight;
legAngle = 180/nLegs;
hipPos = pelvisWidth;
sidePrefix = 'rHind';
RimlessWheel_addHindLegs

% Make and add a Left Hind leg 1
nLegs = nLeftLegs;
legAngle = 180/nLegs;
angleOffset = angleOffsetLeft;
hipPos = -pelvisWidth;
sidePrefix = 'lHind';
RimlessWheel_addHindLegs

%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
fname = [modelName,'.osim'];
osimModel.print(fname);
disp([fname,' printed!']);
