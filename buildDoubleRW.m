function fname = buildDoubleRW(Murphy,Phase,nSpokes,Slope,options)
% Quadrupedal rimless wheel

optionsDefault = struct;

% whether to run a simulation
optionsDefault.simulate = true;
optionsDefault.endTime = 5;
optionsDefault.visualizeSim = true;
optionsDefault.visualizeModel = true;

if nargin < 5 || isempty(options)
   options = optionsDefault;
end

options = fillDefaults(options,optionsDefault);

%% Import OSim Libraries
import org.opensim.modeling.*

%% Key Model Variables
modelNamePrefix = 'DoubleRW';
resultsDir = 'modelsAndResults';
reorientGravity = true; % if true, there is no platform joint. Gravity is reoriented according to the platform angle.

legLength = 0.50;
global legWidth contactSphereRadius stiffness dissipation ...
    staticFriction dynamicFriction viscousFriction transitionVelocity cylLength
legWidth = 0.05;
wheelMass = 1;
trunkMass = 20;
trunkLength = 1.5;
trunkWidth = 0.5;
trunkDepth = 0.125;
contactSphereRadius = 0.025;
rampInitialAngle = Slope; % negative angles point the normal force along the x axis
rampHeightOffset = 5;
trunkColor = [255,10,10]/256;

leftRightPhase = 0.5;
hindForePhase = Phase;
angleOffsetRight = 360/nSpokes*leftRightPhase;
angleOffsetLeft = 0;
angleOffsetHindToFore = 360/nSpokes*hindForePhase;
hipPosX = -trunkLength/2;
shoulderPosX = trunkLength/2;

trunkMOI = [Murphy(1)*trunkWidth^2,Murphy(2)*trunkWidth^2,Murphy(3)*trunkLength^2]*trunkMass/4;

initialSpeed = 0.5;

% Define Contact Force Parameters
stiffness           = 2000000;
dissipation         = 2.0;
staticFriction      = 2;
dynamicFriction     = 0.8;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;
% halve leg length, because of how cylinders are built in opensim
cylLength = legLength/2;

% get leg mass
legMassLeft = wheelMass/nSpokes;
legMassRight = wheelMass/nSpokes;


%% intantiate an empty OpenSim Model
angleOffsetRightToLeft = angleOffsetRight - angleOffsetLeft;
modelNamePostfix = sprintf('%i_M%.2f-%.2f-%.2f_RL%.0f_FH%.0f',nSpokes,Murphy(1),Murphy(2),Murphy(3),angleOffsetRightToLeft,angleOffsetHindToFore);
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
coordNames = strcat('Trunk_',{'rx','ry','rz','tx','ty','tz'});
if reorientGravity
    trunkToPlatform = CustomFreeJoint('TrunkToGround', ground, trunk, coordNames);    
else
    trunkToPlatform = CustomFreeJoint('TrunkToPlatform', platform, trunk);
end
% Add Joint to model
osimModel.addJoint(trunkToPlatform)

%
% Update the coordinates of the new joint
trunk_rx = trunkToPlatform.upd_coordinates(0); % Rotation about x
trunk_rx.setRange([-pi, pi]);
trunk_rx.setDefaultValue(0);

trunk_ry = trunkToPlatform.upd_coordinates(1); % Rotation about y
trunk_ry.setRange([-pi, pi]);
trunk_ry.setDefaultValue(0);

trunk_rz = trunkToPlatform.upd_coordinates(2); % Rotation about z
trunk_rz.setRange([-pi, pi]);
trunk_rz.setDefaultValue(0);

trunk_tx = trunkToPlatform.upd_coordinates(3); % Translation about x
trunk_tx.setRange([-10, 10]);
trunk_tx.setDefaultValue(-10*(~reorientGravity)); % zero if gravity is reoriented, -10 if on the ramp
trunk_tx.setDefaultSpeedValue(initialSpeed)
 
trunk_ty = trunkToPlatform.upd_coordinates(4); % Translation about y
trunk_ty.setRange([0,5]);
trunk_ty.setDefaultValue(legLength+0.1);
trunk_ty.setDefaultSpeedValue(0)

trunk_tz = trunkToPlatform.upd_coordinates(5); % Translation along z
trunk_tz.setRange([-5,5]);
trunk_tz.setDefaultValue(0);
trunk_tz.setDefaultSpeedValue(0)

osimModel.initSystem();
%% Add Hind Legs

[LegS, ContactS, ForceS] = deal(struct);

% Make and add Left Hind legs
nLegs = nSpokes;
newLegSet = true;
angleOffsetToPinLeg = 0;
angleOffset = angleOffsetLeft;
legAngle = 360/nSpokes;
trunkToLegPosZ = -trunkWidth/2;
trunkToLegPosX = hipPosX;
legZOffset = 0;
sidePrefix = 'lHind';
legMass = legMassLeft;
RimlessWheelQuad_addLegs

% Make and add a Right Hind legs
newLegSet = false;
angleOffsetToPinLeg = angleOffsetRight-angleOffsetLeft;
legZOffset = trunkWidth;
legAngle = 360/nSpokes;
sidePrefix = 'rHind';
legMass = legMassRight;
RimlessWheelQuad_addLegs

%% Add Fore Legs

% Make and add Left Fore legs
newLegSet = true;
legAngle = 360/nSpokes;
angleOffset = angleOffsetLeft;
angleOffsetToPinLeg = 0;
legZOffset = 0;
trunkToLegPosZ = -trunkWidth/2;
trunkToLegPosX = shoulderPosX;
sidePrefix = 'lFore';
RimlessWheelQuad_addLegs

% Make and add Right Fore legs
newLegSet = false;
angleOffsetToPinLeg = angleOffsetRight-angleOffsetLeft;
legZOffset = trunkWidth;
legAngle = 360/nLegs;
sidePrefix = 'rFore';
RimlessWheelQuad_addLegs

%% Add coordinate coupler constraint

legPhaseConstraint = CoordinateCouplerConstraint();
%legPhaseConstraint = legPhaseConstraint.safeDownCast(legPhaseConstraint)

independentCoords = ArrayStr();
independentCoords.append('lHind1_rz');
legPhaseConstraint.setIndependentCoordinateNames(independentCoords);
legPhaseConstraint.setDependentCoordinateName('lFore1_rz');
coefficients = Vector(3,0);
coefficients.set(0,1);
legPhaseConstraint.setFunction(LinearFunction(1,deg2rad(angleOffsetHindToFore)))
osimModel.addConstraint(legPhaseConstraint);


%% Initialize the System (checks model consistency).
osimModel.initSystem();

saveDir = [resultsDir,'/',modelName,'/'];
mkdir(saveDir)

% Save the model to a file
fname = [saveDir,modelName,'.osim'];
osimModel.print(fname);
disp([fname,' printed!']);
fname = [pwd,filesep,fname];


%% Run simulation
if options.simulate
    VisOptions = struct('endTime',options.endTime,'stepSize',0.001,'reportInterval',0.01,...
    'useVis',options.visualizeSim);
    RimlessWheelForwardSimulation(fname,'',[],VisOptions);
end

% show the model
if options.visualizeModel
    VisualizerUtilities().showModel(osimModel)
end

end

function osimModel = addLegs(osimModel,LegS,ContactS,ForceS,newLegSet,Nspokes,angleOffset,trunkToLegPosX,trunkToLegPosZ,angleOffsetToPinLeg,legZOffset,legAngle,sidePrefix,legMass);
global legWidth contactSphereRadius 

import org.opensim.modeling.*
for i = 1:Nspokes
    bodyname = [sidePrefix,num2str(i)];
    if newLegSet
        jointname = [bodyname,'ToTrunk'];
        pinnedLegName = bodyname;
    else
        jointname = [bodyname,'To',pinnedLegName];
    end
    LegS.(bodyname) = Body();
    LegS.(bodyname).setName(bodyname);
    LegS.(bodyname).setMass(legMass);
    LegS.(bodyname).setInertia(Inertia(0,0,0,0,0,0));
    % Add geometry for display
    dispGeom = Cylinder(legWidth/2,cylLength);
    if newLegSet
        dispColor = [255,255,51]/256; % a nice yellow color, only for pinned leg
    else
        dispColor = [1 1 1];
    end
    dispGeom.setColor(Vec3(dispColor(1),dispColor(2),dispColor(3)));
    LegS.(bodyname).attachGeometry(dispGeom);
    
    % Add Body to the Model
    osimModel.addBody(LegS.(bodyname));
    
    
    % Make and add a joint for the leg
    orientationInParent = Vec3(0,0,deg2rad(legAngle*(i-1)+newLegSet*angleOffset+(~newLegSet)*angleOffsetToPinLeg));
    locationInChild     = Vec3(0,-cylLength,0);
    orientationInChild  = Vec3(0,0,0);
    
    if newLegSet
        % use a pin joint
        locationInParent    = Vec3(trunkToLegPosX,0,trunkToLegPosZ);
        LegS.(jointname) = PinJoint(jointname, trunk, locationInParent, ...
            orientationInParent, LegS.(bodyname), locationInChild, orientationInChild);
        pinJoint_rz = LegS.(jointname).upd_coordinates(0);
        pinJoint_rz.setName([bodyname,'_rz'])
        osimModel.addJoint(LegS.(jointname))
    else
        % Weld joint for all other legs
        locationInParent    = Vec3(0,-cylLength,legZOffset);
        LegS.(jointname) = WeldJoint(jointname, LegS.(pinnedLegName), locationInParent, ...
            orientationInParent, LegS.(bodyname), locationInChild, orientationInChild);
        osimModel.addJoint(LegS.(jointname))
    end
    
    contactname = [bodyname,'Contact'];
    forcename = [bodyname,'Force'];
    % Make a Right leg Contact Sphere
    ContactS.(contactname) = ContactSphere();
    ContactS.(contactname).setRadius(contactSphereRadius);
    ContactS.(contactname).setLocation( Vec3(0,cylLength,0) );
    ContactS.(contactname).setFrame(LegS.(bodyname))
    ContactS.(contactname).setName(contactname);
    osimModel.addContactGeometry(ContactS.(contactname));
    
    % Make a Smooth Hunt Crossley Force and update parameters
    ForceS.(forcename) = SmoothSphereHalfSpaceForce(forcename,ContactS.(contactname),groundContactSpace);
    ForceS.(forcename).set_stiffness(stiffness);
    ForceS.(forcename).set_dissipation(dissipation);
    ForceS.(forcename).set_static_friction(staticFriction);
    ForceS.(forcename).set_dynamic_friction(dynamicFriction);
    ForceS.(forcename).set_viscous_friction(viscousFriction);
    ForceS.(forcename).set_transition_velocity(transitionVelocity);
    osimModel.addForce(ForceS.(forcename));

    newLegSet = false;
end
end