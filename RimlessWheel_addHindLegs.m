import org.opensim.modeling.*
for i = 1:nLegs
    bodyname = [sidePrefix,num2str(i)];
    jointname = [bodyname,'ToPelvis'];
    LegS.(bodyname) = Body();
    LegS.(bodyname).setName(bodyname);
    LegS.(bodyname).setMass(0.1);
    LegS.(bodyname).setInertia(Inertia(0,0,0,0,0,0));
    % Add geometry for display
    LegS.(bodyname).attachGeometry(Cylinder(legWidth/2,cylLength));
    % Add Body to the Model
    osimModel.addBody(LegS.(bodyname));
    
    % Make and add a Weld joint for the leg
    locationInParent    = Vec3(0,0,hipPos);
    orientationInParent = Vec3(0,0,deg2rad(legAngle*(i-1) + angleOffset));
    locationInChild     = Vec3(0,-cylLength,0);
    orientationInChild  = Vec3(0,0,0);
    LegS.(jointname) = WeldJoint(jointname, pelvis, locationInParent, ...
        orientationInParent, LegS.(bodyname), locationInChild, orientationInChild);
    osimModel.addJoint(LegS.(jointname))
    
    contactname = [bodyname,'Contact'];
    forcename = [bodyname,'Force'];
    % Make a Right leg Contact Sphere
    ContactS.(contactname) = ContactSphere();
    ContactS.(contactname).setRadius(contactSphereRadius);
    ContactS.(contactname).setLocation( Vec3(0,cylLength,0) );
    ContactS.(contactname).setFrame(LegS.(bodyname))
    ContactS.(contactname).setName(contactname);
    osimModel.addContactGeometry(ContactS.(contactname));
    
    % Make a Hunt Crossley Force for Right Hind1 and update parameters
    ForceS.(forcename) = HuntCrossleyForce();
    ForceS.(forcename).setName(forcename);
    ForceS.(forcename).addGeometry(contactname);
    ForceS.(forcename).addGeometry('PlatformContact');
    ForceS.(forcename).setStiffness(stiffness);
    ForceS.(forcename).setDissipation(dissipation);
    ForceS.(forcename).setStaticFriction(staticFriction);
    ForceS.(forcename).setDynamicFriction(dynamicFriction);
    ForceS.(forcename).setViscousFriction(viscousFriction);
    ForceS.(forcename).setTransitionVelocity(transitionVelocity);
    osimModel.addForce(ForceS.(forcename));
end