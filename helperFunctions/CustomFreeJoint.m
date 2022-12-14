% CustomFreeJoint creates a custom joint equivalent to a free joint.
% Useage:
% joint = CustomFreeJoint(jointName,parentBody,childBody) creates a
% CustomJoint called joint at the origins of childBody and parentBody.
% Coordinates are given default names of rot{X,Y,Z} and trans{X,Y,Z}.
%
% joint = CustomFreeJoint(jointName,parentBody,childBody,coordNames) as
% above, but sets coordinate names to the values given in the cell array
% coordNames. Names must be given according to coordinate index (i.e. first
% rotation about X, then rotation about Y... finishing with translation
% along Z)
%
% To specify coordinate parameters, use the following template after
% running joint = CustomFreeJoint(...):
% 
%       model.addJoint(joint)
%       joint = model.updJointSet().get(jointName)
%       rotX = joint.upd_coordinates(0);
%       rotX.setRange([-pi, pi])
%       ... etc ...
%       model.initSystem()
%
% Note that it does not seem possible to change joint names after they are
% specified in the CustomJoint

function joint = CustomFreeJoint(name,parentBody,childBody,coordNames)

if nargin < 4
    coordNames = {'rotX','rotY','rotZ','transX','transY','transZ'};
elseif ~iscell(coordNames) || length(coordNames) ~= 6
       error('coordNames must be a cell array of strings of length 6')
end
    

import org.opensim.modeling.*

rot1 = TransformAxis();
rot2 = TransformAxis();
rot3 = TransformAxis();
tra1 = TransformAxis();
tra2 = TransformAxis();
tra3 = TransformAxis();

rot1.set_coordinates(0,coordNames{1})
rot1.setAxis(Vec3(1, 0, 0));
rot1.set_function(LinearFunction());

rot2.set_coordinates(0,coordNames{2})
rot2.setAxis(Vec3(0, 1, 0));
rot2.set_function(LinearFunction());

rot3.set_coordinates(0,coordNames{3})
rot3.setAxis(Vec3(0, 0, 1));
rot3.set_function(LinearFunction());

tra1.set_coordinates(0,coordNames{4})
tra1.setAxis(Vec3(1, 0, 0));
tra1.set_function(LinearFunction());

tra2.set_coordinates(0,coordNames{5})
tra2.setAxis(Vec3(0, 1, 0));
tra2.set_function(LinearFunction());

tra3.set_coordinates(0,coordNames{6})
tra3.setAxis(Vec3(0, 0, 1));
tra3.set_function(LinearFunction());

st_FreeJoint = SpatialTransform();
st_FreeJoint.set_rotation1(rot1)
st_FreeJoint.set_rotation2(rot2)
st_FreeJoint.set_rotation3(rot3)
st_FreeJoint.set_translation1(tra1)
st_FreeJoint.set_translation2(tra2)
st_FreeJoint.set_translation3(tra3)


joint = CustomJoint(name,parentBody,Vec3(0,0,0),Vec3(0,0,0),childBody,Vec3(0,0,0),Vec3(0,0,0),st_FreeJoint);
