function inverseKinematicsDesigner(varargin)
%INVERSEKINEMATICSDESIGNER Design IK solvers, waypoints, and paths for robot models
%   INVERSEKINEMATICSDESIGNER opens the Inverse Kinematics Designer app.
%
%   INVERSEKINEMATICSDESIGNER(SESSIONPATHNAME) opens the Inverse Kinematics
%   Designer app using session data saved as a MAT file at the specified
%   filepath, SESSIONPATHNAME.
%
%   % EXAMPLE:
%   % Launch the app
%   inverseKinematicsDesigner;
%
%    See also inverseKinematics, generalizedInverseKinematics, interactiveRigidBodyTree

%   Copyright 2021 The MathWorks, Inc.

    robotics.ikdesigner.internal.InverseKinematicsDesigner(varargin{:});

end
