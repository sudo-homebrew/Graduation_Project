function stanfordRobot = stanford(D2, D6)
    %Stanford - This constructs the 6-DOF Stanford manipulator
    % The body names are: L1, L2,... L6
    % The joint names are: jnt1, jnt2... jnt6

    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen

    if ~nargin
        D2 = 0.15;
        D6 = 0.35;
    end

    stanfordRobot = rigidBodyTree('MaxNumBodies', 6);

    % The th (in the DH parameter) for the third joint (which is a prismatic joint)
    % can be anything, zero has been picked here.
    % th1 = th2 = th4 = th5 = th6 = 0 (all revolute joint positions should be
    % passed later as configurations)
    % d3 = 0; i.e., prismatic joint should get this value during
    % randomConfigurations
    th = zeros(6,1);

    % DH Parameters for the links
    %          [ a     alpha   d	th]
    dhParams = [ 0     -pi/2    0   th(1); ...
                 0     pi/2     D2	th(2); ...
                 0     0        0	  0    ; ...
                 0     -pi/2    0	  th(4); ...
                 0     pi/2     0	  th(5); ...
                 0     0        D6	th(6)];

   % common joint
   jnt = rigidBodyJoint('tempJointName', 'revolute');

   % common rigidbody
   link = rigidBody('tempLinkName');

   commonName = 'L';

   % Construct a serial link robot using the dh params
   for idx = 1:size(dhParams, 1)
       %Construct a valid (non-repeating) link name
       tempLinkName = [commonName num2str(idx)];

       if idx ==3
           % Create a prismatic joint
           newJoint = rigidBodyJoint(['jnt' num2str(idx)], 'prismatic');
           newJoint.setFixedTransform(dhParams(idx, :), 'dh');
       else
           %Modify common revolute joint after making a copy
           newJoint = jnt;
           newJoint.Name = ['jnt' num2str(idx)];
           newJoint.setFixedTransform(dhParams(idx, :), 'dh');
       end

       %Modify common link
       link.Name = tempLinkName;
       link.Joint = newJoint;

       %Add link to the robot (as a serial link)
       if idx == 1
            stanfordRobot.addBody(link, 'base');
       else
           stanfordRobot.addBody(link, [commonName num2str(idx - 1)]);
       end
   end
end
