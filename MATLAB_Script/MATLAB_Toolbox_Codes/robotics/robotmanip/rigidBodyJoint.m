classdef rigidBodyJoint < handle
    %rigidBodyJoint Create a joint 
    %   rigidBodyJoint creates a joint object that defines how a rigid body moves
    %   relative to another. In a tree-structured robot, a joint always
    %   belongs to a specific rigid body, and each rigid body has exactly
    %   one joint.
    %
    %   JNT = rigidBodyJoint(JNAME) creates a fixed joint with name
    %   JNAME. Note that a nonempty joint name is required.
    %
    %   JNT = rigidBodyJoint(JNAME, JOINTTYPE) creates a joint of type
    %   JOINTTYPE and with name JNAME.
    %
    %   
    %   rigidBodyJoint properties:
    %       Type                   - Type of the joint
    %       Name                   - Name of the joint
    %       PositionLimits         - Position limits of the joint
    %       HomePosition           - Home Position of the joint
    %       JointAxis              - Axis of motion for revolute/prismatic joint
    %       JointToParentTransform - Fixed transform from joint to parent frame
    %       ChildToJointTransform  - Fixed transform from child to joint frame
    %
    %
    %   rigidBodyJoint methods:
    %       copy               - Create a copy of the joint
    %       setFixedTransform  - Set fixed transform properties of the joint 
    %
    %   Example:
    %
    %       % Create a revolute joint named 'joint1' 
    %       jnt1 = rigidBodyJoint('joint1','revolute');
    %
    %       % Create a prismatic joint named 'joint2' 
    %       jnt2 = rigidBodyJoint('joint2','prismatic');
    %
    %   See also rigidBody, rigidBodyTree.
    
    %#codegen
    
    %   Copyright 2016-2019 The MathWorks, Inc.
       
    properties (SetAccess = private)
        
        %Type Joint type 
        %   Currently, supported joint types are: revolute, prismatic and fixed. 
        %
        %   Default: 'fixed'
        Type 
        
    end
    
    properties (Dependent)
        %Name Name of the joint
        %   
        %   Default: ''
        Name = ''
    end
    
    properties (Access={?robotics.manip.internal.InternalAccess, ?rigidBodyJoint})

       %VelocityNumber
       VelocityNumber 
       
       %PositionNumber
       PositionNumber 
       
       %MotionSubspace
       MotionSubspace    
       
       %InTree
       InTree = false
    end
    

    
    properties (Dependent)
        %JointAxis Axis of motion for revolute/prismatic joint
        %   JointAxis represents the rotation axis of a revolute joint, 
        %   or the sliding direction of a prismatic joint. This property is
        %   only meaningful for revolute or prismatic joints.
        %
        %   Default: [nan nan nan]
        JointAxis
        
        
        %PositionLimits Position limits of the joint
        %   
        %   Default: [0, 0]
        PositionLimits
        
        %HomePosition Home position of the joint
        %   Home position must fall in the range set by PositionLimits.
        %   
        %   Default: 0
        HomePosition        
        
    end
       
    properties (SetAccess={?robotics.manip.internal.InternalAccess, ?rigidBodyJoint})
        %JointToParentTransform The first fixed transform property of a joint 
        %   JointToParentTransform represents the homogeneous transform
        %   that converts points originally expressed in the joint predecessor
        %   frame to the parent body frame. This property is read-only.
        %
        %   Note joint predecessor frame is fixed on the parent body.
        %
        %   Default: eye(4)  
        JointToParentTransform = eye(4)
        
        %ChildToJointTransform The second fixed transform property of a joint 
        %   ChildToJointTransform represents the homogeneous transform that
        %   converts points originally expressed in the child body frame to
        %   the joint successor frame. This property is read-only. 
        %
        %   Note the rigid body who owns the joint is the child body in this
        %   context. And the joint successor frame is fixed on the child
        %   body.
        %   
        %   Default: eye(4)  
        ChildToJointTransform = eye(4)
        
    end
    
    properties (Access={?robotics.manip.internal.InternalAccess, ?rigidBodyJoint})
        
        NameInternal
        
        PositionLimitsInternal = [0, 0] 
        
        HomePositionInternal = 0
        
        JointAxisInternal = [0 0 0]
        
    end

    
   
    methods
        function obj = rigidBodyJoint(jname, jtype)
            %rigidBodyJoint Constructor
            %   rigidBodyJoint constructor takes either one input argument (joint
            %   name) or two arguments (joint name, and joint type)
            narginchk(1,2);
            
            jname = convertStringsToChars(jname);
            validateattributes(jname, {'char', 'string'}, {'nonempty','row'}, ...
                             'rigidBodyJoint', 'jname');
            coder.varsize('jointname', [1, inf], [0, 1]);
            jointname = jname;
            obj.NameInternal = jointname;

            coder.varsize('jointtype', [1, 20], [0, 1]);
            if nargin == 2
                jointtype = validatestring(jtype, ...
                    {'revolute', 'prismatic', 'fixed'}, ...
                     'rigidBodyJoint','jtype');
            else
                jointtype = 'fixed'; 
            end

            obj.Type = jointtype;
            
            coder.varsize('msubspace', [6, 6], [0, 1]);
            coder.varsize('poslim', [6, 2], [1, 0]);
            coder.varsize('homepos', [6, 1], [1,0]);
            
            switch obj.Type
                case 'revolute' 
                    msubspace = [0 0 1 0 0 0]';
                    poslim = [-pi, pi];
                    homepos = 0; 
                    obj.VelocityNumber = 1;
                    obj.PositionNumber = 1;
                    obj.JointAxisInternal = [0 0 1];
                case 'prismatic' 
                    msubspace = [0 0 0 0 0 1]';
                    poslim = [-0.5, 0.5];
                    homepos = 0;                    
                    obj.VelocityNumber = 1;
                    obj.PositionNumber = 1;
                    obj.JointAxisInternal = [0 0 1];
                otherwise % 'fixed' 
                    msubspace = zeros(6,1);
                    poslim = [0, 0];
                    homepos = 0;
                    obj.VelocityNumber = 0;
                    obj.PositionNumber = 0;
                    obj.JointAxisInternal = [0 0 0 ];
            end
            
            obj.MotionSubspace = msubspace;
            obj.PositionLimitsInternal = poslim;
            obj.HomePositionInternal = homepos;
        end
        
       
        function newjoint = copy(obj)
            %copy Creates a copy of the rigidBodyJoint object
            %   NEWJNT = COPY(JNT) creates a copy of the jnt object
            %   with the same properties.
            %
            %   Example:
            %       % Create a revolute joint named 'joint_1'
            %       jnt1 = rigidBodyJoint('joint_1','revolute');
            %
            %       % Create a copy of the joint object
            %       jnt1cp = copy(jnt1);
            %
            %       % Delete the old joint object
            %       delete(jnt1)
            %
            %   See also rigidBodyJoint
            
            newjoint = copyInternal(obj); 
        end
        
 
    end
        

    
    methods
        
        function set.Name(obj, nm)
            nm = convertStringsToChars(nm);
            validateattributes(nm, {'char','string'},{'nonempty','row'},'rigidBodyJoint','Name');
            if ~obj.InTree
                obj.NameInternal = nm;
            else
                robotics.manip.internal.warning('joint:JointNotFree');
            end
        end
        
        function nm = get.Name(obj)
            nm = obj.NameInternal;
        end
        
        function set.PositionLimits(obj, lims)
            resetHome = validatePositionLimits(obj,lims); 
            lims = double(lims);
            obj.PositionLimitsInternal = lims;
            if resetHome
                obj.resetHomePosition();
            end
        end
        
        function set.HomePosition(obj, home)
            validateHomePosition(obj,home);
            obj.HomePositionInternal = double(home);
        end
        
        function home = get.HomePosition(obj)
            home = obj.HomePositionInternal;
        end

        function lims = get.PositionLimits(obj)
            lims = obj.PositionLimitsInternal;
        end        
        
        function ax = get.JointAxis(obj)
            if strcmp(obj.Type,'revolute') || strcmp(obj.Type,'prismatic')
                ax = obj.JointAxisInternal;
            else
                ax = [nan nan nan];
            end
        end 
        
        function set.JointAxis(obj, ax)
            validateattributes(ax, {'double'}, ...
                {'nonnan', 'real', 'finite', 'nonempty', 'vector', 'numel',3},...
                'rigidBodyJoint','JointAxis');
            if ~strcmp(obj.Type,'prismatic') && ~strcmp(obj.Type,'revolute')
                robotics.manip.internal.error('joint:JointAxisNotSettable', obj.Type); 
            end
            ax = reshape(ax/norm(ax),[1,3]);
            if abs(norm(ax)) - 1 < sqrt(eps)
                obj.JointAxisInternal = ax;
                obj.resetMotionSubspace(ax);
            else
                robotics.manip.internal.error('joint:JointAxisNormalizationError');
            end
        end
        
        function setFixedTransform(obj, input, notation)
            %setFixedTransform Set fixed transform properties of joint
            %
            %   setFixedTransform(JOINT, DHPARAMS, 'dh') sets 
            %   ChildToJointTransform using Denavit-Hartenberg parameters
            %   DHPARAMS and JointToParentTransform to an identity matrix. 
            %   DHPARAMS are given in the order [a alpha d theta].
            %
            %   setFixedTransform(JOINT, MDHPARAMS, 'mdh') sets 
            %   JointToParentTransform using Modified Denavit-Hartenberg
            %   parameters MDHPARAMS and ChildToJointTransform to an 
            %   identity matrix. MDHPARAMS are given in the order 
            %   [a alpha d theta]. 
            %
            %   Note that for a revolute joint, theta is a joint variable
            %   and thus not considered part of the joint's fixed transform.
            %   Similarly, for a prismatic joint, d is a joint variable.
            %   Joint variables in DH/MDH parameters will be ignored during
            %   setFixedTransform calls. Check the methods in RigidBodyTree
            %   class (such as getTransform) to see how joint variables are
            %   specified.
            %
            %   setFixedTransform(JOINT, T) sets JointToParentTransform to
            %   the user-supplied 4x4 homogeneous transform matrix T and 
            %   ChildToJointTransform to an identity matrix.
            %
            %   Example:
            %       % Create a revolute joint named joint_2 
            %       jnt2 = rigidBodyJoint('joint_2','revolute');
            %
            %       % Set fixed joint transforms using MDH parameters 
            %       setFixedTransform(jnt2, [0, -pi/2, 0.2435, 0.5],'mdh');
            %
            %       % Note: For a revolute joint, the theta (4th entry of 
            %       % the MDH parameter) is in fact a joint variable and 
            %       % not considered part of the fixed transform. As a 
            %       % result it is ignored during setFixedTransform call. 
            %       % The line of code above is equivalent to:
            %       setFixedTransform(jnt2, [0, -pi/2, 0.2435, 0.0],'mdh');
            
            narginchk(2,3);
            if nargin > 2
                n = validatestring(notation,{'dh', 'mdh'},...
                    'setFixedTransform','notation'); 
                if strcmp(n, 'dh')
                    extractFixedTransformFromDH(obj, input); 
                else
                    extractFixedTransformFromMDH(obj, input); 
                end
                return
            end
            
            %robotics.internal.validation.validateHomogeneousTransform
            validateattributes(input, {'double'},...
                {'nonnan', 'finite', 'real','nonempty', 'size',[4, 4]}, ...
                 'setFixedTransform', 'input'); 
            if ~isequal(double(input(4,:)),[0 0 0 1])
                robotics.manip.internal.error('joint:LastRowOfHomogeneousTransformMatrix');
            end

            obj.JointToParentTransform = double(input);  
            obj.ChildToJointTransform = eye(4);  

        end
        
        
    end
    
    
    methods (Access = private) 

        function newjoint = copyInternal(obj)
            %copyInternal
            jtype = obj.Type;
            jname = obj.Name;
            
            newjoint = rigidBodyJoint(jname, jtype);
            
            if ~isempty(obj.Name)
                newjoint.Name = obj.Name;
            end
            
            newjoint.PositionLimitsInternal = obj.PositionLimitsInternal;
            newjoint.HomePositionInternal =obj.HomePositionInternal;
            newjoint.JointAxisInternal = obj.JointAxisInternal;  
            newjoint.MotionSubspace = obj.MotionSubspace;
            
            newjoint.JointToParentTransform = obj.JointToParentTransform; 
            newjoint.ChildToJointTransform = obj.ChildToJointTransform;
        end    
        
        function resetMotionSubspace(obj, ax)
            switch(obj.Type)
                case 'revolute'
                    obj.MotionSubspace = [ax(1) ax(2) ax(3) 0 0 0]';
                otherwise % case 'prismatic'
                    obj.MotionSubspace = [0 0 0 ax(1) ax(2) ax(3)]';
            end
        end
                
        
        
        function TJ = jointTransform(obj, q)
            %jointTransform Compute the variable joint transform that is
            %   dependent on joint position. q is the joint position vector. 
            
            switch(obj.Type)
                case 'fixed'
                    TJ = eye(4);
                case 'revolute'
                    v = obj.JointAxis;
                    TJ = axang2tform([v, q]);
                otherwise % case 'prismatic'
                    v = obj.JointAxis;
                    TJ = [eye(3),v'*q;[0 0 0 1]];
            end                 
        end %jointTransform
        
        function resetHome = validatePositionLimits(obj, lims)
            resetHome = false;
            switch(obj.PositionNumber)
                case 0  % fixed joint
                    robotics.manip.internal.error('joint:FixedJointPositionLimitsNotSettable');
                otherwise %case 1  % revolute or prismatic joint
                    validateattributes(lims, {'double'}, ...
                      { 'nonnan', 'real', 'nonempty', 'size', [1 2], 'nondecreasing'},...
                       'rigidBodyJoint', 'PositionLimits');

                    if any(obj.HomePositionInternal > lims(:,2)) || any(obj.HomePositionInternal< lims(:,1))
                        resetHome = true;
                        robotics.manip.internal.warning('joint:ResettingHomePosition');  
                    end
            end                
        end

        function validateHomePosition(obj, home)
            
            switch(obj.PositionNumber)
                case 0
                    robotics.manip.internal.error('joint:FixedJointHomePositionNotSettable');
                otherwise % case 1
                    validateattributes(home, {'double'}, ...
                        {'nonempty', 'scalar', 'nonnan', 'real'}, ...
                         'rigidBodyJoint','HomePosition');
                    if any(home > obj.PositionLimitsInternal(:,2)) || any(home < obj.PositionLimitsInternal(:,1))
                        robotics.manip.internal.error('joint:ProvidedJointHomePositionOutOfLimits');  
                    end
            end                
        end

        function extractFixedTransformFromMDH(obj, mdhparams )
            %extractFixedTransformFromMDH
            validateattributes(mdhparams, {'double'}, {'nonnan', ...
                'finite', 'real', 'nonempty', 'vector', ...
                'numel', 4}, 'extractFixedTransformFromMDH','mdhparams'); 
            
            a = mdhparams(1);
            alpha = mdhparams(2);
            d = mdhparams(3);
            theta = mdhparams(4);

            Ta = [eye(3), [a, 0, 0]'; [0, 0, 0, 1]];
            Talpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0;...
                       0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
            Td = [eye(3), [0, 0, d]'; [0, 0, 0, 1]];
            Ttheta = [cos(theta), -sin(theta), 0, 0; ...
                      sin(theta), cos(theta), 0, 0; 0, 0, 1, 0;0, 0, 0, 1];

            TL = eye(4); 
            switch(obj.Type)
                case 'revolute'
                    TL = Ta*Talpha*Td;
                case 'prismatic'
                    TL = Ta*Talpha*Ttheta;
                case 'fixed'
                    TL = Ta*Talpha*Ttheta*Td;
                otherwise
                    robotics.manip.internal.error('joint:DHApplicability');
            end
            obj.JointToParentTransform = TL;   
            obj.ChildToJointTransform = eye(4); 
        end
        
        function extractFixedTransformFromDH(obj, dhparams)
            %extractFixedTransformFromDH 
            validateattributes(dhparams, {'double'}, { 'nonnan', ...
                'finite', 'real', 'nonempty', 'vector',...
                'numel', 4}, 'extractFixedTransformFromDH', 'dhparams'); 
            
            a = dhparams(1);
            alpha = dhparams(2);
            d = dhparams(3);
            theta = dhparams(4);

            Ta = [eye(3), [a, 0, 0]'; [0, 0, 0, 1]];
            Talpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0;...
                        0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
            Td = [eye(3), [0, 0, d]'; [0, 0, 0, 1]];
            Ttheta = [cos(theta), -sin(theta), 0, 0; ...
                   sin(theta), cos(theta), 0, 0; 0, 0, 1, 0;0, 0, 0, 1];

            TL = eye(4);
            switch(obj.Type)
                case 'revolute'
                    TL = Td*Ta*Talpha; 
                case 'prismatic'
                    TL = Ttheta*Ta*Talpha; 
                case 'fixed'
                    TL = Ttheta*Td*Ta*Talpha; 
                otherwise
                    robotics.manip.internal.error('joint:DHApplicability');
            end
            obj.ChildToJointTransform = TL;  
            obj.JointToParentTransform = eye(4); 

        end
        
        
    end % private methods
    
    
    methods (Access = {?robotics.manip.internal.InternalAccess, ?rigidBodyJoint})
        function T = transformBodyToParent(obj, q)
            %transformBodyToParent
            
            T = obj.JointToParentTransform*...
                  obj.jointTransform(q)*obj.ChildToJointTransform;
        end
        
        function qv = randomPosition(obj)
            %randomPosition Pseudo-random number drawn from global random
            %   stream
            switch(obj.PositionNumber)
                case 0
                    qv = nan;
                otherwise
                    ub = obj.PositionLimitsInternal(:,2);
                    lb = obj.PositionLimitsInternal(:,1);
                    if all(isfinite(lb)) && all(isfinite(ub))
                        rn = rand(obj.PositionNumber, 1);
                        qv = lb + rn.*(ub-lb);
                    elseif all(isfinite(lb)) && any(~isfinite(ub))
                        qv = lb + abs(randn(size(lb)));
                    elseif any(~isfinite(lb)) && all(isfinite(ub))
                        qv = ub - abs(randn(size(ub)));   
                    else
                        qv = randn(size(ub));
                    end
            end            
        end
        
                
        function resetHomePosition(obj)
            %resetHomePosition
            ub = obj.PositionLimitsInternal(:,2);
            lb = obj.PositionLimitsInternal(:,1);
            if obj.PositionNumber == 1
                if all(isfinite(lb)) && all(isfinite(ub))
                    obj.HomePositionInternal = 0.5*(lb + ub);
                elseif all(isfinite(lb)) && any(~isfinite(ub))
                    obj.HomePositionInternal = lb;
                elseif any(~isfinite(lb)) && all(isfinite(ub))
                    obj.HomePositionInternal = ub;
                else
                    obj.HomePositionInternal = zeros(size(lb));
                end
            end
        end
        
    end
    
end

