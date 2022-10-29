classdef ros2node < ros.internal.mixin.InternalAccess & ...
        coder.ExternalDependency
    %ros2node Initialize ROS 2 node on specified network
    %   The ros2node object represents a ROS 2 node, and provides a foundation
    %   for communication with the rest of the ROS 2 network.
    %
    %   N = ros2node("NAME") initializes the ROS 2 node with the given NAME.
    %   The node connects with the default domain identification 0 unless
    %   otherwise specified by the ROS_DOMAIN_ID environment variable.
    %
    %   The node uses default ROS Middleware implementation 'rmw_fastrtps_cpp'
    %   unless otherwise specified by the RMW_IMPLEMENTATION environment variable.
    %
    %   N = ros2node("NAME",ID) initializes the ROS 2 node with NAME and
    %   connects it to the network using the given domain ID. The acceptable
    %   values for the domain identification are between 0 and 232.
    %
    %   Node properties:
    %      Name         - (Read-only) Name of the node
    %      ID           - (Read-only) Network domain identification
    %
    %   Node methods:
    %      resolveName  - Check and resolve ROS 2 name
    %
    %   Example:
    %      % Initialize the node "/node_1" on the default network
    %      node1 = ros2node("/node_1");
    %
    %      % Create a separate node that connects to a different network
    %      % identified with domain 2
    %      node2 = ros2node("/node_2",2);

    %   Copyright 2021 The MathWorks, Inc.
    %#codegen
    properties (SetAccess = private)
        %Name - Name of the node
        %   This needs to be unique in the ROS 2 network
        Name

        %ID - Domain identification of the network
        %   Default: 0
        ID
    end

    properties (SetAccess = private, GetAccess = ?ros.internal.mixin.InternalAccess)
        %NodeHandle - Opaque variable representing C++ node handle
        NodeHandle

        %Namespace - Namespace of the node
        Namespace
    end

    methods
        function obj = ros2node(name,varargin)
            %ros2node Create a ROS 2 node object
            %   The "name" argument is required and specifies a node name to be
            %   used on the ROS 2 network.
            coder.extrinsic('ros.codertarget.internal.locAddNode')
            coder.internal.prefer_const(name); % Specialize ros2node class based on name

            % Parse and assign inputs
            narginchk(1, inf)
            parseInputs(obj,name,varargin{:});

            % Check and verify we have a single node defined
            coder.const(@ros.codertarget.internal.locAddNode,obj.Name); % Check multiple nodes in onAfterCodegen

            % Create NodeHandle which is an opaque variable representing
            % global node handle. This variable should be passed to
            % coder.ceval.
            obj.NodeHandle = coder.opaque('rclcpp::Node::SharedPtr','HeaderFile','mlros2_node.h');
            obj.NodeHandle = coder.ceval('MATLAB::getGlobalNodeHandle');
        end
    end

    methods (Access = ?ros.internal.mixin.InternalAccess)
        function rosName = resolveName(obj, name)
            %resolveName Check and resolve ROS 2 name
            %   ROSNAME = resolveName(NODE, NAME) resolves the ROS 2 name, NAME
            %   that is associated with a particular node object, NODE.
            %   This function returns a ROS 2 name with properly applied
            %   namespace in ROSNAME.
            %
            %   This function will take the relevant namespace into account if
            %   the name is defined as relative name without a leading slash.
            %   A relative name, e.g. 'testName', will be resolved relative to
            %   this node's root namespace. A private name, e.g. '~/testName'
            %   will be resolved relative to this node's namespace. A leading
            %   forward slash indicates a global name, e.g. '/testName'.
            %
            %   This function does not check the name against the ROS 2
            %   naming standard - it only resolves valid namespace indicators
            %   to be relative to the appropriate node-related namespace.

            rosName = name;                 % Default to no change
            if startsWith(name, '~/')       % Private namespace
                % Replace private token with full node name
                % (using full node namespace and name as the new namespace)
                rosName = strrep(name, '~', obj.Name);
            elseif ~startsWith(name, '/')   % Relative namespace
                rosName = [obj.Namespace, '/', name];
            end
        end

        function parseInputs(obj,name,varargin)
            coder.extrinsic('ros.internal.utilities.getDefaultDomainID')
            % Validate & set obj.Name
            name = convertStringsToChars(name);
            validateattributes(name,{'char'},{'nonempty','scalartext'}, ...
                'ros2node','name');
            obj.Name = name;

            % Validate & set obj.ID.
            % Domain IDs are typically between 0 and 232
            if nargin > 2
                validateattributes(varargin{1},{'numeric'}, ...
                    {'scalar','integer','nonnegative','<=', 232}, ...
                    'ros2node','id')
                obj.ID = varargin{1};
            else
                obj.ID = coder.const(ros.internal.utilities.getDefaultDomainID);
            end

            % Set namespace
            obj.Namespace = loc_getNodeNamespace(obj.Name);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'Name','ID'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS 2 Node';
        end

        function ret = isSupportedContext(ctx)
            ret = ctx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,ctx)
            if ctx.isCodeGenTarget('rtw')
                srcFolder = fullfile(toolboxdir('ros'),'codertarget','src');
                addIncludeFiles(buildInfo,'mlros2_node.h',srcFolder);
            end
        end
    end
end

% Helper functions
function namespace = loc_getNodeNamespace(name)
% Extract namespace as last text block separated by slash
for k = length(name):-1:1
    if name(k) == '/'
        break;
    end
end
namespace = name(1:k-1);
end

% LocalWords:  ROSNAME
