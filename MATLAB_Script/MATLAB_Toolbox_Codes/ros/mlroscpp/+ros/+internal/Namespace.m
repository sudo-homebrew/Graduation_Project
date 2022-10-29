classdef Namespace
%This class is for internal use only. It may be removed in the future.

%Namespace Set of static functions for supporting ROS namespaces
%
%   Namespace methods:
%      canonicalizeName  - Validate ROS name
%      isValidGraphName  - Determines if the name is a valid graph name
%      isValidFieldName  - Determines if the name is a valid field name
%      parentNamespace   - Extract parent namespace from ROS graph name
%      resolveNodeName   - Resolve a node name

%   Copyright 2020 The MathWorks, Inc.

    properties (Constant, Access = private)
        %% Root Namespace
        ROOT = '/';
    end

    methods (Static)
        function validName = canonicalizeName(name)
        %ros.internal.Namespace.canonicalizeName Validate ROS name
        %   Validate and convert the ROS name into its canonical representation.
        %   Canonical representations have no trailing slashes and can be
        %   global (/...), private (~...), or relative. Please note
        %   that this function will not change the type of
        %   representation. For example, a relative name will not be
        %   changed to an absolute one.
        %
        %   An empty NAME string is a valid relative graph name (and
        %   will be converted to absolute once resolved).
        %
        %   This function will throw an error if the name is not a valid
        %   ROS graph resource name.
        %
        %   See also ros.internal.Namespace.isValidGraphName

            validateattributes(name, {'char'}, {}, 'canonicalizeName', 'name');

            pattern = '^[\~\/A-Za-z][\w\/_]*$';

            if isempty(name)
                validName = '';
                return
            end

            if isempty(regexp(name,pattern,'once'))
                newEx = ros.internal.ROSException(message('ros:mlros:util:NameInvalid', ...
                                                          'graph', name));
                throw(newEx);
            end

            while ~strcmp(name, ros.internal.Namespace.ROOT) && (name(end) == ros.internal.Namespace.ROOT)
                name = name(1:end-1);
            end


            if startsWith(name,'~/')
                name = "~" + name(3:end);
            end

            validName = char(name);
        end

        function valid = isValidGraphName(name)
        %isValidGraphName Determines if the name is a valid graph name
        %   A valid ROS graph name has the following characteristics:
        %   1. First character is an alpha character ([a-z|A-Z]),
        %      tilde (~) or forward slash (/)
        %   2. Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]),
        %      underscores (_), or forward slashes (/)
        %   Empty graph names are invalid.
        %   See http://wiki.ros.org/Names for more information.
        %
        %   Examples:
        %      import ros.internal.Namespace.isValidGraphName
        %
        %      VALID = isValidGraphName('/position/pos') returns TRUE
        %      VALID = isValidGraphName('~private') returns TRUE
        %      VALID = isValidGraphName('/with space') returns FALSE
        %      VALID = isValidGraphName('/spe%20cial') returns FALSE

            persistent pattern
            if isempty(pattern)
                pattern = '^[\~\/A-Za-z][\w\/]*$';
            end

            valid = false;

            try
                validateattributes(name, {'char'}, ...
                                   {'nonempty'}, 'isValidGraphName', 'name');
            catch
                return;
            end

            if ~isempty(regexp(name, pattern, 'once'))
                valid = true;
            end
        end

        function valid = isValidFieldName(name)
        %isValidFieldName Determines if the name is a valid field name
        %   A field is a variable or constant in a ROS message
        %   definition.
        %   A valid field name has the following characteristics:
        %   1. First character is an alpha character ([a-z|A-Z]).
        %   2. Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]),
        %      or underscores (_)
        %   Empty field names are invalid.
        %   See http://wiki.ros.org/msg for more information.
        %
        %   Examples:
        %      import ros.internal.Namespace.isValidFieldName
        %
        %      VALID = isValidFieldName('position') returns TRUE
        %      VALID = isValidFieldName('7position') returns FALSE
        %      VALID = isValidFieldName('CaMeLCaSe5592') returns TRUE
        %      VALID = isValidFieldName('with space') returns FALSE

            persistent pattern
            if isempty(pattern)
                pattern = '^[A-Za-z][\w]*$';
            end

            valid = false;

            try
                validateattributes(name, {'char'}, ...
                                   {'nonempty'}, 'isValidFieldName', 'name');
            catch
                return;
            end

            if ~isempty(regexp(name, pattern, 'once'))
                valid = true;
            end
        end

        function nodeName = resolveNodeName(name)
        %ros.internal.Namespace.resolveNodeName Resolve a node name
        %   This canonicalize the node name and returns it. Throws an
        %   error if the node name does not correspond to ROS naming
        %   standards

            validateattributes(name, {'char'}, ...
                               {'nonempty'}, 'resolveNodeName', 'name');

            try
                %Canonicalize node name and get current Namespace and Parent Namespace of the node
                name = ros.internal.Namespace.canonicalizeName(name);

                %Check the Namespace environment
                rosNamespace = getenv('ROS_NAMESPACE');
                rosNamespace = ros.internal.Namespace.canonicalizeName(rosNamespace);

                parentNs = ros.internal.Namespace.parentNamespace(name);

                %Name is private
                if name(1) == '~'
                    newEx = ros.internal.ROSException(message('ros:mlros:node:NodeNameResolveError', name));
                    throw(newEx);
                end

                %Name is relative with no parent namespace
                if isempty(parentNs)
                    if name(1) ~= ros.internal.Namespace.ROOT
                        name = strcat(ros.internal.Namespace.ROOT,name);
                    end
                    % Append the current Namespace to node name.
                    graphName = strcat(rosNamespace, name);
                    %Name is global
                elseif parentNs(1) == ros.internal.Namespace.ROOT
                    graphName = name ;
                    %Name is relative with parent namespace
                else
                    if isempty(rosNamespace)
                        graphName = name;
                    elseif rosNamespace(end) ~= ros.internal.Namespace.ROOT
                        rosNamespace = strcat(rosNamespace, ros.internal.Namespace.ROOT);
                        graphName = strcat(rosNamespace, name);
                    end
                end

                if graphName(1) ~= ros.internal.Namespace.ROOT
                    graphName = strcat(ros.internal.Namespace.ROOT,graphName);
                end

                nodeName = char(graphName);
            catch ex
                newEx = ros.internal.ROSException(message('ros:mlros:node:NodeNameResolveError', name));
                throw(newEx);
            end
        end

        function parentNs = parentNamespace(graphName)
        %parentNamespace Extract parent namespace from ROS graph name
        %   PARENTNS = parentNamespace(GRAPHNAME) parses the ROS graph
        %   name GRAPHNAME and returns the parent namespace in
        %   PARENTNS. If the given graph name does not have a parent
        %   namespace, PARENTNS is set to the input. The GRAPHNAME can
        %   be absolute, relative, or private.
        %   An error will be displayed if GRAPHNAME is not a valid ROS
        %   graph name.
        %
        %   Example:
        %      import ros.internal.Namespace.parentNamespace
        %
        %      % Absolute graph name. The function returns "/a".
        %      parentNamespace('/a/b')
        %
        %      % Relative graph name. The function returns "rel/a".
        %      parentNamespace('rel/a/b')
        %
        %      % The graph name does not have a parent namespace.
        %      % The function returns "/"
        %      parentNamespace('/')

            name = ros.internal.Namespace.canonicalizeName(graphName);

            if isempty(name)
                parentNs = '';
            elseif strcmp(name,ros.internal.Namespace.ROOT)
                parentNs = ros.internal.Namespace.ROOT;
            else
                idx = strfind(name,ros.internal.Namespace.ROOT);

                if isempty(idx)
                    parentNs = '';
                else
                    idx = idx(end);
                    if idx > 1
                        parentNs = name(1:idx-1);
                    elseif name(1) == ros.internal.Namespace.ROOT
                        parentNs = ros.internal.Namespace.ROOT;
                    end
                end
            end
        end
    end
end
