classdef ROSMaster
%This class is for internal use only. It may be removed in the future.

%  ROSMaster is an interface class encapsulating interactions with ROS
%  Master in simulation. It establishes the connection using a
%  specified NetworkAddrProfile, or (by default) gets the Standard
%  NetworkAddrProfile from MATLAB Preferences.

%   Copyright 2014-2018 The MathWorks, Inc.

    properties
        TimeoutInSeconds = ros.internal.Settings.DefaultMasterConnectionTimeout
    end

    properties(SetAccess=private)
        MasterURI = ''
        Profile = ros.slros.internal.sim.NetworkAddrProfile.empty
    end

    methods

        function obj = ROSMaster(profile)
            if exist('profile', 'var')
                validateattributes(profile, {'ros.slros.internal.sim.NetworkAddrProfile'}, {'scalar'});
                obj.Profile = profile;
            else
                netAddrStore = ros.slros.internal.sim.NetworkAddrStore;
                obj.Profile = netAddrStore.getProfile();
            end

            % generateMasterURI will error if Hostname/IPAddress is not
            % syntactically correct or if the hostname cannot be resolved.
            % However, it won't check if the ROS master is reachable

            if obj.Profile.MasterUseDefault
                % The true flag indicates that console output should be suppressed
                obj.MasterURI = ros.internal.Net.generateMasterURI(true);
            else
                % The true flag (to suppress console output) is not really
                % needed here since explicit arguments are passed in, but
                % we pass it in for consistency with above case.
                obj.MasterURI = ros.internal.Net.generateMasterURI(...
                    obj.Profile.MasterHost, obj.Profile.MasterPort, true);
            end
        end


        function isReachable = isReachable(obj)
            isReachable  = ros.internal.NetworkIntrospection.isMasterReachable(...
                obj.MasterURI, obj.TimeoutInSeconds);
        end


        function verifyReachable(obj)
            if ~obj.isReachable()
                ex = MSLException([], message('ros:slros:ros:ROSMasterNotReachable', obj.MasterURI));
                throw(ex);
            end
        end


        function [topics, msgtypes] = getTopicNamesTypes(obj)
        %getTopicNamesTypes Retrieve list of published topics and their types
        %   This function can timeout if Master is not reachable. It is
        %   the caller's responsibility to check reachability with
        %   verifyReachable() or isReachable()

            [topics, msgtypes] = ...
                ros.internal.NetworkIntrospection.getPublishedTopicNamesTypes(obj.MasterURI);
        end

        function [serviceNames, serviceTypes] = getServiceNamesTypes(obj)
        %getServiceNamesTypes Retrieve list of registered services and their types
        %   This function can timeout if Master is not reachable. It is
        %   the caller's responsibility to check reachability with
        %   verifyReachable() or isReachable()

            serviceAndNodes = ros.internal.NetworkIntrospection.getServiceList(obj.MasterURI);

            % Sort list of service names alphabetically
            serviceNames = sort(serviceAndNodes(:,1));
            serviceTypes = cell(length(serviceNames), 1);

            % Retrieve type for each service separately. Note that this can
            % take a bit of time, since we have to connect to each service
            % server separately.
            for i = 1:length(serviceNames)
                svcName = serviceNames{i};
                serviceTypes{i} = ros.internal.NetworkIntrospection.getServiceType(svcName, obj.MasterURI);
            end
        end

        function [params, paramTypes] = getParameterNamesTypes(obj)
        % This function can timeout if Master is not reachable. It is
        % the caller's responsibility to check reachability with
        % verifyReachable() or isReachable()

        % To access the parameter server, we have to create a new ROS
        % node. It will be deleted when this function exits.
            [~,nodeName] = fileparts(tempname);
            node = obj.createNode(nodeName);
            ptree = ros.ParameterTree(node);

            % Get the root namespace as a struct. This contains the nested
            % tree and all parameter values.
            % This is much faster than retrieving the parameters one-by-one
            % to determine their data type.
            paramDict = ptree.get('/');
            params = ptree.AvailableParameters;
            paramTypes = cell(numel(params), 1);

            % Determine the data type for all parameters
            for i = 1:numel(params)
                paramName = params{i};
                paramElems = strsplit(paramName(2:end), '/');

                % Access the structure according to the namespace elements
                p = paramDict;
                for j = 1:numel(paramElems)
                    p = p.(paramElems{j});
                end

                % Store the parameter type
                paramTypes{i} = class(p);
            end
        end


        function node = createNode(obj, name)
            validateattributes(name, {'char'}, {'nonempty'});

            canonicalName = ros.internal.Namespace.canonicalizeName(name);
            if obj.Profile.NodeUseDefault
                node = ros.Node(canonicalName, obj.MasterURI);
            else
                node = ros.Node(canonicalName, obj.MasterURI, ...
                                'NodeHost', obj.Profile.NodeHost);
            end
        end


    end


    methods(Static)

        function newName = makeUniqueName(name)
        % Using the model name as the node name runs into some issues:
        %
        % 1) There are 2 MATLAB sessions running the same model
        %
        % 2) A model registers a node with ROS Master on model init
        %    and clears the node on model termination. In some cases, the ROS
        %    master can hold on to the node name even if the node
        %    itself (in ROSJAVA) is cleared. This causes a problem
        %    during model init on subsequent simulation runs.
        %
        % To avoid these kinds of issues, we randomize the node name
        % during simulation.

            newName = [name '_' num2str(randi(1e5,1))];
        end
    end
end
