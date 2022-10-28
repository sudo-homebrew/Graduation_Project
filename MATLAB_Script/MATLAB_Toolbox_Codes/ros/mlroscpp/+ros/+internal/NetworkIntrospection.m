classdef NetworkIntrospection < ros.internal.mixin.ROSInternalAccess & handle
%This class is for internal use only. It may be removed in the future.

%NetworkIntrospection ROS network introspection
%   Gives access to commonly used functions for
%   introspecting into the ROS network.
%   All methods on this object are defined as static. Since the ROS network
%   state might be changing continuously, it does not make sense to
%   store any data.

%   Copyright 2020 The MathWorks, Inc.


    methods (Access = public, Static)

        function numSubs = getNumberofSubscribers(topicName, masterUri)
            [~,list_subs,~] = ros.internal.NetworkIntrospection.getSystemState(masterUri);
            numSubs = 0;

            for ii = 1:size(list_subs,1)
                if strcmp(topicName,list_subs{ii,1})
                    numSubs = size(list_subs{ii,2},1);
                    break;
                end
            end
        end

        function nodeNames = getNodeNames(masterUri)
        %getNodeNames Returns the names of all nodes
        %   Data is returned as cell array of strings. Note that the nodes
        %   need to have active publishers and subscribers to be included
        %   in this list.
        %
        %   NODENAMES = ros.internal.NetworkIntrospection.getNodeNames('MASTERURI')
        %   returns a cell array of strings and each string represents one
        %   node name.
        %   The MASTERURI specifies the ROS Master address that should
        %   be queried for this information. If MASTERURI is empty, the
        %   information from the global node will be used to determine
        %   the Master location.

            [list_pubs,list_subs,list_services] = ros.internal.NetworkIntrospection.getSystemState(masterUri);

            names = {};
            for jj = 1:size(list_pubs,1)
                list_pub_nodes = cellstr(list_pubs{jj,2});
                for zz = 1:length(list_pub_nodes)
                    names{end+1} = list_pub_nodes{zz}; %#ok<AGROW>
                end
            end

            for jj = 1:size(list_subs,1)
                list_sub_nodes = cellstr(list_subs{jj,2});
                for zz = 1:length(list_sub_nodes)
                    names{end+1} = list_sub_nodes{zz}; %#ok<AGROW>
                end
            end

            for jj = 1:size(list_services,1)
                list_services_nodes = cellstr(list_services{jj,2});
                for zz = 1:length(list_services_nodes)
                    names{end+1} = list_services_nodes{zz}; %#ok<AGROW>
                end
            end
            nodeNames = sort(unique(names));
        end

        function nodeInfo = getNodeInfo(nodeName, masterUri)
        %getNodeInfo Get detailed information about a node by name

        % Make sure that the node exists
            nodeInfo.NodeName = ros.internal.NetworkIntrospection.addLeadingSlash(nodeName);
            nodeInfo.URI = ros.internal.NetworkIntrospection.getNodeURI(nodeInfo.NodeName, masterUri);
            if isempty(nodeInfo.URI)
                error(message('ros:mlros:node:NodeNotExist', nodeInfo.NodeName));
            end

            % Retrieve a list of all published topics
            [topicNames, topicTypes] = ros.internal.NetworkIntrospection.getPublishedTopicNamesTypes(masterUri);
            topicMap = containers.Map;
            for ii = 1:length(topicNames)
                topicMap(topicNames{ii}) = topicTypes{ii};
            end

            % For each topic, see if it is published by this node
            nodeInfo.Publications = struct.empty;
            nodeInfo.Subscriptions = struct.empty;
            nodeInfo.Services = struct.empty;

            [list_pubs,list_subs,~] = ros.internal.NetworkIntrospection.getSystemState(masterUri);
            for ii = 1:size(list_pubs,1)
                pubs = cellstr(list_pubs{ii,2});
                pubsSel = ismember(nodeInfo.NodeName, pubs);
                if pubsSel
                    name = list_pubs{ii,1};
                    type = topicMap(name);
                    nodeInfo.Publications(end+1,1).TopicName = name;
                    nodeInfo.Publications(end,1).MessageType = type;
                end
            end

            for ii = 1:size(list_subs,1)
                subs = cellstr(list_subs{ii,2});
                subsSel = ismember(nodeInfo.NodeName, subs);
                if subsSel
                    name = list_subs{ii,1};
                    type = topicMap(name);
                    nodeInfo.Subscriptions(end+1,1).TopicName = name;
                    nodeInfo.Subscriptions(end,1).MessageType = type;
                end
            end

            % Sort the publications and subscription lists
            if ~isempty(nodeInfo.Publications)
                [~, ind] = sort({nodeInfo.Publications.TopicName});
                nodeInfo.Publications = nodeInfo.Publications(ind);
            end
            if ~isempty(nodeInfo.Subscriptions)
                [~, ind] = sort({nodeInfo.Subscriptions.TopicName});
                nodeInfo.Subscriptions = nodeInfo.Subscriptions(ind);
            end

            % Retrieve a list of services and filter by node name
            serviceNamesNodes = ros.internal.NetworkIntrospection.getServiceList(masterUri);
            svcSel = cellfun(@(x) strcmpi(x, nodeInfo.NodeName), serviceNamesNodes(:,2));
            serviceNames = sort(serviceNamesNodes(svcSel, 1));
            for ii = 1:length(serviceNames)
                name = serviceNames{ii};
                nodeInfo.Services(end+1,1).Name = name;
                nodeInfo.Services(end,1).Type = ros.internal.NetworkIntrospection.getServiceType(name, masterUri);
            end
        end

        function type = getPublishedTopicType(topicName, partialMatching, masterURI)
        %getPublishedTopicType Get the message type for a topic
        %   Note that the topic needs to have active publishers and/or
        %   subscribers for this function to succeed.
        %
        %   TYPE = ros.internal.NetworkIntrospection.getPublishedTopicType('TOPICNAME', PARTIALMATCHING)
        %   returns a string TYPE representing the message type of
        %   messages published on TOPICNAME. If PARTIALMATCHING is
        %   true, the topic name will be resolved even if it only
        %   partially (but uniquely) matches a published topic. If
        %   PARTIALMATCHING is false, only exact topic name matches are
        %   supported.

            if ~exist('masterURI', 'var')
                masterURI = '';
            end

            topicName = ros.internal.Namespace.canonicalizeName(topicName);
            topicName = ros.internal.NetworkIntrospection.addLeadingSlash(topicName);

            [topicNames, topicTypes] = ...
                ros.internal.NetworkIntrospection.getPublishedTopicNamesTypes(masterURI);

            try
                [matchedName, index] = ros.internal.Parsing.matchString(...
                    topicName, topicNames, partialMatching);
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:topic:TopicNameNotFound', topicName));
                throw(newEx.addCustomCause(ex));
            end

            if ~strcmp(matchedName, topicName)
                warning(message('ros:mlros:util:PartialTopicNameMatch', ...
                                topicName, matchedName));
            end

            type = topicTypes{index};
        end

        function type = getTopicTypeWithWait(topic, node)
        %getTopicTypeWithWait Get the message type for an existing topic
        %   Note that the topic needs to have registered publishers and/or
        %   subscribers for this function to succeed.
        %   This function waits for a short time period to ensure that
        %   the topic is published
        %
        %   TOPIC is a string denoting the topic name. If the exact topic
        %   name is found on the ROS master, its TYPE will be
        %   returned.
        %   The NODE object's MasterURI property will be used to
        %   determine the URI of the master.

            waitTimeout = 1;

            try
                % First wait with a short timeout until topic is available.
                ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                             ros.internal.NetworkIntrospection.isTopicPublished(topic, ...
                                                                  node.MasterURI), waitTimeout );

                type = ros.internal.NetworkIntrospection.getPublishedTopicType(topic, false, ...
                                                                                      node.MasterURI);
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:topic:CannotDetermineType', topic));
                throw(newEx.addCustomCause(ex));
            end
        end

        function info = getPublishedTopicInfo(topicName, partialMatching, masterURI)
        %getPublishedTopicInfo Get information for a topic
        %   Note that the topic needs to have active publishers and/or
        %   subscribers for this function to succeed.
        %
        %   INFO = ros.internal.NetworkIntrospection.getPublishedTopicInfo('TOPICNAME', PARTIALMATCHING)
        %   returns an information structure INFO that contains
        %   information about the message type and active publishers
        %   and subscribers for TOPICNAME. If PARTIALMATCHING is
        %   true, the topic name will be resolved even if it only
        %   partially (but uniquely) matches a published topic. If
        %   PARTIALMATCHING is false, only exact topic name matches are
        %   supported

            if ~exist('masterURI', 'var')
                masterURI = '';
            end

            topicName = ros.internal.Namespace.canonicalizeName(topicName);
            topicName = ros.internal.NetworkIntrospection.addLeadingSlash(topicName);

            [list_pubs,list_subs,~] = ros.internal.NetworkIntrospection.getSystemState(masterURI);

            % Get all the pub and sub topics.
            list_pub_topics = list_pubs(:,1);
            list_subs_topics = list_subs(:,1);

            topicNames = [list_pub_topics;list_subs_topics];
            topicNames = unique(topicNames);
            try
                [matchedName, ~] = ros.internal.Parsing.matchString(...
                    topicName, topicNames, partialMatching);
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:topic:TopicNameNotFound', topicName));
                throw(newEx.addCustomCause(ex));
            end

            if ~strcmp(matchedName, topicName)
                warning(message('ros:mlros:util:PartialTopicNameMatch', ...
                                topicName, matchedName));
            end


            info.MessageType = ros.internal.NetworkIntrospection.getPublishedTopicType(matchedName, false, masterURI);
            info.Publishers = struct.empty;
            info.Subscribers = struct.empty;

            matchIndex = find(strcmp(list_pub_topics,matchedName),1);
            if~isempty(matchIndex)
                % List of pub nodes
                pubs = list_pubs(matchIndex,2);
                list_nodes = cellstr(pubs{:});
                for jj = 1:length(list_nodes)
                    info.Publishers(jj,1).NodeName = char(list_nodes(jj));
                    info.Publishers(jj,1).URI = ros.internal.NetworkIntrospection.getNodeURI(char(list_nodes(jj)), masterURI);
                end
            end
            matchIndex = find(strcmp(list_subs_topics,matchedName),1);
            if~isempty(matchIndex)
                % List of pub nodes
                subs = list_subs(matchIndex,2);
                list_nodes = cellstr(subs{:});
                for jj = 1:length(list_nodes)
                    info.Subscribers(jj,1).NodeName = char(list_nodes(jj));
                    info.Subscribers(jj,1).URI = ros.internal.NetworkIntrospection.getNodeURI(char(list_nodes(jj)), masterURI);
                end
            end
        end

        function [topicNamesReturn, topicTypesReturn] = getPublishedTopicNamesTypes(masterUri)
        %getPublishedTopicNamesTypes Returns the names (and optionally types)
        %   of all unique topics as cell array(s) of
        %   strings. Note that there need to be active publishers and
        %   subscribers in order for the topic types to be included in this
        %   list.
        %
        %   Outputs:
        %      names - Cell array of strings of topic names
        %      types - Cell array of strings of topic types


            [list_pubs,list_subs,~] = ros.internal.NetworkIntrospection.getSystemState(masterUri);
            try

                %Get the list of publishers and subscribers that are
                %registered.
                registered_topics = [list_pubs(:,1);list_subs(:,1)];
                registered_topics = sort(unique(registered_topics));

                introspectionNode = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterUri);
                [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(introspectionNode.MasterURI);
                topic_nameandtype= introspection(introspectionNode.InternalNode, ...
                                                 introspectionNode.ServerNodeHandle, ...
                                                 {'executexmlrpc','getTopicTypes',host,port,'matlab'});

                topicType = topic_nameandtype{3};
                topicNames = cell(length(topicType),1);
                topicTypes = cell(length(topicType),1);

                for ii = 1:length(topicType)
                    tt = topicType{ii};
                    % Extract topic name and originating node which is
                    % subscribing
                    topicNames{ii,1} = char(tt{1});
                    topicTypes{ii,1} = char(tt{2});
                end

                topicMap = containers.Map;
                for ii = 1:length(topicNames)
                    topicMap(topicNames{ii}) = topicTypes{ii};
                end
                
                registered_topics = registered_topics(ismember(registered_topics, topicNames));
                
                topicNamesReturn = registered_topics;
                topicTypesReturn = cell(length(topicNamesReturn),1);
                for ii=1:length(registered_topics)
                    topicTypesReturn{ii,1} = topicMap(registered_topics{ii,1});
                end

            catch
                error(message('ros:mlros:node:NoSystemState'));
            end
        end

        function nodeUri = getNodeURI(nodeName, masterURI)
        %getNodeURI Return URI for a given node name
        %
        %   NODEURI = ros.internal.NetworkIntrospection.getNodeURI(NODENAME,
        %   MASTERURI) returns the URI for a given node name. The URI is
        %   recovered by communicating with the ROS Master at
        %   MASTERURI.
        %   The output NODEURI contains the node URI as a string and
        %   will be an empty string if an error occurred.

        % Add a leading slash if the node name does not have one
            nodeName = ros.internal.NetworkIntrospection.addLeadingSlash(nodeName);
            try
                introspectionNode = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterURI);
                [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(introspectionNode.MasterURI);
                nodeuriVal= introspection(introspectionNode.InternalNode, ...
                                          introspectionNode.ServerNodeHandle, ...
                                          {'executexmlrpc','lookupNode',host,port,{'matlab',nodeName}});

                if isequal(nodeuriVal{1},1)
                    nodeUri = nodeuriVal{3};
                else
                    nodeUri = '';
                end
            catch
                nodeUri = '';
            end
        end

        function serviceUri = getServiceURI(serviceName, masterURI)
        %getServiceURI Return URI for a given service name
        %
        %   SERVICEURI = ros.internal.NetworkIntrospection.getServiceURI(
        %   SERVICENAME, MASTERURI) returns the URI for a given service name.
        %   The URI is recovered by communicating with the ROS Master at
        %   MASTERURI.
        %   The output SERVICEURI contains the service URI as a string and
        %   will be an empty string if an error occurred.

        % Add a leading slash if the node name does not have one
            serviceName = ros.internal.NetworkIntrospection.addLeadingSlash(serviceName);

            try
                introspectionNode = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterURI);
                [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(introspectionNode.MasterURI);
                serviceUriVal= introspection(introspectionNode.InternalNode, ...
                                             introspectionNode.ServerNodeHandle, ...
                                             {'executexmlrpc','lookupService',host,port,{'matlab',serviceName}});


                if isequal(serviceUriVal{1},1)
                    serviceUri = serviceUriVal{3};
                else
                    serviceUri = '';
                end
            catch
                serviceUri = '';
            end
        end

        function list = getServiceList(masterURI)
        %getServiceList Return list of advertised services
        %
        %   LIST = ros.internal.NetworkIntrospection.getServiceList(
        %   MASTERURI) returns the list of services that are currently
        %   registered with the Master at MASTERURI. If MASTERURI is
        %   empty, the Master URI of the global node is used.
        %   The output LIST is a Nx2 cell array containing the N
        %   registered service names in its first column and their
        %   corresponding node names in the second column.

            introspectionNode = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterURI);
            [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(introspectionNode.MasterURI);
            try

                systemState= introspection(introspectionNode.InternalNode, ...
                                           introspectionNode.ServerNodeHandle, ...
                                           {'executexmlrpc','getSystemState',host,port,'master'});

                pubsubsvc = systemState(3);
                services = pubsubsvc{1}{3};
                list = cell(length(services),2);
                for ii = 1:length(services)
                    svc = services{ii};

                    % Extract service name and originating node
                    list{ii,1} = char(svc{1});
                    list{ii,2} = char(svc{2});
                end
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:service:ListError'));
                throw(newEx.addCustomCause(ex));
            end
        end

        function info = getServiceInfo(serviceName, masterURI)
        %getServiceInfo Retrieve information on service
        %   The return information structure contains the following
        %   elements:
        %   1. Node: The node that this service is attached to
        %   2. URI: The URI of the service
        %   3. Type: The type of the service
        %   4. Args: The arguments that the request to this service has

            serviceName = ros.internal.NetworkIntrospection.addLeadingSlash(serviceName);

            if ~ros.internal.NetworkIntrospection.isServiceAvailable(serviceName,masterURI)
                error(message('ros:mlros:service:ServiceNameNotFound', ...
                              serviceName));
            end

            % Recover the name of the node that advertised this service
            svcList = ros.internal.NetworkIntrospection.getServiceList(masterURI);
            svcSel = svcList(strcmp(svcList(:,1), serviceName), :);
            info.Node = '';
            if ~isempty(svcSel)
                info.Node = svcSel{2};
            end

            % Recover URI and type of this service
            info.URI = ros.internal.NetworkIntrospection.getServiceURI(serviceName, masterURI);
            info.Type = ros.internal.NetworkIntrospection.getServiceType(serviceName, masterURI);

            % Recover the arguments of the request message for this service
            % by parsing the property list of the MATLAB message object
            reqType = [info.Type, 'Request'];
            reg = ros.internal.ros.getMessageInfo(reqType);
            clnup = []; %#ok<NASGU>
            if reg.custom
                curpath = path;
                clnup = onCleanup(@(x)path(curpath));
                addpath(fullfile(reg.installDir,'m'));
            end
            [msgStruct, ~] = eval(reg.msgStructGen);
            propList = fieldnames(msgStruct);

            info.Args = {};
            for i = 1:numel(propList)
                info.Args{end+1} = propList{i};
            end
        end

        function node = getNodeForMasterURI(masterURI)
        %Helper to create the node only once.
        %if the MasterURI is different, a new node is created
            persistent Node;
            [active, node] = ros.internal.Global.isNodeActive;

            if isempty(masterURI) && ~active
                error(message('ros:mlros:node:GlobalNodeNotRunning'));
            end

            if active && (isempty(masterURI) || isequal(node.MasterURI, masterURI))
                % return the global node
                return;
            end

            if isempty(Node) || ~isequal(Node.MasterURI, masterURI)
                %Make the nodename unique by using current time-stamp.
                nodeName = ['/matlab_introspec_' datestr(now,'ddmmyyyyHHMMSSFFF')];
                Node = ros.Node(nodeName,masterURI);
            end
            node = Node;
        end

        function type = getServiceType(serviceName, masterURI)
        %getServiceType Retrieve type of service
        %   If the service is not available, this call will return the
        %   empty string. If the service is available, but does not
        %   respond to HTTP requests, this call will time out after 5
        %   seconds.

            node = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterURI);

            serviceName = ros.internal.NetworkIntrospection.addLeadingSlash(serviceName);

            type = '';
            serviceURI = ros.internal.NetworkIntrospection.getServiceURI(serviceName, masterURI);
            if isempty(serviceURI)
                return;
            end

            try
                info = introspection(node.InternalNode, ...
                                     node.ServerNodeHandle, ...
                                     {'servicetype', serviceName});
                type = info.type;
            catch ex
                rethrow ex
            end
        end

        function avail = isServiceAvailable(serviceName, masterURI)
        %isServiceAvailable Test connectivity to service
        %
        %   AVAIL = isServiceAvailable('SERVICENAME') returns true if the
        %   given SERVICENAME is currently an active service on the ROS
        %   Master. The function returns false otherwise.

            if ~exist('masterURI', 'var')
                masterURI = [];
            end

            serviceName = ros.internal.NetworkIntrospection.addLeadingSlash(serviceName);

            avail = false;
            uri = ros.internal.NetworkIntrospection.getServiceURI(serviceName, masterURI);
            if isempty(uri)
                return;
            end
            avail = true;
        end

        function hasSvcServer = isServiceOwner(nodeName, serviceName, masterURI)
        %isServiceOwner Checks if a node is the owner of a service server
        %
        %   HASSVCSERVER = isServiceOwner(NODENAME, SERVICENAME,
        %   MASTERURI) queries the ROS master at MASTERURI and
        %   determines if the node with name NODENAME has a registered
        %   service server for the service with name SERVICENAME.
        %   If so, HASSVCSERVER will be 'true'. Otherwise, HASSVCSERVER
        %   will be 'false'.

            if ~exist('masterURI', 'var')
                masterURI = [];
            end

            hasSvcServer = false;

            serviceName = ros.internal.NetworkIntrospection.addLeadingSlash(serviceName);

            % Recover the name of the node that advertised this service
            try
                svcList = ros.internal.NetworkIntrospection.getServiceList(masterURI);
            catch
                % If master is not reachable or service list cannot be
                % retrieved, return.
                return
            end
            svcSel = svcList(strcmp(svcList(:,1), serviceName), :);
            if isempty(svcSel)
                % Service is not advertised, so we can return
                return
            end
            svcNodeName = svcSel{2};

            % Now compare node names. If they are the same then this
            % function can return 'true'.
            if strcmp(svcNodeName, nodeName)
                hasSvcServer = true;
            end
        end

        function pub = isTopicPublished(topicName, masterURI)
        %isTopicPublished Checks if topic is listed as active on Master
        %
        %   PUBLISHED = isTopicPublished('TOPICNAME') returns true if the
        %   given TOPICNAME is currently active on the ROS Master, i.e.
        %   has active subscribers or publishers. The function returns
        %   false otherwise.

            if ~exist('masterURI', 'var')
                masterURI = [];
            end

            pub = false;
            try
                ros.internal.NetworkIntrospection.getPublishedTopicType(topicName, false, masterURI);
            catch
                return;
            end
            pub = true;
        end

        function [hasPub, hasSub] = isTopicParticipant(nodeName, topicName, masterURI)
        %isTopicParticipant Checks if a node is a topic participant
        %
        %   [HASPUB, HASSUB] = isTopicParticipant(NODENAME, TOPICNAME,
        %   MASTERURI) queries the ROS master at MASTERURI and
        %   determines if the node with name NODENAME has publishers or
        %   subscribers registered for topic TOPICNAME. If the node has
        %   active publishers for this topic, HASPUB will be 'true'.
        %   Otherwise, HASPUB will be 'false'. HASSUB will be 'true' if
        %   the node has active subscribers for this topic and 'false'
        %   otherwise.

            hasPub = false;
            hasSub = false;

            topicName = ros.internal.Namespace.canonicalizeName(topicName);
            topicName = ros.internal.NetworkIntrospection.addLeadingSlash(topicName);

            try

                [list_pubs,list_subs,~] = ros.internal.NetworkIntrospection.getSystemState(masterURI);

                % Get all the pub and sub topics.
                list_pubs_topics = list_pubs(:,1);
                list_subs_topics = list_subs(:,1);

                topics = [list_pubs_topics;list_pubs_topics];
                matchIndex = find(strcmp(topics,topicName), 1);
                if isempty(matchIndex)
                    return;
                end
            catch
                % If topic isn't found or master isn't reachable,
                % then this node is not a participant
                return;
            end

            matchIndex = find(strcmp(list_pubs_topics,topicName), 1);
            % List of pub nodes
            if ~isempty(matchIndex)
                pubs = list_pubs(matchIndex,2);
                list_nodes = cellstr(pubs{:});
                for jj = 1:length(list_nodes)
                    if strcmp(char(list_nodes(jj)),nodeName)
                        hasPub = true;
                        break;
                    end
                end
            end
            matchIndex = find(strcmp(list_subs_topics,topicName), 1);
            % List of pub nodes
            if ~isempty(matchIndex)
                subs = list_subs(matchIndex,2);
                list_nodes = cellstr(subs{:});
                for jj = 1:length(list_nodes)
                    if strcmp(char(list_nodes(jj)),nodeName)
                        hasSub = true;
                        break;
                    end
                end
            end
        end
        
        function reachable = isTCPServerReachable(host, port, timeout)
		% Creates a tcp client and tries to reach the TCP server at 
		% given host and port. 
		% "timeout" the maximum time to wait for a connection request to the 
		% specified remote host to succeed or fail.
		
            try 
                tcpClient = matlabshared.network.internal.TCPClient(host,str2double(port));
                tcpClient.ConnectTimeout = timeout;
                tcpClient.connect();
                reachable = true;
                tcpClient.disconnect();
            catch
                reachable = false;
            end
        end
        
        function ret = isValidMaster(uri)
		% Creates a node and confirms a master URI is from a valid rosmaster
        % or not.
		    ret = true;
            try 
                randomUniqNodeName = ['/ml_tmp_node_' datestr(now,'dd_mmm_yyyy_HH_MM_SS_FFF') '_' char(randi([97 122],1,10))];
                ros.Node(randomUniqNodeName,uri);
            catch ex
                if isequal(ex.identifier,'ros:mlros:node:NoMasterConnection')
                    ret = false;
                end
            end
        end
        
        function reachable = isMasterReachable(masterURI, timeout)
        %isMasterReachable Test connectivity to Master
        %   You can also specify the timeout for the connection attempt
        %   with TIMEOUT (in seconds). This is implemented on the
        %   XML-RPC level, since the timeouts in rosjava are hard-coded
        %   to 10 seconds.

            if isequal(nargin,1) || isempty(timeout) || (timeout < 1)
                % Default timeout is 1 second
                timeout = 1;
            end
           
            [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(masterURI);

            if strcmp(host,'localhost')
                host = '127.0.0.1';
            end
            
            reachable = true;
            try
                ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                             ros.internal.NetworkIntrospection.isTCPServerReachable(host, port, timeout), ...
                                                             timeout);
            catch
                reachable = false;
            end
        end

        function [reachable, responseTime] = isNodeReachable(nodeName, masterURI, timeout)
        %isNodeReachable Test connectivity to node
        %   The connectivity is checked by trying to retrieve some data
        %   from the node over XML-RPC.
        %
        %   [REACHABLE, RESPONSETIME] = ros.internal.NetworkIntrospection.isNodeReachable(NODENAME, MASTERURI, TIMEOUT)
        %   returns TRUE if the ROS node with the name NODENAME can be
        %   reached and returns information. If the node name is
        %   invalid, or if the node does not respond to XML-RPC
        %   requests within TIMEOUT seconds, the function will return FALSE.
        %   In order to lookup the node's URI, this function has to
        %   communicate to the ROS Master at MASTERURI.
        %   RESPONSETIME is the time in seconds that it took to get the
        %   response from the node.
        %
        %   If TIMEOUT is empty [], the default timeout of 1 second is
        %   used.

            reachable = true;
            responseTime = 0;

            nodeURI = ros.internal.NetworkIntrospection.getNodeURI(nodeName, masterURI);
            if isempty(nodeURI)
                reachable = false;
                return;
            end

            if isequal(nargin,2) || isempty(timeout) || (timeout < 1)
                % Default timeout is 1 second
                timeout = 1;
            end

            [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(nodeURI);

            try
                xmlTime = tic;
                ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                             ros.internal.NetworkIntrospection.isTCPServerReachable(host, port, timeout), ...
                                                             timeout);
                responseTime = toc(xmlTime);
            catch
                reachable = false;
            end
        end
    end

    methods (Static)
        function slashedName = addLeadingSlash(name)
        %addLeadingSlash Add a leading slash if the name does not have one

            if ~strncmp(name, '/', 1)
                slashedName = ['/' name];
            else
                slashedName = name;
            end
        end
    end

    methods (Access = public, Static)

        function [list_pubs,list_subs,list_services] = getSystemState(masterURI)
        %getNodeNames Returns the names of all nodes
        %   Data is returned as cell array of strings. Note that the nodes
        %   need to have active publishers and subscribers to be included
        %   in this list.
        %
        %   NODENAMES = ros.internal.ROSNetworkIntrospection.getNodeNames;
        %   returns a cell array of strings and each string represents one
        %   node name.
        %node = ros.internal.Global.getNodeHandle(false);
        %ret = node.InternalNode.introspection(node.ServerNodeHandle,{'node','list'});
        %ret = reshape(result,[],1);
        %             h = ros.internal.Introspection;
        %             ret = h.nodelist([], ros.ros2.internal.NetworkIntrospection.getDomainIDForSimulink);

            introspectionNode = ros.internal.NetworkIntrospection.getNodeForMasterURI(masterURI);
            [host, port] = ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(introspectionNode.MasterURI);
            try
                systemState= introspection(introspectionNode.InternalNode, ...
                                           introspectionNode.ServerNodeHandle, ...
                                           {'executexmlrpc','getSystemState',host,port,'master'});

                % Extract the pub,sub and services information
                pubsubsvc = systemState(3);
                % Extract the topic and subscribers
                pubs = pubsubsvc{1}{1};
                list_pubs = cell(length(pubs),2);
                for ii = 1:length(pubs)
                    pub = pubs{ii};
                    % Extract topic name and originating node which is
                    % subscribing
                    list_pubs{ii,1} = char(pub{1});
                    list_pubs{ii,2} = char(pub{2});
                end

                % Extract the topic and subscribers
                subs = pubsubsvc{1}{2};
                list_subs = cell(length(subs),2);
                for ii = 1:length(subs)
                    sub = subs{ii};
                    % Extract topic name and originating node which is
                    % subscribing
                    list_subs{ii,1} = char(sub{1});
                    list_subs{ii,2} = char(sub{2});
                end
                % Extract the service name and service providers
                services = pubsubsvc{1}{3};
                list_services = cell(length(services),2);
                for ii = 1:length(services)
                    svc = services{ii};

                    % Extract service name and originating node
                    list_services{ii,1} = char(svc{1});
                    list_services{ii,2} = char(svc{2});
                end
            catch ex
                error(message('ros:mlros:node:NoSystemState'));
            end
        end

        function [topics, types] = getTopicsTypesWithNode(node)
        %getTopicsTypesWithNode Get topic names and types using ROS node
        %   TOPICS is a cell array specifying topics on the node's network.
        %   TYPES is a cell array of cell arrays containing the
        %   corresponding types for each topic. Each topic may have
        %   more than one message type.

            allTopicsAndTypes = ...
                introspection(node.InternalNode, ...
                              node.ServerNodeHandle, ...
                              {'topic', 'list'});
            topics = allTopicsAndTypes(:, 1);
            types = allTopicsAndTypes(:, 2);
        end

        function type = getTypeFromTopicWithNode(node, topic)
        %getTypeFromTopicWithNode Get type for topic using ROS node
        %   TYPE is a cell array containing the message type(s) used on
        %   the specified TOPIC. There may be more than one message type
        %   per topic .

            type = {};
            [topics, types] = ...
                ros.internal.NetworkIntrospection.getTopicsTypesWithNode(node);
            whichTopic = strcmp(topic, topics);
            if any(whichTopic)
                % In the unlikely event of multiple matching topics on the
                % list, this syntax will use the first
                type = types{whichTopic};
            end
        end

        function [host, port] = getHostAndPortFromMasterURI(masterURI)
        %getHostAndPortFromMasterURI Get Host and Port from given Master URI

            hostandPortWithOuthttp = strsplit(masterURI,'://');
            hostandPort = strsplit(hostandPortWithOuthttp{2},':');
            host = hostandPort{1};
            portWithslash = strsplit(hostandPort{2},'/');
            port = portWithslash{1};
        end
    end
end
