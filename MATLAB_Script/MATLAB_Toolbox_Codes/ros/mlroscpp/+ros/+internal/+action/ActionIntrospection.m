classdef ActionIntrospection < ...
        robotics.core.internal.mixin.Unsaveable & ...
        ros.internal.action.IActionIntrospection
    %This class is for internal use only. It may be removed in the future.
    
    %ActionIntrospection Class retrieves information about actions in the ROS network
    %
    %   ActionIntrospection properties:
    %      MasterURI - The ROS master that is used to query actions
    %
    %   ActionIntrospection methods:
    %      actionList  - Get list of actions in ROS network
    %      actionType  - Get type of action in ROS namespace
    %      actionInfo  - Get detailed information about action in ROS namespace
    %      infoStructToString - Convert the action information structure to a string
    
    %   Copyright 2016-2020 The MathWorks, Inc.
    
    properties (SetAccess = private)
        %MasterURI - URI of ROS Master to which node is connected
        MasterURI = ''
    end
    
    properties (Access = ?matlab.unittest.TestCase)
        %NetworkIntrospection - Object used for getting information about ROS network
        NetworkIntrospection = ros.internal.NetworkIntrospection
        
        %Namespace - Object used for manipulating ROS namespaces
        Namespace = ros.internal.Namespace
    end
    
    properties (Transient, Access = ?matlab.unittest.TestCase)
        %PublishedTopicNames - Storage for published topic names
        PublishedTopicNames
        
        %PublishedTopicTypes - Storage for message types of published topic names
        %   The cell array will have the same size as PublishedTopicNames.
        PublishedTopicTypes
        
        %NodeWeakRef - A weak reference to the parent node.
        NodeWeakHndl
    end
    
    methods
        function obj = ActionIntrospection(node)
            %ActionIntrospection Standard Constructor
            %   The object will use the provided ROS node to connect to the
            %   ROS network and get information about actions.
            %   If NODE is empty, the global node will be used.
            
            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end
            
            obj.NodeWeakHndl = matlab.internal.WeakHandle(node);
            
            obj.MasterURI = node.MasterURI;
        end
        
        function namespaceList = actionList(obj)
            %actionList Get list of actions in ROS network
            %   NAMESPACELIST = actionList(OBJ) returns a string array of
            %   namespaces NAMESPACELIST that correspond to actions in the
            %   ROS network.
            
            % Store names and message types of all published topics
            obj.storeTopicNamesTypes;
            
            % Extract unique namespaces from topic names
            allTopicNamespaces = cellfun(@(topicName) obj.Namespace.parentNamespace(topicName), ...
                obj.PublishedTopicNames, 'UniformOutput', false);
            uniqueNamespaces = unique(allTopicNamespaces);
            
            % Find the namespaces that are associated with actions
            isActNs = cellfun(@(ns) obj.isActionNamespace(ns), uniqueNamespaces);
            
            % Return list of action namespaces. The list will be sorted,
            % since unique sorts automatically.
            namespaceList = uniqueNamespaces(isActNs);
        end
        
        function actType = actionType(obj, nameSpace)
            %actionType Get type of action in ROS namespace
            %   ACTTYPE = actionType(OBJ, NAMESPACE) retrieves the action
            %   type, ACTTYPE, for an action in the ROS NAMESPACE.
            %   If NAMESPACE does not correspond to an action, and error will
            %   be displayed.
            
            % Store names and message types of all published topics
            obj.storeTopicNamesTypes;
            
            % Verify that this is an action namespace and retrieve action type
            [isAction, actType] = isActionNamespace(obj, nameSpace);
            
            % Display an error if the namespace does not refer to an action
            assert(isAction, message('ros:mlros:action:NotActionNamespace', nameSpace));
        end
        
        function infoStruct = actionInfo(obj, nameSpace)
            %actionInfo Get detailed information about action in ROS namespace
            %   INFOSTRUCT = actionInfo(OBJ, NAMESPACE) retrieves a
            %   structure containing detailed information about the action.
            %   INFOSTRUCT is a structure containing the following fields:
            %   - ActionType: Type of the action
            %   - GoalMessageType: Message type of goal messages sent
            %        by the action client
            %   - FeedbackMessageType: Message type of feedback messages
            %        sent by the action server.
            %   - ResultMessageType: Message type of result messages
            %        sent by the action server.
            %   - ActionServer: Structure with information about the
            %        action server.
            %   - ActionClients: Structure array with information about
            %        all registered action clients.
            %
            %   The structure for 'ActionServer' and 'ActionClients' has
            %   the following fields:
            %   - NodeName: Name of the node containing the action server or client
            %   - URI: The URI of the node containing the action server or client
            
            
            % Store names and message types of all published topics
            obj.storeTopicNamesTypes;
            
            % Pre-allocate return structure
            infoStruct = struct('ActionType', '', ...
                'GoalMessageType', '', ...
                'FeedbackMessageType', '', ...
                'ResultMessageType', '', ...
                'ActionServer', struct.empty(0,1), ...
                'ActionClients', struct.empty(0,1) ...
                );
            
            % Verify that this is an action namespace and retrieve action type
            [isAction, infoStruct.ActionType] = isActionNamespace(obj, nameSpace);
            
            % Display an error if the namespace does not refer to an action
            assert(isAction, message('ros:mlros:action:NotActionNamespace', nameSpace));
            
            % Construct expected message types for goal, feedback, and result
            infoStruct.GoalMessageType = [infoStruct.ActionType 'Goal'];
            infoStruct.FeedbackMessageType = [infoStruct.ActionType 'Feedback'];
            infoStruct.ResultMessageType = [infoStruct.ActionType 'Result'];
            
            % Get detailed information about action topics. We are
            % interested in active publishers and subscribers to
            % determine the list of action clients and servers.
            % We verified above that this is a valid action namespace, so
            % all topics should exist

            
            nameSpace = resolveName(obj.NodeWeakHndl.get, char(nameSpace));
            
            goalInfo = obj.NetworkIntrospection.getPublishedTopicInfo(...
                [nameSpace , '/goal'], false, obj.MasterURI);
            cancelInfo = obj.NetworkIntrospection.getPublishedTopicInfo(...
                [nameSpace , '/cancel'], false, obj.MasterURI);
            statusInfo = obj.NetworkIntrospection.getPublishedTopicInfo(...
                [nameSpace , '/status'], false, obj.MasterURI);
            resultInfo = obj.NetworkIntrospection.getPublishedTopicInfo(...
                [nameSpace , '/result'], false, obj.MasterURI);
            feedbackInfo = obj.NetworkIntrospection.getPublishedTopicInfo(...
                [nameSpace , '/feedback'], false, obj.MasterURI);
            
            % Action server subscribes to goal/cancel, and publishes status/result/feedback
            if isempty(goalInfo.Subscribers) || isempty(cancelInfo.Subscribers) || ...
                    isempty(statusInfo.Publishers) || isempty(resultInfo.Publishers) || isempty(feedbackInfo.Publishers)
                actServerNodeNames = {};
            else
                actServerNodeNames = obj.intersectAll({goalInfo.Subscribers.NodeName}, {cancelInfo.Subscribers.NodeName}, ...
                    {statusInfo.Publishers.NodeName}, {resultInfo.Publishers.NodeName}, {feedbackInfo.Publishers.NodeName});
            end
            
            % Actionlib was designed to only have a single action server per namespace,
            % but this code can handle the degenerate condition of multiple
            % action servers.
            infoStruct.ActionServer = obj.populateNodeInfo(actServerNodeNames);
            
            % Action client subscribes to status/result/feedback, and publishes goal/cancel
            if isempty(goalInfo.Publishers) || isempty(cancelInfo.Publishers) || ...
                    isempty(statusInfo.Subscribers) || isempty(resultInfo.Subscribers) || isempty(feedbackInfo.Subscribers)
                actClientNodeNames = {};
            else
                actClientNodeNames = obj.intersectAll({goalInfo.Publishers.NodeName}, {cancelInfo.Publishers.NodeName}, ...
                    {statusInfo.Subscribers.NodeName}, {resultInfo.Subscribers.NodeName}, {feedbackInfo.Subscribers.NodeName});
            end
            
            % Populate information structure with information for each action client
            infoStruct.ActionClients = obj.populateNodeInfo(actClientNodeNames);
        end
        
        function infoString = infoStructToString(~, infoStruct)
            %infoStructToString Convert the action information structure to a string
            %   This string can be used for displaying the information.
            %   
            %   Example string output for one action server and two action
            %   clients:
            %
            %   >> rosaction info /fibonacci
            %   ActionType:          actionlib_tutorials/Fibonacci
            % 
            %   GoalMessageType: 	 actionlib_tutorials/FibonacciGoal
            %   FeedbackMessageType: actionlib_tutorials/FibonacciFeedback
            %   ResultMessageType: 	 actionlib_tutorials/FibonacciResult
            % 
            %   Action Server: 
            %   * /fibonacci (http://172.28.194.77:38647/)
            % 
            %   Action Clients: None
            %   * /node_1 (http://AH-RPILLAT:50986/)
            %   * /some_other_node_name (http://AH-RPILLAT:50994/)
            
            infoWriter = StringWriter;
            
            % Print action type
            infoWriter.addcr([message('ros:mlros:action:InfoType').getString ': ' infoStruct.ActionType]);
            infoWriter.addcr;
            
            % Print message types
            infoWriter.addcr([message('ros:mlros:action:InfoGoalType').getString ': ' infoStruct.GoalMessageType]);
            infoWriter.addcr([message('ros:mlros:action:InfoFeedbackType').getString ': ' infoStruct.FeedbackMessageType]);
            infoWriter.addcr([message('ros:mlros:action:InfoResultType').getString ': ' infoStruct.ResultMessageType]);
            infoWriter.addcr;
            
            % Print information about action server(s)
            infoWriter.add([message('ros:mlros:action:InfoServer').getString ':']);
            if isempty(infoStruct.ActionServer)
                infoWriter.add([' ' message('ros:mlros:action:InfoNone').getString]);
            end
            infoWriter.addcr;
            
            for i = 1:numel(infoStruct.ActionServer)
                infoWriter.addcr(['* ' infoStruct.ActionServer(i).NodeName ...
                    ' (' infoStruct.ActionServer(i).URI ')']);
            end
            infoWriter.addcr;
            
            % Print information about action clients
            infoWriter.add([message('ros:mlros:action:InfoClients').getString ':']);
            if isempty(infoStruct.ActionClients)
                infoWriter.add([' ' message('ros:mlros:action:InfoNone').getString]);
            end
            infoWriter.addcr;
            
            for i = 1:numel(infoStruct.ActionClients)
                infoWriter.addcr(['* ' infoStruct.ActionClients(i).NodeName ...
                    ' (' infoStruct.ActionClients(i).URI ')']);
            end
            
            % Return complete string
            infoString = infoWriter.string;
        end
    end
    
    methods (Access = ?matlab.unittest.TestCase)
        function [isAction, actionType] = isActionNamespace(obj, nameSpace)
            %isActionNamespace Determine if given namespace contains action topics
            %   ISACTION = isActionNamespace(OBJ, NAMESPACE) returns true
            %   if the given ROS NAMESPACE contains the topic associated
            %   with an action.
            %   [ISACTION, ACTIONTYPE} = ____ also returns the type of
            %   action in this NAMESPACE. ACTIONTYPE is empty if ISACTION
            %   is false.
            
            % Resolve absolute name through node. This is a Java GraphName
            % object.

            nameSpace = resolveName(obj.NodeWeakHndl.get, char(nameSpace));
            
            isAction = false;
            actionType = '';
            
            % If any exception occurs, this is not an action namespace
            try
                % For the goal, feedback, and result topics, we can extract the
                % common action type
                actionTypes = cell(3,1);
                
                % Verify that the goal, feedback, and result topics exists
                % and that they have the right message type
                actionTypes{1} = obj.getTypeWithoutSuffix([nameSpace , '/goal'], 'ActionGoal');
                actionTypes{2} = obj.getTypeWithoutSuffix([nameSpace , '/feedback'], 'ActionFeedback');
                actionTypes{3} = obj.getTypeWithoutSuffix([nameSpace , '/result'], 'ActionResult');
                
                % The action type (without suffix) should be the same
                % across the three topics. If not, this is not an action.
                uniqueTypes = unique(actionTypes);
                if length(uniqueTypes) ~= 1
                    return;
                end
                
                % Also check the cancel and status topics, We can ignore
                % the output, since we are only interested in verifying the
                % message type.
                obj.getTypeWithoutSuffix([nameSpace , '/cancel'], 'actionlib_msgs/GoalID');
                obj.getTypeWithoutSuffix([nameSpace , '/status'], 'actionlib_msgs/GoalStatusArray');
            catch
                % If any exception occurs, return false
                return;
            end
            
            % All checks passed, so this is an action namespace.
            isAction = true;
            actionType = uniqueTypes{1};
        end
        
        function actionType = getTypeWithoutSuffix(obj, topicName, expectedSuffix)
            %getTypeWithoutSuffix Remove suffix from topic type
            %   ACTIONTYPE = getTypeWithoutSuffix(OBJ, TOPICNAME, SUFFIX)
            %   retrieves the message type of the topic with TOPICNAME. The
            %   the type ends in SUFFIX, remove the suffix and return the
            %   resulting string in ACTIONTYPE.
            %
            %   An error is displayed if the topic with TOPICNAME does not
            %   exist, or if it does not end in SUFFIX.
            
            % Get message type for topic. This will throw an error if
            % the topic with the given name does not exist.
            [topicExists, topicIdx] = ismember(topicName, obj.PublishedTopicNames);
            assert(topicExists, message('ros:mlros:topic:TopicNameNotFound', topicName));
            msgType = obj.PublishedTopicTypes{topicIdx};
            
            % Make sure that message type ends with the expected suffix.
            % Throw an error otherwise.
            assert(ros.internal.Parsing.endsWith(msgType, expectedSuffix), ...
                message('ros:mlros:action:UnexpectedSuffix', msgType, expectedSuffix));
            
            actionType = msgType(1:end-length(expectedSuffix));
            if isempty(actionType)
                % If complete string is removed, make sure that return data
                % type is a string.
                actionType = '';
            end
        end
        
        function storeTopicNamesTypes(obj)
            %storeTopicNamesTypes Store published topic names and associated message types
            %   This object data will be used by other internal functions.
            [obj.PublishedTopicNames, obj.PublishedTopicTypes] = obj.NetworkIntrospection.getPublishedTopicNamesTypes(obj.MasterURI);
        end
    end
    
    methods (Access = private)
        function commonElements = intersectAll(~, varargin)
            %intersectAll Calculate set intersection of multiple entities
            %   COMMONELEMENTS = intersectAll(OBJ, VARARGIN) calculates the
            %   set intersection of all entities passed with VARARGIN. It
            %   returns the common entity elements in COMMONELEMENTS.
            
            entities = varargin;
            
            if length(entities) < 2
                commonElements = entities;
                return;
            else
                commonElements = entities{1};
            end
            
            for i = 2:length(entities)
                commonElements = intersect(commonElements, entities{i});
            end
        end
        
        function nodeInfoStruct = populateNodeInfo(obj, nodeNames)
            %populateNodeInfo Create info structure for ROS nodes
            %   NODEINFOSTRUCT = populateNodeInfo(OBJ, NODENAMES) creates a
            %   structure array of node information. Each node in NODENAMES
            %   creates a separate structure in the array.
            %   Each structure has the following fields:
            %   - NodeName - Name of the node
            %   - URI - The URI of the node
            
            nodeInfoStruct = struct.empty(0,1);
            
            % Return empty structure if no node names specified
            if isempty(nodeNames)
                return;
            end
            
            % Create structure array for all node names. Retrieve node URI
            % for each node name.
            numNodes = length(nodeNames);
            nodeInfoStruct = repmat(struct('NodeName', '', 'URI', ''), numNodes, 1);
            for i = 1:numNodes
                nodeInfoStruct(i).NodeName = nodeNames{i};
                nodeInfoStruct(i).URI = obj.NetworkIntrospection.getNodeURI(nodeNames{i}, obj.MasterURI);
            end
        end
    end
    
    methods (Access = protected)
        function onNodeShutdownComplete(obj, ~, ~)
            %onNodeShutdownComplete Called when the Java node finishes shut down
            %   This function overrides the function definition in
            %   ros.internal.mixin.NodeDependent
            %   This callback will be triggered when the node that this
            %   object is attached to shuts down. In this case, the
            %   handle object should be deleted, so it cannot be
            %   used in the workspace and is clearly marked as
            %   non-functional.
            
            obj.delete;
        end
    end
    
end
