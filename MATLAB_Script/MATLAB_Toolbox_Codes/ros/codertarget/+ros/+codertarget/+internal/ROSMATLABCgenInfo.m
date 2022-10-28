classdef ROSMATLABCgenInfo < handle
%This class is for internal use only. It may be removed in the future.

%  ROSMATLABCgenInfo is a utility class that encapsulates information about
%  ROS use in a MATLAB function used for codegen.
%

%   Copyright 2020-2021 The MathWorks, Inc.

    properties (SetAccess = private)
        % The name of the ROS node corresponding to the MATLAB function
        NodeName

        % Codegen folder
        CodegenFolder

        % List of top-level message types in the model.
        % This list may have duplicates, and it *does not* include
        % nested message types.
        MessageTypes = {}

        % List of messages containing 64-bit integer type
        MessageTypesWithInt64 = {}

        % List of service types, this list may have duplicates
        ServiceTypes = {}

        % Info about ros.Subscriber classes
        SubscriberList = struct.empty

        % Info about ros.Publisher classes
        PublisherList = struct.empty

        % Info about nodes
        NodeList = {}
    end

    methods (Access = private)
        % Private constructor to prevent explicit object construction
        function obj = ROSMATLABCgenInfo
        end
    end

    %% Singleton class access method
    methods (Static)
        function obj = getInstance
            persistent instance__
            if isempty(instance__)
                instance__ = ros.codertarget.internal.ROSMATLABCgenInfo();
            end
            obj = instance__;
        end
    end

    methods
        function reset(obj)
        % Must be called at onBuildEntry hook. Resets all codegen
        % information.
            obj.SubscriberList = struct.empty;
            obj.PublisherList = struct.empty;
            obj.NodeName = '';
            obj.MessageTypes = {};
            obj.ServiceTypes = {};
            obj.MessageTypesWithInt64 = {};
            obj.NodeList = {};
        end

        function addMessageType(obj,messageType)
            obj.MessageTypes{end+1} = messageType;
        end

        function addServiceType(obj,serviceType)
            obj.ServiceTypes{end+1} = serviceType;
        end

        function addMessageTypeWithInt64(obj,messageType)
            obj.MessageTypesWithInt64{end+1} = messageType;
        end

        function addSubscriber(obj,topic,messageType,cppMessageType)
            temp = struct('Topic',topic,...
                          'MessageType',messageType,...
                          'CppMessageType',cppMessageType);
            if isempty(obj.SubscriberList)
                obj.SubscriberList = temp;
            else
                obj.SubscriberList(end+1) = temp;
            end
        end

        function addPublisher(obj,topic,messageType,cppMessageType)
            temp = struct('Topic',topic,...
                          'MessageType',messageType,...
                          'cppMessageType',cppMessageType);
            if isempty(obj.PublisherList)
                obj.PublisherList = temp;
            else
                obj.PublisherList(end+1) =  temp;
            end
        end

        function addNode(obj,name)
            obj.NodeList{end+1} = name;
        end

        function setNodeName(obj,nodeName)
            obj.NodeName = nodeName;
        end

        function setCodegenFolder(obj,codegenFolder)
            obj.CodegenFolder = codegenFolder;
        end

        function nodes = getNodes(obj)
            nodes = obj.NodeList;
        end

        function msgTypes = getMessageTypes(obj)
        %getMessageTypes Returns the unique top-level message types in model
            msgTypes = unique(obj.MessageTypes);
        end

        function svcTypes = getServiceTypes(obj)
        %getServiceTypes Returns the unique top-level service types in
        %model
            svcTypes = unique(obj.ServiceTypes);
        end

        function msgTypes = getMessageTypesWithInt64(obj)
        %getMessageTypesWithInt64 Returns message types containing 64-bit
        %integer types
            msgTypes = unique(obj.MessageTypesWithInt64);
        end

        function info = getNodeDependencies(obj)
        %   INFO = getNodeDependencies(OBJ) returns the set of ROS
        %   packages required by this model. This includes the packages
        %   required for message and service types present in the model, as
        %   well as any additional packages required by the blocks in
        %   the model.

        % Get the dependencies for message types
            info = ros.slros.internal.cgen.getNodeDependencies(obj.MessageTypes);
            info.nodeDependencies = unique(info.nodeDependencies);
        end

        function [baseName, namespace] = getNodeNameParts(~, name)
            %nodeNameParts Split name into namespace and node name
            %   Requires fully qualified name as input
            %   Assumes name is a character vector and non-empty

            % Extract name as last text block separated by slash
            nameSplit = strsplit(name, '/', 'CollapseDelimiters', false);
            namespace = strjoin(nameSplit(1:end-1), '/');
            baseName = nameSplit{end};
        end
    end
end

% LocalWords: getNodeDependencies
