classdef ROSModelInfo
%This class is for internal use only. It may be removed in the future.

%  ROSModelInfo is a utility class that encapsulates information about
%  ROS blocks in a Simulink model.
%
%  See also: cgen.ROSBlockInfo

%   Copyright 2014-2021 The MathWorks, Inc.

    properties(SetAccess=private)
        %Model - Name of Simulink model
        Model

        %LibraryBlocks - List of ROS publish, subscribe, and blank message blocks in the model
        LibraryBlocks

        %ParameterBlocks - List of ROS get / set parameter blocks
        ParameterBlocks

        %ImageBlocks - List of ROS read image blocks
        ImageBlocks

        %TimeBlocks - List of ROS time blocks
        TimeBlocks

        %ServiceCallBlocks - List of ROS service call blocks
        ServiceCallBlocks

        % List of top-level message types in the model (i.e., the
        % message types used in ROS blocks that handle buses).
        % This list may have duplicates, and it *does not* include
        % nested message types.
        TopLevelMsgTypes

        % List of top-level service types in the model (i.e., the
        % service types used in ROS service blocks).
        TopLevelServiceTypes

        % The name of the ROS node corresponding to the Simulink model
        NodeName

        % Info about Subscribe blocks in model (list of cgen.ROSMessageBlockInfo objects)
        SubscriberList = ros.slros.internal.cgen.ROSMessageBlockInfo.empty

        % Info about Publish blocks in model (list of cgen.ROSMessageBlockInfo objects)
        PublisherList = ros.slros.internal.cgen.ROSMessageBlockInfo.empty

        %ParameterGetterList - Info about Get Parameter blocks in model
        %   This is a list of cgen.ROSParameterBlockInfo objects.
        ParameterGetterList = ros.slros.internal.cgen.ROSParameterBlockInfo.empty

        %ParameterSetterList - Info about Set Parameter blocks in model
        %   This is a list of cgen.ROSParameterBlockInfo objects.
        ParameterSetterList = ros.slros.internal.cgen.ROSParameterBlockInfo.empty

        %ServiceCallerList - Info about Call Service blocks in model
        %   This is a list of cgen.ROSServiceBlockInfo objects.
        ServiceCallerList = ros.slros.internal.cgen.ROSServiceBlockInfo.empty
    end
    

    %%
    methods
        function info = augmentBlockInfo(obj, info)
            % AUGMENTBLOCKINFO Create block code-generation meta data map
            % for all the blocks in the given model for use with local
            % build using ProjectTool
            
            info.Publishers     = getContainersMap(obj.PublisherList);
            info.Subscribers    = getContainersMap(obj.SubscriberList);
            info.ParamGetters   = getContainersMap(obj.ParameterGetterList);
            info.ParamSetters   = getContainersMap(obj.ParameterSetterList);
            info.ServiceCallers = getContainersMap(obj.ServiceCallerList);
            if ~isempty(obj.ImageBlocks)
                info.ImageDepends = {'cv_bridge', 'sensor_msgs'};
            end
            %info.TimeGetters    = containers.Map;
            info.MessageTypes   = obj.TopLevelMsgTypes;
            info.ServiceTypes   = obj.TopLevelServiceTypes;
            function mapObj = getContainersMap(itemList)
                mapObj = containers.Map;
                registry = ros.internal.CustomMessageRegistry.getInstance('ros');
                if ~isempty(itemList)
                    for iter=1:numel(itemList)
                        thisItem = itemList(iter);
                        theKey = strrep(strrep(thisItem.Comment, 'For Block ',''),newline,' ');
                        thisStruct = struct('BlockID', thisItem.Label);
                        thisStruct.Comment = thisItem.Comment;
                        if isprop(thisItem, 'SlBusName')
                            thisStruct.BusName = thisItem.SlBusName;
                        end
                        if isprop(thisItem, 'MsgType')
                            thisStruct.msgInfo = ros.internal.ros.getMessageInfo(thisItem.MsgType, registry);
                        end
                        if isprop(thisItem, 'SrvType')
                            thisStruct.msgInfo = ros.internal.ros.getMessageInfo(thisItem.SrvType, registry);
                            thisStruct.Request.msgInfo = ros.internal.ros.getMessageInfo([thisItem.SrvType,'Request'], registry);
                            thisStruct.Response.msgInfo = ros.internal.ros.getMessageInfo([thisItem.SrvType,'Response'], registry);
                        end                        
                        if isprop(thisItem, 'SlInputBusName')
                            thisStruct.InputBusName = thisItem.SlInputBusName;
                        end
                        if isprop(thisItem, 'SlOutputBusName')
                            thisStruct.OutputBusName = thisItem.SlOutputBusName;
                        end
                        if isprop(thisItem, 'IsArray')
                            thisStruct.IsArray = thisItem.IsArray;
                        end      
                        if isprop(thisItem, 'ParamType')
                            thisStruct.ParamType = thisItem.ParamType;
                        end                            
                        if isprop(thisItem, 'CppParamType')
                            thisStruct.CppParamType = thisItem.CppParamType;
                        end                            
                        if isprop(thisItem, 'ROSCppParamType')
                            thisStruct.ROSCppParamType = thisItem.ROSCppParamType;
                        end                            
                        mapObj(theKey) = thisStruct;
                    end
                end
            end
        end
        
        function obj= ROSModelInfo(model)
            obj.Model = model;
            obj.NodeName = model;

            [obj.TopLevelMsgTypes, obj.TopLevelServiceTypes, obj.LibraryBlocks, ...
             obj.ParameterBlocks, obj.ImageBlocks, obj.TimeBlocks, ...
             obj.ServiceCallBlocks] = ...
                ros.slros.internal.bus.Util.getROSBlocksInModel(model);

            obj = getInfoFromROSBlocks(obj);
        end

        function msgTypes = messageTypesInModel(obj)
        %messageTypesInModel Returns the unique top-level message types in model
            msgTypes = unique(obj.TopLevelMsgTypes);
        end

        function svcTypes = serviceTypesInModel(obj)
        %serviceTypesInModel Returns the unique top-level service types in model
            svcTypes = unique(obj.TopLevelServiceTypes);
        end

        function info = getNodeDependencies(obj)
        %   INFO = getNodeDependencies(OBJ) returns the set of ROS
        %   packages required by this model. This includes the packages
        %   required for message and service types present in the model, as
        %   well as any additional packages required by the blocks in
        %   the model.

        % Get the dependencies for message types
            info = ros.slros.internal.cgen.getNodeDependencies( ...
                obj.messageTypesInModel);

            % Add required dependencies for Read Image blocks if necessary.
            % These are used for deserializing image messages
            if ~isempty(obj.ImageBlocks)
                info.nodeDependencies{end+1} = 'cv_bridge';
                info.nodeDependencies{end+1} = 'sensor_msgs';
            end

            % Remove any duplicate entries from the list of dependencies
            info.nodeDependencies = unique(info.nodeDependencies);
        end
    end

    %%
    methods(Access = private)

        function obj = getInfoFromROSBlocks(obj)
            obj.NodeName = obj.Model;

            % Get more information about subscribers and publishers
            [obj.SubscriberList, obj.PublisherList] = ...
                obj.getInfoFromPubSubBlocks;

            % Get more information about parameter blocks
            [obj.ParameterGetterList, obj.ParameterSetterList] = ...
                obj.getInfoFromParameterBlocks;

            % Get more information about service call blocks
            obj.ServiceCallerList = obj.getInfoFromServiceBlocks;
        end

        function [subList, pubList] = getInfoFromPubSubBlocks(obj)
        %getInfoFromPubSubBlocks Iterate over blocks and extract information from pub/sub blocks

            subList = [];
            pubList = [];

            for i=1:numel(obj.LibraryBlocks)
                block = obj.LibraryBlocks{i};
                maskType = get_param(block, 'MaskType');
                if strcmpi(maskType, ros.slros.internal.block.MessageBlockMask.MaskType)
                    % We don't need to store any information for Blank Message blocks
                    continue;
                end

                switch maskType
                  case ros.slros.internal.block.PublishBlockMask.MaskType
                    sysObjBlock = [block '/SinkBlock'];
                  case ros.slros.internal.block.SubscribeBlockMask.MaskType
                    sysObjBlock = [block '/SourceBlock'];

                  otherwise
                    assert(false, sprintf('Unexpected mask type %s for block %s', ...
                                          maskType, block));
                end

                s = ros.slros.internal.cgen.ROSMessageBlockInfo;
                s.MsgType = get_param(block, 'messageType');
                s.SlBusName = get_param(sysObjBlock, 'SLBusName');
                s.Label = get_param(sysObjBlock, 'BlockId');
                s.Comment = sprintf('For Block %s', block);
                s.CppRosType = ros.slros.internal.cgen.Util.rosMsgTypeToCppClass(s.MsgType);

                % consistency check
                expectedBusName = ...
                    ros.slros.internal.bus.Util.rosMsgTypeToBusName(s.MsgType, obj.Model);
                assert(strcmpi(s.SlBusName, expectedBusName), ...
                       sprintf('Mismatch: %s, %s', s.SlBusName, expectedBusName));

                switch maskType
                  case ros.slros.internal.block.PublishBlockMask.MaskType
                    pubList = [pubList s]; %#ok<AGROW>
                  case ros.slros.internal.block.SubscribeBlockMask.MaskType
                    subList = [subList s]; %#ok<AGROW>
                end

            end
        end

        function [paramGetList, paramSetList] = getInfoFromParameterBlocks(obj)
        %getInfoFromParameterBlocks Iterate over blocks and extract information from parameter blocks

            paramGetList = [];
            paramSetList = [];

            for i = 1:numel(obj.ParameterBlocks)
                block = obj.ParameterBlocks{i};
                s = ros.slros.internal.cgen.ROSParameterBlockInfo;

                s.ParamType = get_param(block, 'ParameterType');
                s.Label = get_param(block, 'BlockId');
                s.Comment = sprintf('For Block %s', block);

                % Convert from Simulink to C++ type
                s.CppParamType = ros.slros.internal.sim.DataTypes.simulinkToCpp(s.ParamType);

                % Convert from Simulink to ROS C++ type
                parameterType = get_param(block, 'ParameterType');

                % Decide if this type corresponds to an array or a scalar
                % parameter.
                s.IsArray = ~ros.slros.internal.sim.DataTypes.isSimulinkDataTypeScalar(parameterType);

                s.ROSCppParamType = ros.slros.internal.sim.DataTypes.simulinkToROSCpp(s.ParamType, s.IsArray);

                maskType = get_param(block, 'MaskType');
                switch maskType
                  case ros.slros.internal.block.GetParameterBlockMask.MaskType
                    paramGetList = [paramGetList s]; %#ok<AGROW>
                  case ros.slros.internal.block.SetParameterBlockMask.MaskType
                    paramSetList = [paramSetList s]; %#ok<AGROW>
                end
            end
        end

        function servList = getInfoFromServiceBlocks(obj)
        %getInfoFromServiceBlocks Iterate over blocks and extract information from service blocks

            servList = [];

            for i = 1:numel(obj.ServiceCallBlocks)
                block = obj.ServiceCallBlocks{i};

                sysObjBlock = [block '/ServiceCaller'];

                % Build info structure for service block
                s = ros.slros.internal.cgen.ROSServiceBlockInfo;
                s.SrvType = get_param(block, 'serviceType');
                s.CppRosType = ros.slros.internal.cgen.Util.rosMsgTypeToCppClass(s.SrvType);
                s.SlInputBusName = get_param(sysObjBlock, 'SLInputBusName');
                s.SlOutputBusName = get_param(sysObjBlock, 'SLOutputBusName');
                s.Label = get_param(sysObjBlock, 'BlockId');
                s.Comment = sprintf('For Block %s', block);

                % Consistency check
                [expectedInputBusName, expectedOutputBusName] = ...
                    ros.slros.internal.bus.Util.rosServiceTypeToBusNames(s.SrvType, obj.Model);
                assert(strcmpi(s.SlInputBusName, expectedInputBusName), ...
                       sprintf('Mismatch: %s, %s', s.SlInputBusName, expectedInputBusName));
                assert(strcmpi(s.SlOutputBusName, expectedOutputBusName), ...
                       sprintf('Mismatch: %s, %s', s.SlOutputBusName, expectedOutputBusName));

                servList = [servList s]; %#ok<AGROW>
            end
        end

    end

end

% LocalWords:  AUGMENTBLOCKINFO
