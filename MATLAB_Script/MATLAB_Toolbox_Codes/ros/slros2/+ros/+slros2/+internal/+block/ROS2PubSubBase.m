classdef (Abstract) ROS2PubSubBase < ros.slros.internal.block.ROSPubSubBase
%#codegen

%   Copyright 2019-2021 The MathWorks, Inc.

    properties (Nontunable)
        %QOSHistory ros:slros2:blockmask:QOSHistoryPrompt
        QOSHistory = getString(message('ros:slros2:blockmask:QOSKeepLast'));

        % QOSDepth ros:slros2:blockmask:QOSDepthPrompt
        QOSDepth = 1;

        % QOSReliability ros:slros2:blockmask:QOSReliabilityPrompt
        QOSReliability = getString(message('ros:slros2:blockmask:QOSBesetEffort'));

        % QOSDurability ros:slros2:blockmask:QOSDurabilityPrompt
        QOSDurability = getString(message('ros:slros2:blockmask:QOSVolatile'));
    end

    % properties(Constant, Nontunable) % for future use
    %   QOSProfiles : Predefined QOS Profiles
    % end

    properties (Constant, Hidden)
        ROSVersion = 'ROS2';
        QOSHistorySet =  matlab.system.StringSet({message('ros:slros2:blockmask:QOSKeepLast').getString...
                            message('ros:slros2:blockmask:QOSKeepAll').getString});
        QOSReliabilitySet =  matlab.system.StringSet({message('ros:slros2:blockmask:QOSBesetEffort').getString...
                            message('ros:slros2:blockmask:QOSReliable').getString});
        QOSDurabilitySet =  matlab.system.StringSet({message('ros:slros2:blockmask:QOSTransient').getString...
                            message('ros:slros2:blockmask:QOSVolatile').getString});

        ROS2NodeConst = ros.slros2.internal.cgen.Constants.NodeInterface;
    end

    properties(Constant,Access=protected)
        % Name of header file with declarations for variables and types
        % referred to in code emitted by setupImpl and stepImpl.
        HeaderFile = 'ros2nodeinterface.h'
    end

    methods (Hidden, Access = protected)
        function flag = isInactivePropertyImpl(obj,propertyName)
            switch(propertyName)
              case 'QOSDepth'
                if strcmp(obj.QOSHistory, coder.const(DAStudio.message('ros:slros2:blockmask:QOSKeepAll')))
                    flag = true;
                else
                    flag = false;
                end
              otherwise
                flag = false;
            end
        end
    end
    % public setter/getter methods
    methods

        function ret = getQOSArguments(obj)
            ret = {'History', lower(regexprep(obj.QOSHistory, '\s','')), ...
                   'Depth', obj.QOSDepth, ...
                   'Reliability', lower(regexprep(obj.QOSReliability, '\s','')), ...
                   'Durability',lower(regexprep(obj.QOSDurability, '\s',''))};
        end

        function obj = ROS2PubSubBase(varargin)
            coder.allowpcode('plain');
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end

        function set.QOSDepth(obj, val)
            validateattributes(val, ...
                               {'numeric'}, {'nonnegative', 'scalar'}, '', 'QOSDepth');
            obj.QOSDepth = val;
        end
    end

    methods(Static, Hidden)
        function setQOSProfile(rmwProfile, qosHist, qosDepth, qosReliability, qosDurability)
        % SETQOSPROFILE Set the QoS profile values as specified in the
        % Publish/Subscribe block
        % This method uses the enumerations for history, durability and
        % reliability values as specified in 'rmw/types.h' header.
            coder.extrinsic("message");
            coder.extrinsic("getString");

            opaqueHeader = {'HeaderFile', 'rmw/types.h'};
            if isequal(qosHist, coder.const(getString(message('ros:slros2:blockmask:QOSKeepAll'))))
                history = coder.opaque('rmw_qos_history_policy_t', ...
                                       'RMW_QOS_POLICY_HISTORY_KEEP_ALL', opaqueHeader{:});
            else
                history = coder.opaque('rmw_qos_history_policy_t', ...
                                       'RMW_QOS_POLICY_HISTORY_KEEP_LAST', opaqueHeader{:});
            end
            if isequal(qosReliability, coder.const(getString(message('ros:slros2:blockmask:QOSReliable'))))
                reliability = coder.opaque('rmw_qos_reliability_policy_t', ...
                                           'RMW_QOS_POLICY_RELIABILITY_RELIABLE', opaqueHeader{:});
            else
                reliability = coder.opaque('rmw_qos_reliability_policy_t', ...
                                           'RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT', opaqueHeader{:});
            end
            if isequal(qosDurability, coder.const(getString(message('ros:slros2:blockmask:QOSTransient'))))
                durability = coder.opaque('rmw_qos_durability_policy_t', ...
                                          'RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL', opaqueHeader{:});
            else
                durability = coder.opaque('rmw_qos_durability_policy_t', ...
                                          'RMW_QOS_POLICY_DURABILITY_VOLATILE', opaqueHeader{:});
            end
            depth = cast(qosDepth,'like',coder.opaque('size_t','0'));

            % Use SET_QOS_VALUES macro in <model name>_common.h to set the
            % structure members of rmw_qos_profile_t structure, The macro takes in
            % the rmw_qos_profile_t structure and assigns the history, depth,
            % durability and reliability values specified on system block to the
            % qos_profile variable in generated C++ code.
            if coder.target('Rtw')
                coder.ceval('SET_QOS_VALUES', rmwProfile, history, depth, ...
                            durability, reliability);
            end
        end
    end

end
