classdef GazeboBlankMessageBlockMask < robotics.slgazebo.internal.block.BlankMsgTypeSelectionMask
%This class is for internal use only. It may be removed in the future.

%GazeboBlankMessageBlockMask - Block mask callbacks for Blank Message block

%   Copyright 2019-2021 The MathWorks, Inc.

    properties

        %BusUtil Create the bus util object
        BusUtil = robotics.slcore.internal.bus.Util

    end

    methods

        function updateSubsystem(obj,msgType,block)

            busNamePrefix = obj.BusUtil.BusNamePrefix;

            [~, busName] = robotics.slcore.internal.bus.Util.messageTypeToDataTypeStr(msgType, bdroot(block),busNamePrefix);

            set_param(block, 'busName', ['''' busName '''']);
        end

    end

    methods(Static)

        function dispatch(methodName, varargin)
            obj = robotics.slgazebo.internal.block.GazeboBlankMessageBlockMask();
            obj.(methodName)(varargin{:});
        end

    end
end
