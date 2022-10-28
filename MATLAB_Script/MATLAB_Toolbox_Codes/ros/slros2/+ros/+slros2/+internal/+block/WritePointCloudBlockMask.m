classdef WritePointCloudBlockMask
%This class is for internal use only. It may be removed in the future.

%WritePointCloudBlockMask - Block mask callbacks for Write Point Cloud block

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS2 Write Point Cloud'
        %MessageType - Message type used by block
        %   Retrieve is with get_param(gcb, 'MessageType')
        MessageType = 'sensor_msgs/PointCloud2'
    end

    methods
        function maskInitialize(~, block)
            ros.internal.block.InitWritePointCloud(block);
            set_param(block, 'MaskType', ros.slros2.internal.block.WritePointCloudBlockMask.MaskType);
            set_param(block, 'MessageType', ros.slros2.internal.block.WritePointCloudBlockMask.MessageType);

            blkH = get_param(block, 'handle');
            ros.internal.setBlockIcon(blkH, 'rosicons.ros2lib_writepointcloud');
        end
    end

    methods (Static)
        function dispatch(methodName, varargin)
            obj = ros.slros2.internal.block.WritePointCloudBlockMask();
            obj.(methodName)(varargin{:});
        end
    end

end
