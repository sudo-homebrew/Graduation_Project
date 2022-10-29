classdef WriteImageBlockMask
%This class is for internal use only. It may be removed in the future.

%WriteImageBlockMask - Block mask callbacks for Write Image block

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS Write Image'
        %MessageType - Message type used by block
        %   Retrieve is with get_param(gcb, 'MessageType')
        MessageType = 'sensor_msgs/Image'
    end

    methods
        function maskInitialize(~, block)
            ros.internal.block.InitWriteImage(block);
            set_param(block, 'MaskType', ros.slros.internal.block.WriteImageBlockMask.MaskType);
            set_param(block, 'MessageType', ros.slros.internal.block.WriteImageBlockMask.MessageType);

            blkH = get_param(block, 'handle');
            ros.internal.setBlockIcon(blkH, 'rosicons.robotlib_writeimage');
        end
    end

    methods (Static)

        function dispatch(methodName, varargin)
            obj = ros.slros.internal.block.WriteImageBlockMask();
            obj.(methodName)(varargin{:});
        end
    end

end
