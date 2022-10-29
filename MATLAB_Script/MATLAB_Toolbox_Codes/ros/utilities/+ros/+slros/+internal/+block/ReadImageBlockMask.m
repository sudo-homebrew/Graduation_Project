classdef ReadImageBlockMask
%This class is for internal use only. It may be removed in the future.

%ReadImageBlockMask Block mask callbacks for ReadImage block
%   Note that we are not using any custom mask on top of the system
%   object, so ReadImageBlockMask is just used as parameter container.

%   Copyright 2018-2020 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ros.slros.internal.block.ReadImage'
    end
end
