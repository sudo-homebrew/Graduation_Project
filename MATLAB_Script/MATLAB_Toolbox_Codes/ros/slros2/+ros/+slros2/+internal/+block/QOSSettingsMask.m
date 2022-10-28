classdef (Hidden) QOSSettingsMask
% QOSSETINGSMASK Block mask callbacks for QOS Settings for Publish and
% Subscribe blocks

% Copyright 2019-2021 The MathWorks, Inc.

    methods (Static, Hidden)
        function qosHistorySelect(block)
            maskEnables = get_param(block,'MaskVisibilities');
            maskObj = Simulink.Mask.get(block);
            qosDepthIdx = arrayfun(@(x)isequal(x.Name, 'QOSDepth'),maskObj.Parameters);
            if isequal(get_param(block, 'QOSHistory'), message('ros:slros2:blockmask:QOSKeepAll').getString)
                maskEnables{qosDepthIdx} = 'off';
            else
                maskEnables{qosDepthIdx} = 'on';
            end
            set_param(block,'MaskVisibilities', maskEnables);
        end
    end
end
