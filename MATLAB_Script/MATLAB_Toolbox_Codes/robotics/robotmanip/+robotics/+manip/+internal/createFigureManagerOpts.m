function opts = createFigureManagerOpts(initBannerWidgets, enableClearInfoOnMousePress)
% This function is for internal use only and may be removed in a future release

%   Copyright 2021 The MathWorks, Inc.

if nargin < 1
    initBannerWidgets = true;
end

if nargin < 2
    enableClearInfoOnMousePress = true;
end

opts = struct(...
    'InitializeBannerWidgets', initBannerWidgets, ...
    'EnableClearInfoOnMousePress', enableClearInfoOnMousePress);
end