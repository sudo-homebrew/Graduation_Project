classdef ShowHideHelper < handle
%This class is for internal use only. It may be removed in the future.

%SHOWHIDEHELPER Mixin class to provide show/hide utility for app tabs

%   Copyright 2018-2020 The MathWorks, Inc.

    properties
        %TabGroup
        TabGroup

        %Tab
        Tab
    end

    methods

        function show(obj)
        %show
            obj.TabGroup.add(obj.Tab);
        end

        function hide(obj)
        %hide
            result = obj.TabGroup.getChildByTag;
            if ~isempty(result)
                currentTabTags = {result.Tag};
                if ismember(obj.Tab.Tag, currentTabTags)
                    obj.TabGroup.remove(obj.Tab);
                end
            end
        end

    end
end
