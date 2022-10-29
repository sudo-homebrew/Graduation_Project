classdef MLRoot < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLRoot returns complete data structure to be stored in output file.

%Copyright 2019 The MathWorks, Inc.


    methods
        function out = render(self, data)
        %Renders the complete data stored in root.

            renderedChildren = cellfun(@(x) x.render(data), self.children, 'UniformOutput', false);
            renderedChildren = renderedChildren(~cellfun('isempty', renderedChildren));
            out = strjoin(renderedChildren, '');
        end
    end
end
