classdef MLVarNode < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLVarNode returns the data stored in the variables.

%Copyright 2019 The MathWorks, Inc.

    properties
        name
    end

    methods
        function self = MLVarNode(token)
        %Get the value, which is stored in the variable node.

            val = token;
            % Remove start & end token symbols if present, and return the
            % data which lies in between them.

            if isequal(val(1:2), ros.internal.emitter.MLCompiler.TokenStart) && ...
                    isequal(val(end-1:end), ros.internal.emitter.MLCompiler.TokenEnd)
                val = val(3:end-2);
            end

            self.name = val;
        end

        function out = render(self, data)
        %Renders the values into the output data.

            out = self.evaluate(self.name, data);
        end
    end
end
