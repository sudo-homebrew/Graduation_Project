classdef MLIfNode < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLIfNode returns the data contained in a If condition.

%Copyright 2019 The MathWorks, Inc.

    properties
        expression
    end

    methods
        function self = MLIfNode(token)
        %Returns the data which is stored in between the TOKEN_START
        %   and TOKEN_END.

            pattern = [ros.internal.emitter.MLCompiler.TokenStart '\s?if\s*(.*?)\s*' ros.internal.emitter.MLCompiler.TokenEnd];
            matchToken = regexp(token, pattern, 'tokens');
            self.expression = strtrim(matchToken{1});
        end

        function out = render(self, data)
        %Returns the data which is stored in If condition.

            out = self.evaluate(self.expression{:}, data);
            if (out == false)
                out = [];
            else
                out = cellfun(@(x) x.render(data), self.children, 'Uniform', false);
                out = char({[out{:}]});
            end
        end
    end
end
