classdef MLForNode < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLForNode returns the data contained in a for loop.

%Copyright 2019-2020 The MathWorks, Inc.

    properties

        expression

        iterator

        fieldname 
        
        value
        
    end

    methods
        function self = MLForNode(token)
        %Returns the data which is stored in between the TOKEN_START
        %   and TOKEN_END.

            pattern = [ros.internal.emitter.MLCompiler.TokenStart '\s?for\s*(.*?)\s*' ros.internal.emitter.MLCompiler.TokenEnd];
            matchToken = regexp(token, pattern, 'tokens');
            self.expression = strtrim(matchToken{1});
            
            a = regexp(self.expression{:},'\s*=\s*','split');
            self.fieldname = a{1};
            self.value = a{2};
        end


        function out = render(self, data)
        %Returns the data contained in For loop.
            
            % Get the count of number of iterations of For loop.
            self.iterator = self.evaluate(self.value, data);
            out = '';

            % Collect the data each time when enters into loop and
            % append it.
            for iterator1 = 1:self.iterator
                data.(ros.internal.emitter.MLTempSpace.TEMPSPACENAME).TempSpace.(self.fieldname) = iterator1;
                out1 = cellfun(@(x) x.render(data), self.children, 'Uniform', false);
                out = [out char({[out1{:}]})]; %#ok<AGROW>
            end
        end
    end
end
