classdef MLAssignNode < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLForNode returns the data contained in a for loop.

%Copyright 2019-2020 The MathWorks, Inc.

    properties

        expression

        fieldname 
        
        value

        concernedvars
    end

    methods
        function self = MLAssignNode(token)
        %Returns the data which is stored in between the TOKEN_START
        %   and TOKEN_END.

            pattern = [ros.internal.emitter.MLCompiler.TokenStart '\s?var\s*(.*?)\s*' ros.internal.emitter.MLCompiler.TokenEnd];
            matchToken = regexp(token, pattern, 'tokens');
            self.expression = strtrim(matchToken{1});
                    
            a = regexp(self.expression{:},'\s*=\s*','split');
            self.fieldname = a{1};
            self.value = a{2};
        end


        function out = render(self, data)
        %Returns the data contained in For loop.

            val = self.evaluate(self.value, data);
            data.(ros.internal.emitter.MLTempSpace.TEMPSPACENAME).TempSpace.(self.fieldname) = val;
            out = '';
        end
    end
end
