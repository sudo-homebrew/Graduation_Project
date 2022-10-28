classdef MLTextNode < ros.internal.emitter.MLNode
%This class is for internal use only. It may be removed in the future.

%MLTextNode returns the text data of a file, which is not dependent on any data.

%Copyright 2019 The MathWorks, Inc.

    properties
        text
    end

    methods
        function self = MLTextNode(token)
        %If the token is sample text and doesn't need any data to
        %   replace, then return the token directly.

            self.text = token;
        end

        function out = render(self, ~)
        %Return the text into output file.

            out = self.text;
        end
    end
end
