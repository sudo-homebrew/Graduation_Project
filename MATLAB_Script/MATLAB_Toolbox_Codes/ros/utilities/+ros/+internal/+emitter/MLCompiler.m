classdef MLCompiler < handle
%This class is for internal use only. It may be removed in the future.

%MLCompiler parse template string, extract all the tokens and allocates the
%   appropriate node for each token based on its type.

%Copyright 2019 The MathWorks, Inc.

    properties (Constant)

        %TokenStart - Start symbol of a token if it is not text
        TokenStart = '{%';

        %TokenEnd -End symbol of a token if it is not text
        TokenEnd = '%}';

        %TokenIfStart - Start symbol of If token
        TokenIfStart = {[ros.internal.emitter.MLCompiler.TokenStart 'if ' ], ...
                        [ros.internal.emitter.MLCompiler.TokenStart ' if ']};

        %TokenIfEnd - End symbol of If token
        TokenIfEnd = {[ros.internal.emitter.MLCompiler.TokenStart ' endif' ros.internal.emitter.MLCompiler.TokenEnd],...
                      [ros.internal.emitter.MLCompiler.TokenStart ' endif ' ros.internal.emitter.MLCompiler.TokenEnd]};

        %TokenForStart - Start symbol of For token
        TokenForStart = {[ros.internal.emitter.MLCompiler.TokenStart 'for ' ], ...
                         [ros.internal.emitter.MLCompiler.TokenStart ' for ']};

        %TokenForEnd - End symbol of For token
        TokenForEnd = {[ros.internal.emitter.MLCompiler.TokenStart ' endfor' ros.internal.emitter.MLCompiler.TokenEnd],...
                       [ros.internal.emitter.MLCompiler.TokenStart ' endfor ' ros.internal.emitter.MLCompiler.TokenEnd]};

        %TokenAssign - var line
        TokenAssign = {[ros.internal.emitter.MLCompiler.TokenStart 'var ' ], ...
                       [ros.internal.emitter.MLCompiler.TokenStart ' var ' ]};
    end

    methods
        function root = compile(self, templatestring)
        %Get the nodes for all types of tokens

            root = ros.internal.emitter.MLRoot();
            nodeList = {root};

            % Extract all the tokens by parsing the template string.
            tokens = self.extractTokens(templatestring);

            % Iterate over tokens and create nodes based on the type.
            for i = 1:length(tokens)
                token = tokens{i};

                parentNode = nodeList{end};
                if endsWith(token, self.TokenIfEnd)
                    nodeList(end) = [];
                    continue;
                end

                if endsWith(token, self.TokenForEnd)
                    nodeList(end) = [];
                    continue;
                end

                % Get the node based on token type.
                newNode = self.createNodeFor(token);
                parentNode.addChild(newNode);

                if isequal(class(newNode), 'ros.internal.emitter.MLIfNode')
                    nodeList{end+1} = newNode;  %#ok<AGROW>
                end

                if isequal(class(newNode), 'ros.internal.emitter.MLForNode')
                    nodeList{end+1} = newNode;  %#ok<AGROW>
                end
            end
        end
    end

    methods(Access = private)
        function tokens = extractTokens(self, templatestring)
        %Parse the template string to extract tokens.

        % Get the data present in between the TOKEN_START and
        % TOKEN_END.
            pattern = sprintf('%s(.*?)%s',...
                              self.TokenStart, self.TokenEnd);
            [vars, text] = regexp(templatestring, ...
                                  pattern, 'match', 'split');

            % Rebuild the string as a cell array of tokens
            tokens = cell(0);
            for i = 1:length(vars); tokens{i*2} = vars{i}; end
            for i = 1:length(text); tokens{(i-1)*2 + 1} = text{i}; end
        end

        function node = createNodeFor(self, token)
        %Query the token type and create the appropriate MLNode object.

            if startsWith(token, self.TokenStart)
                % Evaluate the token type and assign the node accordingly.

                if startsWith(token, self.TokenIfStart)
                    node = ros.internal.emitter.MLIfNode(token);
                elseif startsWith(token, self.TokenForStart)
                    node = ros.internal.emitter.MLForNode(token);
                elseif startsWith(token, self.TokenAssign)
                    node = ros.internal.emitter.MLAssignNode(token);
                else
                    node = ros.internal.emitter.MLVarNode(token);
                end
            else
                % If the token doesn't start with TOKEN_START, then it is
                % a text. So start the text node.

                node = ros.internal.emitter.MLTextNode(token);
            end
        end
    end
end
