classdef (Abstract) MLNode < handle
%This class is for internal use only. It may be removed in the future.

%MLNode creates a node for each token.

%Copyright 2019-2020 The MathWorks, Inc.


    properties
        children
        expressioncache
        reqdfields
        datafields
    end

    methods
        function addChild(self, child)
        % Adds all the children of type MLNode.

            if ~isa(child, 'ros.internal.emitter.MLNode')
                error(message('ros:utilities:templates:NotMLNode',class(child)));
            end
            self.children{end+1} = child;
        end

        function out = createConcernedVarList(~, expr)
            %temp = strsplit(expr,'%%');
            %tempSplit = strsplit(temp{2},',');
            words = regexp(expr,'(\w+)','tokens');
            out = [words{:}];
        end
        
        function assignin(self,data__)
            for fieldIterator__ = 1:length(self.datafields)
                %if isequal(fields__{fieldIterator__},ros.internal.emitter.MLTempSpace.TEMPSPACENAME)
                %    continue;
                %end
                % create the variables from fields in current workspace
                % and access it.
                assignin('caller', self.datafields{fieldIterator__}, data__.(self.datafields{fieldIterator__}));
            end
            if isfield(data__,ros.internal.emitter.MLTempSpace.TEMPSPACENAME)
                fields__ = fieldnames(data__.(ros.internal.emitter.MLTempSpace.TEMPSPACENAME).TempSpace);
                fields__ = intersect(fields__, self.reqdfields);
                for fieldIterator__ = 1:length(fields__)
                    % create the variables from fields in current workspace
                    % and access it.
                    assignin('caller', fields__{fieldIterator__}, ...
                               data__.(ros.internal.emitter.MLTempSpace.TEMPSPACENAME).TempSpace.(fields__{fieldIterator__}));
                end
            end
        end
        
        function out = eval(self)
            % If expr is a variable, eval returns its value and if it is an
            % 'If' expression then it evaluates the expression and returns
            % the value.
            out = evalin('caller',self.expressioncache);
        end
        
        function out = evaluate(self, expr__, data__)
        % Unpacks data structure into workspace and evaluates expression.
        % using a crazy names so no local var will clash with fields in data
            if ~isstruct(data__)
                error(message('ros:utilities:templates:NotStruct'));
            end
            if ~isequal(self.expressioncache, expr__)
                self.expressioncache = expr__;
                self.reqdfields = self.createConcernedVarList(expr__);
                self.datafields = intersect(fields(data__), self.reqdfields);
            end
            try
                self.assignin(data__);
                out = self.eval();
            catch ex
                newEx = MException('ros:utilities:templates:ErrorEvaluatingExpr',self.expressioncache);
                newEx = newEx.addCause(ex);
                throw(newEx);
            end
        end
    end

    methods(Abstract)
        out = render(self, data);
    end
end
