classdef ReservedNamesChecker
%This class is for internal use only. It may be removed in the future.

%  ReservedNamesChecker is a utility class for checking if a ROS
%  property name are on a "reserved name" list. Property names that
%  match a reserved name can be "mangled", which appends an underscore
%  to the name.
%
%  Usage:
%    checker = ros.slros.internal.bus.ReservedNamesChecker.getInstance;
%    checker.isReserved('Vector')
%    mangledName = checker.mangleNameIfNeeded('Vector') % returns 'Vector_'

%   Copyright 2016-2018 The MathWorks, Inc.

% Details:
%  When Simulink generates code for a bus structure, it errors out if
%  any of the bus fields match language keywords (for C, C++, TLC).
%  Bus property names such as "Vector" and "Matrix" are TLC language
%  keywords, and hence cause a code-generation error (see g1336418).
%  This matching is case-sensitive, e.g., bus elements with name
%  "vector" and "matrix" do NOT cause an error during code generation.
%
%  The above error is an issue when mapping ROS messages to Simulink
%  buses, since ROS message properties can have arbitrary names.
%
%  The strategy for dealing with this issue is the following:
%   - When creating a bus for a ROS message, check if each property
%     name is on a "reserved name" list. If it is, use a mangled
%     variant of the property name in the bus (e.g., the bus will have
%     an element "Vector_" instead of "Vector").
%   - When converting a ROS message to/from Simulink buses, the
%     data for "reserved" property names is invisibly mapped to the
%     corresponding mangled name.
%
%  The reserved name or "DefaultExclusionList" is based on the
%  list of reserved keywords specified in the following files (referred
%  to below as "Simulink Coder reserved words"):
%    matlab/src/simulink/sl_engin/rtwgen_ident_hash.cpp (search for "reservedIdents")
%    src/cgir_xform/dom_c/CReservedWords.cpp
%
%  Two simplifying factors:
%  1) The bus creation logic (for Simulink ROS) uses the property names
%     of MATLAB ROS messages. By design, these are all SentenceCased,
%     which means that we don't have to worry about Simulink Coder
%     reserved words that start with a lowercase letter.
%  2) We can ignore Simulink Coder reserved words that are ALLUPPERCASE
%     as these are pretty obscure (and would only be an issue with
%     constant properties, which aren't supported in SLROS as of 16b).
%
% Following (1) and (2), the DefaultExclusionList below only includes
% Simulink Coder reserved words that are SentencedCased.

    properties(Constant)
        DefaultExclusionList = {
            'Vector', 'Matrix' 'BlockIO' 'ExternalInputs' 'ContinuousStates' ...
            'StateDerivatives' 'StateDisabled' 'CstateAbsTol' 'ExternalOutputs' ...
            'Parameters' 'ConstParam' 'ConstParamWithInit' }
    end

    properties(SetAccess=immutable)
        ExclusionList
    end

    methods(Access=private)
        function obj = ReservedNamesChecker(listToUse)
            obj.ExclusionList = listToUse;
        end

        function out = mangleName(obj, name) %#ok<INUSL>
            out = [name '_'];
        end
    end


    methods (Access = {?ros.slros.internal.bus.ReservedNamesChecker,?matlab.unittest.TestCase})
        function out = isMangledName(obj, name)
        % TF = isMangledName(obj, NAME) returns true if and only if
        %  NAME has the  format "<ReservedWord>_"
            out = name(end)=='_' && ismember(name(1:end-1), obj.ExclusionList);
        end
    end


    methods
        function out = isReserved(obj, name)
        % TF = isReserved(obj, NAME) returns true if and only if
        %  NAME exactly (& case-sensitively) matches a reserved word

            out = ismember(name, obj.ExclusionList);
        end

        function [out, isReserved] = mangleNameIfNeeded(obj, name)
        % [MNAME, TF] = mangleNameIfNeeded(obj, NAME) returns the
        %   mangled name, MNAME, if NAME matches a reserved word. If
        %   NAME does not match a reserved word, MNAME is same as NAME.
        %   TF is a boolean indicating if NAME is a reserved word (and
        %   hence whether MNAME is a mangled name).

            isReserved = ismember(name, obj.ExclusionList);
            if isReserved
                out = obj.mangleName(name);
            else
                out = name;
            end
        end
    end


    methods(Static)
        function [isAvailable, list] = exclusionListForTesting(listToUse)
        % exclusionListForTesting(listToUse) overrides the exclusion list
        % that will be used in by ReservedNamesChecker in a subsequent
        % call to getInstance. listToUse should be a cell array of
        % strings.
        %
        % exclusionListForTesting([]) resets (i.e., removes) the override.

            persistent exclusionList
            if nargin > 0
                % allow empty double [] (indicates "clear")
                assert(isempty(listToUse) || iscellstr(listToUse));
                exclusionList = listToUse;
            end
            isAvailable = ~(isa(exclusionList,'double') && isempty(exclusionList));
            list = exclusionList;
        end

        function obj = getInstance()
            [isAvailable, list] = ros.slros.internal.bus.ReservedNamesChecker.exclusionListForTesting();
            if isAvailable
                obj = ros.slros.internal.bus.ReservedNamesChecker(list);
            else
                obj = ros.slros.internal.bus.ReservedNamesChecker(...
                    ros.slros.internal.bus.ReservedNamesChecker.DefaultExclusionList);
            end
        end

    end

end
