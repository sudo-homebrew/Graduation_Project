classdef PriorityQueue < coder.ExternalDependency & nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%PriorityQueue Codegen redirect class for nav.algs.internal.builtin.PriorityQueue 

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen
    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = 'PriorityQueue';
        end

        function isSupported = isSupportedContext(~)
            %isSupportedContext Determine if build context supports external dependency
            isSupported = true;
        end

        function updateBuildInfo(buildInfo, buildConfig)
            %updateBuildInfo

            buildInfo.addIncludePaths(fullfile(matlabroot, 'extern', 'include', 'nav'));

            % Always build with full sources (for both host and target codegen)
            buildInfo.addSourcePaths({fullfile(matlabroot,'toolbox', ...
                                               'nav','navalgs2','builtins','libsrc','priorityqueuecodegen')});
            buildInfo.addSourceFiles('priorityqueue_api.cpp');
        end
    end
    
    properties (Access = private)
        %PQInternal
        PQInternal        
    end
    properties(SetAccess = immutable)
        % DataDim is the property to store node dimension for the priority
        % queue. Making it immutable enables coder to fix bounds according
        % to it.
        DataDim
    end

    methods
        
        function obj = PriorityQueue(dim, primeIdx)
            %PriorityQueue Constructor (for codegen redirect class)
                        
            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'PriorityQueue', 'Data Dimension');
            validateattributes(primeIdx, {'double'},{'scalar', 'positive', 'finite', 'integer', '<=', dim}, 'PriorityQueue','Prime Index');

            coder.cinclude('priorityqueue_api.hpp');
            obj.PQInternal = coder.opaque('void*','NULL');
            obj.PQInternal = coder.ceval('priorityqueuecodegen_constructPQ', dim, primeIdx-1); % primeIdx - 1 as MATLAB uses 1-index
            obj.DataDim = dim;
        end
        
        function delete(obj)
            %delete
            coder.cinclude('priorityqueue_api.hpp');
            
            if ~isempty(obj.PQInternal)
                coder.ceval('priorityqueuecodegen_destructPQ', obj.PQInternal);
            end
        end

        function flag = isEmpty(obj)
            %isEmpty
            coder.cinclude('priorityqueue_api.hpp');
            flag = coder.nullcopy(0);
            flag = coder.ceval('priorityqueuecodegen_isEmpty', obj.PQInternal);
            flag = logical(flag); % as MATLAB uses 1-index
        end

        function idx = push(obj, nodeData)
            %push
            coder.cinclude('priorityqueue_api.hpp');
            idx = coder.nullcopy(0);            
            idx = coder.ceval('priorityqueuecodegen_push', obj.PQInternal, coder.rref(nodeData));
            idx = idx + 1; 
        end

        function [nodeData, nodeId] = top(obj)
            %top           
            coder.cinclude('priorityqueue_api.hpp');
            
            dataDim = coder.nullcopy(0);
            dataDim = coder.ceval('priorityqueuecodegen_getDataDim', obj.PQInternal);
            % Asserting that dim we got returned after function call is
            % still within limits of DataDim, which is nontunable property.
            assert(dataDim<=obj.DataDim);

            nodeData = coder.nullcopy(zeros(1, dataDim));
            nodeId = coder.nullcopy(0);   

            coder.ceval('priorityqueuecodegen_top', obj.PQInternal, coder.ref(nodeData), coder.ref(nodeId));
            % Reasserting size of nodeData, after function call, to let coder know
	    % the size of nodeData.
            assert(size(nodeData,2)<=obj.DataDim)

            nodeId = nodeId + 1; % as MATLAB uses 1-index
        end

        function pop(obj)
            %pop
            coder.cinclude('priorityqueue_api.hpp');
            coder.ceval('priorityqueuecodegen_pop',obj.PQInternal);
        end

        function sz = size(obj)
            %size
            coder.cinclude('priorityqueue_api.hpp');
            sz = coder.nullcopy(0);
            sz = coder.ceval('priorityqueuecodegen_size', obj.PQInternal);
        end
    end
   
    methods (Static, Hidden)
        function result = matlabCodegenSoftNontunableProperties(~)
        %matlabCodegenSoftNontunableProperties Mark properties as nontunable during codegen
        % 
        % Marking properties as 'Nontunable' indicates to Coder that
        % the property should be made compile-time Constant.
            result = {'DataDim'};
        end
    end

end
