classdef NodeMap < coder.ExternalDependency & nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%NODEMAP Codegen redirect class for nav.algs.internal.builtin.NodeMap 

% Copyright 2021 The MathWorks, Inc.

%#codegen
    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = 'NodeMap';
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
    
    properties
        %NodeMapInternal
        NodeMapInternal
    end
    properties (SetAccess = immutable)
        % DataDim is the property to store node dimension for the priority
        % queue. Making it immutable enables coder to fix bounds according
        % to it.
        DataDim

        % Max number of elements allowed in array variable as defined by
        % coder, when dynamic memory allocation is off.
        maxAllowedElements
    end
    
    methods

        function obj = NodeMap(dim)
            %NodeMap Constructor (for codegen redirect class)
            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'NodeMap', 'Data Dimension');
            coder.cinclude('priorityqueue_api.hpp');
            obj.NodeMapInternal = coder.opaque('void*', 'NULL');
            obj.NodeMapInternal = coder.ceval('priorityqueuecodegen_constructNodeMap', dim);
            obj.DataDim = dim;
            VARSIZE_ENABLED = coder.const(strcmp(eml_option('VariableSizing'),'DisableInInference'));
            if(VARSIZE_ENABLED)
                obj.maxAllowedElements = inf;
            else
                obj.maxAllowedElements = 134217728;
            end
        end
        
        function delete(obj)
            %delete
            coder.cinclude('priorityqueue_api.hpp');
            
            if ~isempty(obj.NodeMapInternal)
                coder.ceval('priorityqueuecodegen_destructNodeMap', obj.NodeMapInternal);
            end
        end
        
        function nodeDataVec = traceBack(obj, idx)
            %traceBack
            coder.cinclude('priorityqueue_api.hpp');
            
            dim = coder.nullcopy(0);
            dim = coder.ceval('priorityqueuecodegen_nodemap_getDataDim', obj.NodeMapInternal);
            % Asserting that dim we got returned after function call is
            % still within limits of DataDim, which is nontunable property.
            assert(dim<=obj.DataDim)

            numNodes = coder.nullcopy(0);
            maxNumNodes = coder.nullcopy(0);
            maxNumNodes = coder.ceval('priorityqueuecodegen_nodemap_getNumNodes', obj.NodeMapInternal);
            
            % Asserting that maxNumNodes we got returned after function call is
            % still within limits.            
            assert(maxNumNodes<(obj.maxAllowedElements/obj.DataDim))
            
            data = coder.nullcopy(zeros(dim, maxNumNodes)); % allocate a matrix that is large enough
            coder.ceval('priorityqueuecodegen_nodemap_traceBack', obj.NodeMapInternal, idx, coder.ref(data), coder.ref(numNodes));
            
            % Asserting that numNodes we got returned after function call is
            % within limits of maxNumNodes
            assert(numNodes<=maxNumNodes)
            nodeDataVec = data(1:dim, 1:numNodes)';
            % Reasserting if nodeDataVec is within limit bounds of
            % nontunable parameters
            assert(size(nodeDataVec,2)<=obj.DataDim)
            assert(size(nodeDataVec,1)<=maxNumNodes)
        end
        
        function nodeId = insertNode(obj, nodeData, parentId)
            %insertNode            
            coder.cinclude('priorityqueue_api.hpp');
            validateattributes(nodeData,{'double'},{'ncols',obj.DataDim});

            numNodes = coder.nullcopy(0);
            numNodes = coder.ceval('priorityqueuecodegen_nodemap_getNumNodes', obj.NodeMapInternal);
            assert(numNodes<(obj.maxAllowedElements/obj.DataDim))
              

            nodeId = coder.nullcopy(0);
            nodeId = coder.ceval('priorityqueuecodegen_nodemap_insertNode', obj.NodeMapInternal, coder.rref(nodeData), parentId);
        end

        function [nodeData, parentId] = getNodeData(obj, nodeId)
            %getNodeData
            coder.cinclude('priorityqueue_api.hpp');
            
            dim = coder.nullcopy(0);
            dim = coder.ceval('priorityqueuecodegen_nodemap_getDataDim', obj.NodeMapInternal);
            % Asserting that dim we got returned after function call is
            % still within limits of DataDim, which is nontunable property.
            assert(dim<=obj.DataDim)

            parentId = coder.nullcopy(0);
            nodeData = coder.nullcopy(zeros(1, dim));
            coder.ceval('priorityqueuecodegen_nodemap_getNodeData', obj.NodeMapInternal, nodeId, coder.ref(nodeData), coder.ref(parentId));
            % Asserting that nodeData we got returned after function call is
            % still within limits of DataDim, which is nontunable property.
            assert(size(nodeData,2)<=obj.DataDim)
        end
    end
    methods (Static, Hidden)
        function result = matlabCodegenSoftNontunableProperties(~)
        %matlabCodegenSoftNontunableProperties Mark properties as nontunable during codegen
        % 
        % Marking properties as 'Nontunable' indicates to Coder that
        % the property should be made compile-time Constant.
            result = {'DataDim','maxAllowedElements'};
        end
    end
end

