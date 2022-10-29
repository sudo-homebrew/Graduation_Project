classdef SimpleMap < coder.ExternalDependency & nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%SIMPLEMAP Codegen redirect class for nav.algs.internal.builtin.SimpleMap 

% Copyright 2021 The MathWorks, Inc.

%#codegen
    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName
            name = 'SimpleMap';
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
        %MapInternal
        MapInternal
    end
    properties(SetAccess = immutable)
        % DataDim is the property to store node dimension for the priority
        % queue. Making it immutable enables coder to fix bounds according
        % to it.
        DataDim
    end
    
    methods

        function obj = SimpleMap(dim)
            %SimpleMap Constructor (for codegen redirect class)
            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'SimpleMap', 'Data Dimension');
            coder.cinclude('priorityqueue_api.hpp');
            obj.MapInternal = coder.opaque('void*', 'NULL');
            obj.MapInternal = coder.ceval('priorityqueuecodegen_constructSimpleMap', dim);
            obj.DataDim = dim;
        end
        
        function delete(obj)
            %delete
            coder.cinclude('priorityqueue_api.hpp');
            
            if ~isempty(obj.MapInternal)
                coder.ceval('priorityqueuecodegen_destructSimpleMap', obj.MapInternal);
            end
        end
        
        function nodeId = insertData(obj, id, data)
            %insertData
            coder.cinclude('priorityqueue_api.hpp');
            coder.ceval('priorityqueuecodegen_simplemap_insertData', obj.MapInternal, id, coder.rref(data));
        end

        function data = getData(obj, id)
            %getData
            coder.cinclude('priorityqueue_api.hpp');
            
            dim = coder.nullcopy(0);
            dim = coder.ceval('priorityqueuecodegen_simplemap_getDataDim', obj.MapInternal);
            % Asserting that dim we got returned after function call is
            % still within limits of DataDim, which is nontunable property.
            assert(dim<=obj.DataDim)

            data = coder.nullcopy(zeros(1, dim));
            coder.ceval('priorityqueuecodegen_simplemap_getData', obj.MapInternal, id, coder.ref(data));
            % Reasserting size of data, after function call, as it
            % might have changed.
            assert(size(data,2)<=obj.DataDim)
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

