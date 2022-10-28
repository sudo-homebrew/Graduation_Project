classdef occupancyMap3DBuildable < coder.ExternalDependency
    %This class is for internal use only. It may be removed in the future.
    
    %occupancyMap3DBuildable Buildable class for octomap code generation
    %
    %   See also nav.algs.internal.occupancyMap3DBuiltins.
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    %#codegen
    
    %% Methods inherited from coder.ExternalDependency
    methods (Static)
        function name = getDescriptiveName(~)
            %getDescriptiveName Get name for external dependency
            
            name = 'occupancyMap3DBuildable';
        end
        
        function updateBuildInfo(buildInfo, buildConfig) %#ok<INUSD>
            %updateBuildInfo Add headers, libraries, and sources to the build info
            
            nav.algs.internal.coder.occupancyMap3DBuildable.addCommonHeaders(buildInfo);
            
            % Always use full sources for code generation
            buildInfo.addSourcePaths({fullfile(matlabroot,'toolbox', ...
                'nav','navalgs','builtins','libsrc','octomapcodegen','octomap')});
            buildInfo.addSourceFiles('octomapcodegen_api.cpp');
            buildInfo.addSourceFiles('octomapcodegen.cpp');
            
            % Add some necessary defines
            % Define octomap_EXPORTS since we are compiling Octomap from source.
            % Define _USE_MATH_DEFINES for math definitions.
            % Add them to OPTS group, so rapid accelerator recognizes these
            % defines. See g1762922 and g1974701 for more information.            
            buildInfo.addDefines('octomap_EXPORTS', 'OPTS');
            buildInfo.addDefines('octomath_EXPORTS', 'OPTS');
            buildInfo.addDefines('_USE_MATH_DEFINES', 'OPTS');
            
            % Collect all sources and header files for OMPL 3P library
            externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
                'robotics', 'externalDependency');
            
            % Find all Octomap *.cpp files recursively
            octomapSrcPath = fullfile(externalSourcePath, 'octomap', 'src');
            octomapCPPFiles = dir(fullfile(octomapSrcPath, '**', '*.cpp'));            

            % Add Octomap sources to buildInfo
            buildInfo.addSourcePaths({octomapCPPFiles.folder});
            arrayfun(@(s)buildInfo.addSourceFiles(s.name), octomapCPPFiles, 'UniformOutput', false);
            
            % Find all Octomap *.h and *.hpp files recursively
            octomapIncludePath = fullfile(externalSourcePath, 'octomap', 'include');
            octomapHFiles = dir(fullfile(octomapIncludePath, '**', '*.h'));
            octomapHXXFiles = dir(fullfile(octomapIncludePath, '**', '*.hxx'));
            octomapIncludeFiles = [octomapHFiles; octomapHXXFiles];

            % Add Octomap includes to buildInfo
            buildInfo.addIncludePaths({octomapIncludePath, octomapIncludeFiles.folder});
            arrayfun(@(s)buildInfo.addIncludeFiles(s.name), octomapIncludeFiles, 'UniformOutput', false);
        end
        
        function isSupported = isSupportedContext(~)
            %isSupportedContext Determine if external dependency supports this build context
            
            % Code generation is supported for both host and target
            % (portable) code generation.
            isSupported = true;
            
        end
    end
    
    methods (Static, Access = protected)
        function addCommonHeaders(buildInfo)
            %addCommonHeaders Add include path for codegen APIs.
            
            includePath = fullfile(matlabroot, 'extern', 'include', 'nav');
            buildInfo.addIncludePaths(includePath, 'Navigation Toolbox Includes');
        end
    end
    
    %% Actual API methods
    
    properties
        %Octomap
        Octomap
    end
    methods
        function obj = occupancyMap3DBuildable(resolution)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            obj.Octomap = coder.opaque('void*', 'NULL');
            
            % Call the C API
            obj.Octomap = coder.ceval('octomapInitialize_real64', resolution);
        end
        
        function setClampingThreshold(obj, clampingThresMin, clampingThresMax)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapSetClampingThreshold_real64', obj.Octomap,...
                clampingThresMin, clampingThresMax);
        end
        
        function res = getResolution(obj)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            res = 0;
            res = coder.ceval('octomapGetResolution_real64', obj.Octomap);
        end
        
        function occ = getOccupancy(obj, pos)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            occ = coder.nullcopy(zeros(size(pos,1),1));
            coder.ceval('octomapGetOccupancy_real64', obj.Octomap, pos, uint32(size(pos,1)), coder.ref(occ));
        end
        
        function data = serialization(obj)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            n = coder.nullcopy(uint32(numel(0)));
            n = coder.ceval('octomapGetSizeSerializationData_real64', obj.Octomap);            
            
            % Call the C API
            data = coder.nullcopy(blanks(n));
            coder.ceval('octomapSerialization_real64', obj.Octomap, coder.ref(data));
        end
        
        function deserialization(obj, data)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            val = true;
            val = coder.ceval('octomapDeserialization_real64', obj.Octomap, data, uint32(numel(data)));
            
            if ~val
                coder.internal.error('nav:navalgs:octomapwrapper:InvalidMapFile', "MAT file");
            end
        end
        
        function setNodeValue(obj, xyz, prob, lazyEval)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapSetNodeValue_real64', obj.Octomap, xyz, prob, lazyEval);
        end
        
        function updateNodeBoolean(obj, xyz, occupied, lazyEval)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapUpdateNodeBoolean_real64', obj.Octomap, xyz, occupied, lazyEval);
        end
        
        function updateNodeDouble(obj, xyz, probUpdate, lazyEval)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapUpdateNodeDouble_real64', obj.Octomap, xyz, probUpdate, lazyEval);
        end
        
        function inflate(obj, inflationRadius, occupiedThreshold)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapInflate_real64', obj.Octomap, inflationRadius, occupiedThreshold);
        end
        
        function insertPointCloud(obj, origin, points, maxRange, lazyEval, discretize)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            nPoints = size(points,1);
            isInfInPointCloud = false;
            isInfInPointCloud = coder.ceval('octomapInsertPointCloud_real64', ...
                obj.Octomap, origin, points, uint32(nPoints), maxRange, lazyEval, discretize);
            
           coder.internal.errorIf(isInfInPointCloud, 'nav:navalgs:octomapwrapper:PointsContainNanInf');
        end
        
        function out =  getRayIntersection(obj, ptStart, ptDirections, occupiedThreshold, ignoreUnknownCells, maxRange)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            
            % Preallocate outputs. Do not initialize memory in codegen,
            % since the C function will assign all elements.
            nPtDirections = size(ptDirections,1);
            out = coder.nullcopy(zeros(nPtDirections,4));                        
            coder.ceval('octomapGetRayIntersection_real64', obj.Octomap, ptStart, ptDirections, uint32(nPtDirections), occupiedThreshold,...
                ignoreUnknownCells, maxRange, coder.wref(out));
        end
        
        function read(obj, filename)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            filePathStatus = true;
            filePathStatus = coder.ceval('octomapRead_real64', obj.Octomap, filename);
            
            coder.internal.errorIf(~filePathStatus, 'nav:navalgs:octomapwrapper:InvalidMapFile', filename);
        end
        
        function readBinary(obj, filename)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            filePathStatus = true;
            filePathStatus = coder.ceval('octomapReadBinary_real64', obj.Octomap, filename);
            
            coder.internal.errorIf(~filePathStatus, 'nav:navalgs:octomapwrapper:InvalidMapFile', filename);
        end
        
        function write(obj, filename)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapWrite_real64', obj.Octomap, filename);            
        end
        
        function writeBinary(obj, filename)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            coder.ceval('octomapWriteBinary_real64', obj.Octomap, filename);            
        end
        
        function sz = memoryUsage(obj)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            sz = coder.opaque('size_t',0);
            sz = coder.ceval('octomapMmemoryUsage_real64', obj.Octomap);
        end
        
        function dims = getMapDimensions(obj)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            dims = coder.nullcopy(zeros(3, 2));
            coder.ceval('octomapGetMapDimensions_real64', obj.Octomap, coder.wref(dims));
        end
        
        function deserializationBinaryROSMsgData(obj, res, data)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            sz = uint32(numel(data));
            coder.ceval('octomapDeserializationBinaryROSMsgData_real64', obj.Octomap, res, data, sz);
        end
        
       function deserializationFullROSMsgData(obj, res, data)
            coder.inline('always');
            coder.cinclude('octomapcodegen_api.hpp')
            
            % Call the C API
            sz = uint32(numel(data));
            coder.ceval('octomapDeserializationFullROSMsgData_real64', obj.Octomap, res, data, sz);
        end
    end
end
