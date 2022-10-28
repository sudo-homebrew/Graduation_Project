classdef MonteCarloLocalizationBuildable < coder.ExternalDependency
%This class is for internal use only. It may be removed in the future.

%MonteCarloLocalizationBuildable Buildable class for MCL code generation
%
%   See also nav.algs.internal.MonteCarloLocalizationBuiltins.

% Copyright 2019-2020 The MathWorks, Inc.

%#codegen

%% Methods inherited from coder.ExternalDependency
    methods (Static)
        function name = getDescriptiveName(~)
        %getDescriptiveName Get name for external dependency

            name = 'MonteCarloLocalizationBuildable';
        end

        function updateBuildInfo(buildInfo, buildConfig) %#ok<INUSD>
            %updateBuildInfo Add headers, libraries, and sources to the build info
            
            nav.algs.internal.coder.MonteCarloLocalizationBuildable.addCommonHeaders(buildInfo);
            
            % Always use full sources for code generation
            buildInfo.addSourcePaths({fullfile(matlabroot,'toolbox', ...
                'nav','navalgs','builtins','libsrc','mclcodegen','amcl')});
            buildInfo.addSourceFiles('mclcodegen_api.cpp');
            
            % Add some necessary defines
            % Define LIBRARY_EXPORTS since we are compiling AMCL from source.
            % Define _USE_MATH_DEFINES for math definitions.
            % Add them to OPTS group, so rapid accelerator recognizes these
            % defines. See g1762922 and g1974701 for more information.
            buildInfo.addDefines('LIBRARY_EXPORTS', 'OPTS');
            buildInfo.addDefines('_USE_MATH_DEFINES', 'OPTS');

            % Collect all sources and header files for AMCL 3P library
            externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
                'robotics', 'externalDependency');
                        
            % Find all AMCL *.c and *.cpp files recursively
            amclSrcPath = fullfile(externalSourcePath, 'amcl-localization', 'src', 'amcl');
            amclCFiles = dir(fullfile(amclSrcPath, '**', '*.c'));
            amclCPPFiles = dir(fullfile(amclSrcPath, '**', '*.cpp'));
            amclSourceFiles = [amclCFiles; amclCPPFiles];
            
            % Add AMCL sources to buildInfo
            buildInfo.addSourcePaths(unique({amclSourceFiles.folder}));
            arrayfun(@(s)buildInfo.addSourceFiles(s.name), amclSourceFiles, 'UniformOutput', false);
            
            % Find all AMCL *.h and *.hpp files recursively
            amclIncludePath = fullfile(externalSourcePath, 'amcl-localization', 'include', 'amcl');
            amclHFiles = dir(fullfile(amclIncludePath, '**', '*.h'));
            amclHPPFiles = dir(fullfile(amclIncludePath, '**', '*.hpp'));
            amclIncludeFiles = [amclHFiles; amclHPPFiles];
            
            % Add AMCL includes to buildInfo
            buildInfo.addIncludePaths(unique({amclIncludeFiles.folder}));
            arrayfun(@(s)buildInfo.addIncludeFiles(s.name), amclIncludeFiles, 'UniformOutput', false);
            
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
    methods (Static)
        function mclData = initializeWithSeed(randomSeed)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            mclData = coder.opaque('void*', 'NULL');
            
            % Call the C API
            mclData = coder.ceval('mclInitializeWithSeed_real64',...
                        randomSeed);
        end

        function setUpdateThresholds(mclData, x,y,theta)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetUpdateThresholds_real64',...
                        mclData, x, y, theta);
        end

        function setResamplingInterval(mclData, interval)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetResamplingInterval_real64',...
                        mclData, interval);
        end

        function setSensorModel(mclData, numBeams, expectedMeasurementWeight, ...
                                          randomMeasurementWeight, measurementNoise, ...
                                          maxLikelihoodDistance, sensorLimits, ...
                                          sensorPose)

            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetSensorModel_real64',...
                        mclData, ...
                        numBeams, expectedMeasurementWeight, ...
                        randomMeasurementWeight, measurementNoise, ...
                        maxLikelihoodDistance, ...
                        coder.rref(sensorLimits), coder.rref(sensorPose));
        end

        function setOccupancyGrid(mclData, width, height, resolution, originX, originY, gridCells)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetOccupancyGrid_real64',...
                        mclData, ...
                        width, height, resolution, ...
                        originX, originY, coder.rref(gridCells));

        end

        function setMotionModel(mclData, motionParams)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetMotionModel_real64', mclData, coder.rref(motionParams));
        end

        function initializePf(mclData, minParticles, maxParticles, ...
                              alphaSlow, alphaFast, kldErr, kldZ)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclInitializePf_real64', mclData, ...
                        minParticles, maxParticles, ...
                        alphaSlow, alphaFast, kldErr, kldZ);
        end

        function setInitialPose(mclData, pose, poseCov)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetInitialPose_real64', ...
                        mclData, ...
                        coder.rref(pose), ...
                        coder.rref(poseCov));
        end

        function update(mclData, rangeCount, ranges, angles, pose)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclUpdate_real64', ...
                        mclData, ...
                        rangeCount, ...
                        coder.rref(ranges), ...
                        coder.rref(angles), ...
                        coder.rref(pose));
        end

        function updated = isUpdated(mclData)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            updated = false;
            updated = coder.ceval('mclIsUpdated_real64', mclData);
        end

        function hyp = getHypothesis(mclData)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            hyp = coder.nullcopy(zeros(1,12));
            coder.ceval('mclGetHypothesis_real64', ...
                        mclData, coder.wref(hyp));
        end

        function globalLocalization(mclData)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclGlobalLocalization_real64', mclData);
        end

        function cleanup(mclData)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclCleanup_real64', mclData);
        end
        
        function setTestHookForMemory(mclData, setHook)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetTestHookForMemory_real64', mclData, setHook);
        end
        
        function particles = getParticles(mclData)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            % First figure out how many particles to allocate
            numParticles = uint32(0);
            numParticles = coder.ceval('mclGetNumParticles_real64', mclData);
            particles = coder.nullcopy(zeros(4,double(numParticles)));
            
            % Then actually retrieve the particles
            coder.ceval('mclGetParticles_real64', mclData, coder.ref(particles));            
        end
        
        function setParticles(mclData, particles, weights)
            coder.inline('always');
            coder.cinclude('mclcodegen_api.hpp')

            % Call the C API
            coder.ceval('mclSetParticles_real64', mclData, coder.rref(particles), coder.rref(weights));
        end
    end
end
