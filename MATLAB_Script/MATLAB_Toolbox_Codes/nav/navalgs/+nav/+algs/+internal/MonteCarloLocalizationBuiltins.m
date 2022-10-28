classdef MonteCarloLocalizationBuiltins < handle
%This class is for internal use only. It may be removed in the future.

%MonteCarloLocalizationBuiltins Interface to builtins used for Monte Carlo Localization
%
%   This class is a collection of functions used for interfacing with the
%   AMCL 3P library. Its main purpose is to dispatch
%   function calls correctly when executed in MATLAB or code
%   generation.
%   During MATLAB execution, we call the existing MCOS C++ class.
%   During code generation we use a codegen-compatible version.
%
%   See also nav.algs.internal.coder.MonteCarloLocalizationBuildable

% Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    properties
        %MCOSObj - MCOS interface object to AMCL
        %   This is only used during MATLAB execution.
        MCOSObj = []
        
        %MCLData - Opaque C++ object
        %   This is only used during code generation.
        MCLData
    end    

    methods
        function obj = MonteCarloLocalizationBuiltins(randomSeed)
        %MonteCarloLocalizationBuiltins Constructor

            if coder.target('MATLAB')
                % Create MCOS class in MATLAB
                obj.MCOSObj = nav.algs.internal.MonteCarloLocalization(randomSeed);
            else
                % Generate code through external dependency
                obj.MCLData = nav.algs.internal.coder.MonteCarloLocalizationBuildable.initializeWithSeed(randomSeed);
            end
        end
        
        function delete(obj)
            if ~isempty(obj.MCOSObj)
                delete(obj.MCOSObj);
            end
        end
    end

    methods
        function setUpdateThresholds(obj, x, y, theta)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setUpdateThresholds(x,y,theta);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setUpdateThresholds(obj.MCLData,x,y,theta);
            end
        end

        function setResamplingInterval(obj, interval)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setResamplingInterval(interval);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setResamplingInterval(obj.MCLData,interval);
            end
        end

        function setSensorModel(obj, ...
                                numBeams, ...
                                expectedMeasurementWeight, ...
                                randomMeasurementWeight, ...
                                measurementNoise, ...
                                maxLikelihoodDistance, ...
                                sensorLimits, ...
                                sensorPose)

            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setSensorModel(numBeams, ...
                                           expectedMeasurementWeight, ...
                                           randomMeasurementWeight, ...
                                           measurementNoise, ...
                                           maxLikelihoodDistance, ...
                                           sensorLimits, ...
                                           sensorPose);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setSensorModel(obj.MCLData, numBeams, ...
                                                                  expectedMeasurementWeight, ...
                                                                  randomMeasurementWeight, ...
                                                                  measurementNoise, ...
                                                                  maxLikelihoodDistance, ...
                                                                  sensorLimits, ...
                                                                  sensorPose);
            end
        end

        function setOccupancyGrid(obj, width, height, resolution, ...
                                       originX, originY, gridCells)

            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setOccupancyGrid(width, height, resolution, ...
                                                    originX, originY, gridCells);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setOccupancyGrid(...
                    obj.MCLData, width, height, resolution, originX, originY, gridCells);
            end
        end

        function setMotionModel(obj, motionParams)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setMotionModel(motionParams);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setMotionModel(obj.MCLData, motionParams);
            end
        end

        function initializePf(obj, minParticles, maxParticles, ...
                              alphaSlow, alphaFast, kldErr, kldZ)

            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.initializePf(minParticles, maxParticles, ...
                                         alphaSlow, alphaFast, kldErr, kldZ);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.initializePf(...
                    obj.MCLData, minParticles, maxParticles, alphaSlow, alphaFast, kldErr, kldZ);
            end
        end

        function setInitialPose(obj, pose, poseCov)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setInitialPose(pose, poseCov);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setInitialPose(obj.MCLData, pose, poseCov);
            end
        end

        function update(obj, rangeCount, ranges, angles, pose)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.update(rangeCount, ranges, angles, pose);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.update(obj.MCLData, rangeCount, ranges, angles, pose);
            end
        end

        function updated = isUpdated(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                updated = obj.MCOSObj.isUpdated();
            else
                % Generate code through external dependency
                updated = nav.algs.internal.coder.MonteCarloLocalizationBuildable.isUpdated(obj.MCLData);
            end
        end

        function hyp = getHypothesis(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                hyp = obj.MCOSObj.getHypothesis();
            else
                % Generate code through external dependency
                hyp = nav.algs.internal.coder.MonteCarloLocalizationBuildable.getHypothesis(obj.MCLData);
            end
        end

        function globalLocalization(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.globalLocalization();
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.globalLocalization(obj.MCLData);
            end
        end

        function cleanup(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                delete(obj.MCOSObj);
                obj.MCOSObj = [];
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.cleanup(obj.MCLData);
            end
        end
        
        function setTestHookForMemory(obj, setHook)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setTestHookForMemory(setHook);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setTestHookForMemory(obj.MCLData, setHook);
            end
        end
        
        function particles = getParticles(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                particles = obj.MCOSObj.getParticles();
            else
                % Generate code through external dependency
                particles = nav.algs.internal.coder.MonteCarloLocalizationBuildable.getParticles(obj.MCLData);
            end
        end
        
        function setParticles(obj, particles, weights)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setParticles(particles, weights);
            else
                % Generate code through external dependency
                nav.algs.internal.coder.MonteCarloLocalizationBuildable.setParticles(obj.MCLData, particles, weights);
            end
        end
    end
end
