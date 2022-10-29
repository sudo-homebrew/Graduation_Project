classdef AccessMCL < handle
%This class is for internal use only. It may be removed in the future.

%ACCESSMCL Allows setting and getting data from internal MCL object
%   This class allows setting and getting data from internal MCL
%   object. The data from MATLAB, such as binaryOccupancyMap,
%   and occupancyMap needs to be manipulated before passing
%   to the MCL object. This internal class implements these
%   manipulation logic.

%   Copyright 2015-2019 The MathWorks, Inc.

%#codegen

    methods (Static)
        function setOccupancyGrid(mclObj, ogObj)
        %setOccupancyGrid Set occupancy grid in internal MCL object
        %   setOccupancyGrid(MCLOBJ, OGOBJ) sets the occupancy grid
        %   from the binaryOccupancyMap or
        %   occupancyMap object OGOBJ to the
        %   internal data structure of
        %   nav.algs.internal.MonteCarloLocalization object
        %   MCLOBJ.

        % MCL grid is row major format starting from bottom,
        % so the grid needs to be rotated
            if isa(ogObj, 'binaryOccupancyMap')
                grid = double(rot90(ogObj.occupancyMatrix, -1));
            else
                grid = double(rot90(ogObj.occupancyMatrix('ternary'), -1));
            end

            % Internal MCL object defines origin of the map at the center
            % of the grid. We need to specify coordinate of the center of
            % the grid. Special handling is needed to match the internal
            % occupancy grid with the binaryOccupancyMap or
            % occupancyMap.

            halfCell = 1.0/(2*ogObj.Resolution);
            origin = zeros(1,2);
            if ~mod(ogObj.GridSize(2),2)
                origin(1,1) = sum(ogObj.XWorldLimits)/2 + halfCell;
            else
                origin(1,1) = sum(ogObj.XWorldLimits)/2;
            end

            if ~mod(ogObj.GridSize(1),2)
                origin(1,2) = sum(ogObj.YWorldLimits)/2 + halfCell;
            else
                origin(1,2) = sum(ogObj.YWorldLimits)/2;
            end

            % Pass data to internal object
            mclObj.setOccupancyGrid(ogObj.GridSize(2),ogObj.GridSize(1), ...
                                                  double(1.0/ogObj.Resolution), origin(1), origin(2), grid);
        end

        function setSensorModel(mclObj, smObj)
        %setSensorModel Set the sensor model
        %   setSensorModel(MCLOBJ, SMOBJ) sets the sensor model data
        %   from the likelihoodFieldSensorModel object SMOBJ
        %   to the internal data structure of
        %   nav.algs.internal.MonteCarloLocalization object
        %   MCLOBJ.

        % Set occupancy grid first
            nav.algs.internal.AccessMCL.setOccupancyGrid(mclObj, smObj.Map);

            % Set sensor model and its parameters
            mclObj.setSensorModel(smObj.NumBeams, ...
                                  smObj.ExpectedMeasurementWeight, ...
                                  smObj.RandomMeasurementWeight, ...
                                  smObj.MeasurementNoise, ...
                                  smObj.MaxLikelihoodDistance, ...
                                  smObj.SensorLimits, ...
                                  smObj.SensorPose);
        end

        function [particles, weights] = getParticles(mclObj)
        %getParticles Get particles and weights from internal object
        %   getParticles(MCLOBJ) returns the particles and weights from
        %   the nav.algs.internal.MonteCarloLocalization object
        %   MCLOBJ.
            parray = mclObj.getParticles;
            pdata = reshape(parray, 4, [])';
            particles = pdata(:,1:3);
            weights = pdata(:,4);
        end

        function [pose, cov] = getHypothesis(mclObj)
        %getHypothesis Get estimated pose and covariance
        %   getHypothesis(MCLOBJ) returns the estimated pose and
        %   covariance of the highest weighted cluster of particles
        %   from the nav.algs.internal.MonteCarloLocalization
        %   object MCLOBJ.

            posecov = mclObj.getHypothesis;
            posedata = reshape(posecov, 3, [])';
            pose = posedata(1,:);
            cov = posedata(2:4, :);
        end
    end

end
