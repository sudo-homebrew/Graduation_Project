classdef factorIMUBiasPrior  < nav.algs.internal.FactorGaussianNoiseModel
%FACTORIMUBIASPRIOR Creates a prior factor for IMU bias
%   F = FACTORIMUBIASPRIOR(ID) returns a factorIMUBiasPrior object, F, with
%   the node identification number set to ID. The measurement represents a
%   a prior on IMU bias (in the format of [bias_gyro, bias_accel]). By
%   default, the prior value is set to [0,0,0, 0,0,0] and the corresponding
%   information matrix is set to eye(6).
%   
%   F = FACTORIMUBIASPRIOR(...,Name=Value,...) returns a factorIMUBiasPrior
%   object, F, with each specified property name set to the specified
%   value. You can specify additional name-value pair arguments in any
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORIMUBIASPRIOR properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured IMU biases in [bias_gyro,bias_accel]
%       Information       - Information matrix associated with measurement
%
%   Example:
%       % Add an IMU bias prior factor with a node ID of 1 to a factor 
%       % graph.
%       f = factorIMUBiasPrior(1);
%       g = factorGraph;
%       addFactor(g,f);
%
%   See also factorGraph.

%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "IMU_Bias_Prior_F";
    end
    
    methods
        function obj = factorIMUBiasPrior(id, varargin)
            %FACTORIMUBIASPRIOR Constructor;
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(id, 1, 'factorIMUBiasPrior', [0,0,0, 0,0,0], eye(6), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.IMUBias;
        end
    end
end

