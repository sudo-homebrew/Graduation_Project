classdef factorPoseSE2Prior  < nav.algs.internal.FactorGaussianNoiseModel
%FACTORPOSESE2PRIOR Creates a full-state prior factor for an SE(2) pose
%
%   F = FACTORPOSESE2PRIOR(ID) returns a factorPoseSE2Prior object, F, with
%   the node identification number set to ID. The measurement represents a
%   absolute SE2 pose prior. By default, the measurement is set to [0,0,0]
%   and the corresponding information matrix is set to eye(3).
%
%   F = FACTORPOSESE2PRIOR(...,Name=Value,...) returns a factorPoseSE2Prior
%   object, F, with each specified property name set to the specified
%   value. You can specify additional name-value pair arguments in any
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   See also factorGraph.

%   Copyright 2021 The MathWorks, Inc.
    
    properties (Hidden, Constant)
        FactorType = "SE2_Prior_F";
    end

    methods
        function obj = factorPoseSE2Prior(id, varargin)
            %FACTORPOSESE2PRIOR Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(id, 1, 'factorPoseSE2Prior', [0,0,0], eye(3), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.SE2;
        end
    end
end