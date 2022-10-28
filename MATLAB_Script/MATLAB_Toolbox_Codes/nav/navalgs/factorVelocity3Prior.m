classdef factorVelocity3Prior  < nav.algs.internal.FactorGaussianNoiseModel
%FACTORVELOCITY3PRIOR Creates a prior factor for 3D velocity
%   F = factorVelocity3Prior(ID) returns a factorVelocity3Prior object, F,
%   with the node identification number set to ID. The measurement
%   represents a a prior on velocity in 3D (in the format of [vx,vy,vz]).
%   By default, the prior value is set to [0,0,0] and the corresponding
%   information matrix is set to eye(3).
%   
%   F = FACTORVELOCITY3PRIOR(...,Name=Value,...) returns a
%   factorVelocity3Prior object, F, with each specified property name set
%   to the specified value. You can specify additional name-value pair
%   arguments in any order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORVELOCITY3PRIOR properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured velocity in [vx,vy,vz]
%       Information       - Information matrix associated with measurement
%
%   Example:
%       % Add a velocity prior to a factor graph.
%       f = factorVelocity3Prior(1);
%       g = factorGraph;
%       addFactor(g,f);
%
%   See also factorGraph.

%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "Vel3_Prior_F";
    end
    
    methods
        function obj = factorVelocity3Prior(id, varargin)
            %FACTORVELOCITY3PRIOR Constructor;
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(id, 1, 'factorVelocity3Prior', [0,0,0], eye(3), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.Velocity3;
        end
    end
end

