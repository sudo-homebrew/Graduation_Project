classdef factorPoseSE3Prior  < nav.algs.internal.FactorGaussianNoiseModel
%FACTORPOSESE3PRIOR Creates a full-state prior factor for an SE(3) pose
%
%   F = FACTORPOSESE3PRIOR(ID) returns a factorPoseSE3Prior object, F, with
%   the node identification number set to ID. The measurement represents a
%   absolute SE3 pose prior (in local coordinates). By default, the
%   measurement is set to [0,0,0,1,0,0,0] and the corresponding information
%   matrix is set to eye(6).
%
%   F = FACTORPOSESE3PRIOR(...,Name=Value,...) returns a factorPoseSE3Prior
%   object, F, with each specified property name set to the specified
%   value. You can specify additional name-value pair arguments in any
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORPOSESE3PRIOR properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured SE3 pose in [x,y,z,qw,qx,qy,qz]
%       Information       - Information matrix associated with measurement
%
%   Example:
%       % Add a 3D pose prior factor with a node ID of 1 to a factor graph.
%       f = factorPoseSE3Prior(1);
%       g = factorGraph;
%       addFactor(g,f);
%
%   See also factorGraph.

%   Copyright 2021 The MathWorks, Inc.
    
    properties (Hidden, Constant)
        FactorType = "SE3_Prior_F";
    end

    methods
        function obj = factorPoseSE3Prior(id, varargin)
            %FACTORPOSESE3PRIOR Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(id, 1, 'factorPoseSE3Prior', [0,0,0,1,0,0,0], eye(6), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.SE3;
        end
    end
end

