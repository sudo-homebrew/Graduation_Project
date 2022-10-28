classdef factorTwoPoseSE3 < nav.algs.internal.FactorGaussianNoiseModel
%FACTORTWOPOSESE3 Creates a factor that relates two SE(3) poses
%
%   F = FACTORTWOPOSESE3(ID) returns a factorTwoPoseSE3 object, F, with the
%   node identification number set to ID. The measurement represents a
%   realtive pose in SE3. By default the measurement is set to
%   [0,0,0,1,0,0,0] (in [dx,dy,dz,dqw,dqx,dqy,dqz] format) and the
%   corresponding information matrix is set to eye(6).
%
%   F = FACTORTWOPOSESE3(...,Name=Value,...) returns a factorTwoPoseSE3
%   object, F, with each specified property name set to the specified
%   value. You can specify additional name-value pair arguments in any
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORTWOPOSESE3 methods:
%       nodeType          - Retrieve the node type for a specified node ID
%
%   FACTORTWOPOSESE3 properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured relative pose in [x,y,z,qw,qx,qy,qz]
%       Information       - Information matrix associated with measurement
%
%   See also factorGraph.

%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "Two_SE3_F";
    end
    
    methods
        function obj = factorTwoPoseSE3(ids, varargin)
            %FACTORTWOPOSESE3 Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(ids, 2, 'factorTwoPoseSE3', [0,0,0,1,0,0,0], eye(6), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.SE3;
        end
    end
end

