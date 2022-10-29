classdef factorTwoPoseSE2 < nav.algs.internal.FactorGaussianNoiseModel
%FACTORTWOPOSESE2 Creates a factor object that relates two SE(2) poses
%
%   F = FACTORTWOPOSESE2(ID) returns a factorTwoPoseSE2 object, F, with the
%   node identification number set to ID. The measurement represents a
%   relative pose in SE2. By default the measurement is set to [0,0,0] and 
%   the corresponding information matrix is set to eye(3).
%
%   F = FACTORTWOPOSESE2(...,Name=Value,...) returns a factorTwoPoseSE2
%   object, F, with each specified property name set to the specified 
%   value. You can specify additional name-value pair arguments in any 
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORTWOPOSESE2 methods:
%       nodeType          - Retrieve the node type for a specified node ID
%
%   FACTORTWOPOSESE2 properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured relative pose [x,y,theta]
%       Information       - Information matrix associated with measurement
%
%   Example:
%       % Add a relative 2D pose factor to a factor graph.
%       f = factorTwoPoseSE2([1 2]);
%       g = factorGraph;
%       addFactor(g,f);
%
%   See also factorGraph.
    
%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "Two_SE2_F";
    end
    
    methods
        function obj = factorTwoPoseSE2(ids, varargin)
            %FACTORTWOPOSESE2 Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(ids, 2, 'factorTwoPoseSE2', zeros(1,3), eye(3), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(~, ~)
            type = nav.internal.factorgraph.NodeTypes.SE2;
        end
    end
end

