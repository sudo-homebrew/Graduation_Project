classdef factorPoseSE2AndPoint2 < nav.algs.internal.FactorGaussianNoiseModel
%FACTORPOSESE2ANDPOINT2 Creates a factor object that relates an SE(2) pose
%  and a 2D point.
%
%   F = FACTORPOSESE2ANDPOINT2(ID) returns a factorPoseSE2AndPoint2 object,
%   F, with the node identification number set to ID. The measurement
%   represents a relative position [dx,dy]. By default the measurement is
%   set to [0,0] and the corresponding information matrix is set to eye(2).
%
%   F = FACTORPOSESE2ANDPOINT2(...,Name=Value,...) returns a
%   factorPoseSE2AndPoint2 object, F, with each specified property name set
%   to the specified value. You can specify additional name-value pair
%   arguments in any order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORPOSESE2ANDPOINT2 methods:
%       nodeType          - Retrieve the node type for a specified node ID
%
%   FACTORPOSESE2ANDPOINT2 properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured relative position [dx,dy]
%       Information       - Information matrix associated with measurement
%
%   See also factorGraph.
    
%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "SE2_Point2_F";
    end
    
    methods
        function obj = factorPoseSE2AndPoint2(ids, varargin)
            %FACTORPOSESE2ANDPOINT2 Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(ids, 2, 'factorPoseSE2AndPoint2', zeros(1,2), eye(2), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(obj, id)
            if find(obj.NodeID == id) == 1
                type = nav.internal.factorgraph.NodeTypes.SE2;
            else
                type = nav.internal.factorgraph.NodeTypes.Point2;
            end
        end
    end
end

