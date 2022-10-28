classdef(Hidden) FactorGaussianNoiseModel
%This class is for internal use only. It may be removed in the future.

%FACTORGAUSSIANNOISEMODEL Represent a general factor whose mesurement noise
%   follows Gaussian distribution.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess=protected)
        %NodeID Node ID number in factor graph
        %
        %   Must be specified at construction
        NodeID
    end

    properties (Dependent)
        %Measurement Measurement values for the factor
        Measurement
        
        %Information The information matrix associated with the measurement
        Information
    end

    properties (Access=protected)
        NumNodes

        FactorName

        MeasurementSize

        InformationSize

        DefaultMeasurement

        DefaultInformation
    end

    properties (Access=protected)
        MeasurementInternal

        InformationInternal
    end

    methods (Access=protected, Abstract)
        nodeTypeImpl(obj, id)
    end

    methods
        function obj = FactorGaussianNoiseModel(ids, numNodes, factorName, defaultMeasurement, defaultInformation, varargin )
            %FACTORGAUSSIANNOISEMODEL Constructor
            
            nav.algs.internal.validation.validateNodeID_FactorConstruction(ids, numNodes, factorName, 'ids');

            obj.FactorName = factorName;
            obj.MeasurementInternal = defaultMeasurement;
            obj.InformationInternal = defaultInformation;
            obj.MeasurementSize = size(defaultMeasurement);
            obj.InformationSize = size(defaultInformation);

            obj = matlabshared.fusionutils.internal.setProperties( ...
                obj, nargin-5, varargin{:});

            obj.NodeID = double(ids);
            
        end

        function obj = set.Measurement(obj, measurement)
            %set.Measurement Setter for Measurement property
            obj.validateMeasurement(measurement);
            obj.MeasurementInternal = double(measurement);
        end

        function obj = set.Information(obj, info)
            %set.Information Setter for Information property
            obj.validateInformation(info);
            obj.InformationInternal = info;
        end

        function measurement = get.Measurement(obj)
            %get.Measurement Getter for Measurement property
            measurement = obj.MeasurementInternal;
        end

        function info = get.Information(obj)
            %set.Information Setter for Information property
            info = obj.InformationInternal;
        end

        function type = nodeType(obj, id)
            %nodeType Retrieve the node type for a specified node ID.
            %
            %   TYPE = NODETYPE(F,ID) returns the node type, TYPE, for the
            %   node with the number ID.

            narginchk(2,2);
            nav.algs.internal.validation.validateNodeID_FactorQuery(id, obj.NodeID, obj.FactorName, 'id');
            type = obj.nodeTypeImpl(id);
        end
    end

    methods (Access=private)
        function validateMeasurement(obj, measurement)
            %validateMeasurement
            validateattributes(measurement, 'numeric', ...
                {'size', obj.MeasurementSize, 'real', 'finite', 'nonsparse'}, obj.FactorName, 'measurement')
        end

        function validateInformation(obj, info)
            %validateInformation
            validateattributes(info, 'numeric', ...
                {'size', obj.InformationSize, 'real', 'finite', 'nonsparse'}, obj.FactorName, 'information');
        end

    end
end

