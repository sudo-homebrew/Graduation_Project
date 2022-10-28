classdef (Abstract) AbstractPHDFilter < handle
    % This is an internal class and may be modified or removed in a future
    % release. 
    
    % Copyright 2019-2020 The MathWorks, Inc.
    
    %#codegen
    properties (Abstract)
        Weights
        Detections
        Labels
    end
    properties (Abstract,SetAccess = private)
        NumComponents
    end
    
    methods (Abstract)
        predict(obj,dT)
        lhood = likelihood(obj,detectionIndices,sensorConfig);
        correct(obj,detectionIndices,detectionLikelihoods,sensorConfig)
        correctUndetected(obj,Pd);
        merge(obj,threshold)
        [states,indices] = extractState(obj,threshold)
        labeledDensity(obj,threshold)
        prune(obj,toPrune)
        sync(obj,obj2)
        obj2 = clone(obj)
        append(obj,obj2);
        nullify(obj)
        scale(obj,a)
        [state,covs] = sigmaPoints(obj)
        stm = models(obj)
        structState = sampleStruct(obj);
        PzeroDets = probZeroDetections(obj,sensorConfig);
    end
    
    methods (Static, Abstract)
        phd = initializeFromTrack(track);
    end
end