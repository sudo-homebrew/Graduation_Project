classdef LabelManagementLogic
    % This is an internal class and may be removed or modified in future.
    
    % Copyright 2018 The MathWorks, Inc.
    
    %#codegen
    
    
    properties
        WeightThresholds;
    end
    
    methods
        function obj = LabelManagementLogic(w)
           obj.WeightThresholds = w; 
        end
        % Check if from a labeled density, which components should be
        % deleted, and which should be unlabeled.
        function [toPrune, toUnlabel] = manageHypothesis(obj,density)
            % density is a PHD filter with components for a single
            % track i.e. it's hypothesis.
            densityWeights = density.Weights;
            n = density.NumComponents;
            w = obj.WeightThresholds;
            [maxWeight, index] = max(densityWeights);
            if sum(densityWeights) > w(1)
                scale(density,w(1)/sum(densityWeights));
            end
            toPrune = false(n,1);
            toUnlabel = false(n,1);
            condition1 = maxWeight > w(2);
            condition2 = maxWeight/sum(densityWeights) > w(3);
            if condition1 || condition2
                toPrune = true(n,1);
            else
                toUnlabel = true(n,1);
            end
            toPrune(index) = false;
            toUnlabel(index) = false;
        end
    end
end