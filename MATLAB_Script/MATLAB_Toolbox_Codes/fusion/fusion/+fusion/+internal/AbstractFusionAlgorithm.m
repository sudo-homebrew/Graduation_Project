classdef (Abstract) AbstractFusionAlgorithm < handle
% This is an internal function and may be removed or modified in a future 
% release.

    % Copyright 2019 The MathWorks, Inc.

    %#codegen

    properties
        StateFusionParameters
        StateTransitionFcn
        StateTransitionJacobianFcn
        ProcessNoise
        HasAdditiveProcessNoise
        FusionFcn
    end
    methods (Abstract)
        fusedTrack = fuse(centralTrack,sourceTracks,assignment)
    end
    methods
        function set.FusionFcn(obj,value)
            validateattributes(value,{'string','char','function_handle'},...
                {},'','StateFusion');
            if isa(value,'function_handle')
                obj.FusionFcn = value;
            else
                obj.FusionFcn = str2func(value);
            end
        end
    end
    methods (Static)
        function [times,indsSortedByTime] = sortTracksByTime(tracks,inds)
            % Returns the indices of tracks sorted by time. 
            if coder.target('MATLAB')
                allTracks = [tracks{inds}];
                allTimes = [allTracks.UpdateTime];
            else
                numTracks = numel(inds);
                allTimes = repmat(tracks{inds(1)}.UpdateTime,1,numTracks);
                for i = 2:numTracks
                    allTimes(i) = tracks{inds(i)}.UpdateTime;
                end
            end
            [times,I] = sort(allTimes,'ascend');
            indsSortedByTime = inds(I);
        end
    end
end