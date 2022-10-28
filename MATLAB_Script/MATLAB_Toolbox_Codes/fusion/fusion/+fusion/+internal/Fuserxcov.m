classdef Fuserxcov < fusion.internal.AbstractFusionAlgorithm
    %Fuserxcov A fuser object based on cross-covariance
    % Fuserxcov implements the fusion.internal.AbstractFusionAlgorithm
    % interface to allow the track fuser to work with it.
    
    % Copyright 2019 The MathWorks, Inc.

    %#codegen
    
    methods
        function obj = Fuserxcov(f,df,Q,hapn,ff,params)
            obj.StateTransitionFcn = f;
            obj.StateTransitionJacobianFcn = df;
            obj.ProcessNoise = Q;
            obj.HasAdditiveProcessNoise = hapn;
            obj.FusionFcn = ff;
            obj.StateFusionParameters = params;
        end
        
        function fusedTrack = fuse(obj,centralTrack,sourceTracks,inAssigned)
            %FUSE    Fuses the source tracks into a central track.
            %sourceTracks have already been converted to centralTrack
            %frame.
            
            [times,indsSortedByTime] = fusion.internal.AbstractFusionAlgorithm.sortTracksByTime(sourceTracks,inAssigned);
            uniqueTimes = unique(times);
            trackTime = centralTrack.UpdateTime;
            x = centralTrack.State;
            P = centralTrack.StateCovariance;
            for j=1:numel(uniqueTimes)
                fusionTime = uniqueTimes(j);
                dt = fusionTime - trackTime;
                [x,P] = fusion.internal.gaussEKFilter.predict(x,P,...
                        obj.ProcessNoise,obj.StateTransitionFcn,...
                        obj.StateTransitionJacobianFcn,obj.HasAdditiveProcessNoise,dt);
                tracksAtThisTime = find(fusionTime == times);
                indsAtThisTime = indsSortedByTime(tracksAtThisTime);
                numTracksToFuse = numel(tracksAtThisTime);
                allStates = repmat(x,[1 numTracksToFuse+1]);
                allCovars = repmat(P,[1 1 numTracksToFuse+1]);
                for k = 1:numTracksToFuse
                    allStates(:,k+1) = sourceTracks{indsAtThisTime(k)}.State;
                    allCovars(:,:,k+1) = sourceTracks{indsAtThisTime(k)}.StateCovariance;
                end
                if isempty(obj.StateFusionParameters)
                    [xx,PP] = obj.FusionFcn(allStates,allCovars);
                else
                    [xx,PP] = obj.FusionFcn(allStates,allCovars,obj.StateFusionParameters);
                end
                x = xx(1:numel(x));
                P = PP(1:numel(x),1:numel(x));
                trackTime = fusionTime;
            end
            fusedTrack = centralTrack;
            fusedTrack.State = x;
            fusedTrack.StateCovariance = fusion.internal.ensurePosDefMatrix(P);
            fusedTrack.UpdateTime = trackTime;
        end
    end
    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'StateTransitionFcn','StateTransitionJacobianFcn','HasAdditiveProcessNoise'};
        end
    end
end