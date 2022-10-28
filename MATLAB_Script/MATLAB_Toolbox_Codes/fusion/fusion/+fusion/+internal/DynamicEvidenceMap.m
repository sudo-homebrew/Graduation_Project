classdef DynamicEvidenceMap < fusion.internal.MapInterface
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    methods (Static)
        function out = loadobj(in)
            out = dynamicEvidentialGridMap(in.Length,in.Width,in.Resolution,...
                'GridOriginInLocal',in.GridOriginInLocal,...
                'NumStateVariables',in.NumStateVariables,...
                'UseGPU',in.UseGPU);
            setOccupiedEvidence(out,in.OccupiedEvidence);
            setFreeEvidence(out,in.FreeEvidence);
            setState(out,in.State);
            setStateCovariance(out,in.StateCovariance);
            setIsDynamic(out,in.IsDynamic);
        end
    end
end