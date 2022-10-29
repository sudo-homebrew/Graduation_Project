classdef fuserSourceConfiguration< fusion.internal.AbstractFusingConfiguration
%fuserSourceConfiguration  Configuration of a source used with track fuser
%  config = fuserSourceConfiguration(sourceIndex) creates a configuration
%  to be used with a track fuser. You must specify sourceIndex as a
%  positive integer. The other properties of the configuration are set to
%  default values.
%
%  config = fuserSourceConfiguration(...,'Name',value) allows specifying
%  additional properties using name-value pairs. 
%
%  fuserSourceConfiguration properties:
%    SourceIndex                 - Unique identifier for the source system
%    IsInternalSource            - Define if the source is internal to the 
%                                  fuser
%    IsInitializingCentralTracks - Define if this source can initialize a 
%                                  central track
%    LocalToCentralTransformFcn  - A function used to transform a local
%                                  track to a central track
%    CentralToLocalTransformFcn  - A function used to transform a central
%                                  local to a source track
%
% % Example:
% % ========
% % Create a fusion configuration for source number 3
% config = fuserSourceConfiguration(3);
% disp(config)
%
% See also: trackFuser, objectTrack

 
% Copyright 2019 The MathWorks, Inc.

    methods
        function out=fuserSourceConfiguration
            % Is SourceIndex specified as property or first variable
        end

        function out=parseCodegen(~) %#ok<STOUT>
            % Define parser
        end

        function out=parseMATLAB(~) %#ok<STOUT>
            % Define parser
        end

        function out=setProperties(~) %#ok<STOUT>
        end

        function out=validateSimilarTracks(~) %#ok<STOUT>
            % For tracks to be similar, check that their state and state
            % covariance sizes are the same
        end

        function out=validateTransformToCentral(~) %#ok<STOUT>
            % This transform can fail in three ways: 
            %  1. The function provided by the user does not support
            %     objectTrack. In that case that error provided will be
            %     clear enough: 'MATLAB:UndefinedFunction'.
            %  2. The function works on objectTrack but returns a value
            %     that is not an objectTrack.
            %  3. The function returns an objectTrack but is not the
            %     inverse of trasnformation to local. 
            % For the last two validations, we use validateTransforms
        end

        function out=validateTransformToLocal(~) %#ok<STOUT>
            % This transform can fail in three ways: 
            %  1. The function provided by the user does not support
            %     objectTrack. In that case that error provided will be
            %     clear enough: 'MATLAB:UndefinedFunction'.
            %  2. The function works on objectTrack but returns a value
            %     that is not an objectTrack.
            %  3. The function returns an objectTrack but is not the
            %     inverse of trasnformation to local. 
            % For the last two validations, we use validateTransforms
        end

    end
    properties
        %CentralToLocalTransformFcn Function to transform a track from
        %                           central to local state space
        % Define the function that transforms a track from fuser coordinate
        % frame to source coordinate frame.
        %
        % Default: @(track)track
        CentralToLocalTransformFcn;

        %IsInitializingCentralTracks Define if this source can initialize a 
        %                            central track
        % Set this flag to true if the tracks coming from this source can
        % initialize a central track. 
        %
        % Default: true
        IsInitializingCentralTracks;

        %IsInternalSource Define if the source is internal to the fuser
        % Set this flag to true if the source is considered as internal to
        % the track fuser. The tracks from an internal source are fused
        % even if they are not self reported. In contrast, set this flag to
        % false to prevent the fuser from fusing tracks that are not self
        % reported (i.e., they are repeated and may cause rumors) by the
        % source.
        % For example, if the fuser is at the vehicle level, a tracking
        % radar installed on this vehicle is considered internal, while
        % another vehicle that reports fused tracks is considered external.
        %
        % Default: true
        IsInternalSource;

        %LocalToCentralTransformFcn Function to transform a track from
        %                           local to central state space
        % Define the function that transforms a track from source
        % coordinate frame to fuser coordinate frame. 
        %
        % Default: @(track)track
        LocalToCentralTransformFcn;

        %SourceIndex Unique identifier for the source system
        % Define the source index as a positive integer. 
        %
        % Default: none
        SourceIndex;

        pCentralToLocalTransformFcn;

        pIsTransformToCentralValid;

        pIsTransformToLocalValid;

        pLocalToCentralTransformFcn;

    end
end
