classdef(Abstract) TrackFuser < handle
    %fusion.trackingArchitecture.TrackFuser Interface definition for trackingArchitecture track fuser
    % This abstract class defines the interface used for a track-to-track
    % fusion object that interfaces with the trackingArchitecture feature.
    % 
    % To add a track-to-track fusion object to the trackingArchitecture, it
    % must inherit from this class.
    %
    % It must implement the following abstract properties:
    %   FuserIndex    - Provide the track-to-track fusion algorithm index
    %
    % It must implement the following abstract methods (see syntax for each):
    %   step          - Run the track-to-track fusion object
    %   isLocked      - Return true if the track-to-track fusion object was stepped
    %   reset         - Reset the states of the track-to-track fusion object
    %   release       - Allow property value and input characterstics changes
    %   clone         - Create a copy of the track-to-track fusion object
    %   sourceIndices - Return the indices of sources to the track-to-track fusion object
    %
    % The following syntax is expected for each method:
    %   confirmedTracks = step(obj,localTracks, time)
    %   tf = isLocked(obj)
    %   reset(obj)
    %   release(obj)
    %   clonedTrackFuser = clone(obj)
    %
    % Example: Add a fuser that inherits from this class to a trackingArchitecture
    % % Create an architecture
    % arch = trackingArchitecture;
    % % To add an object that inherits from this class, you would follow
    % % the lines commented out below:
    % % myFuser = fuser;
    % % addTrackFuser(arch, myFuser);
    %
    % See also: trackingArchitecture, fusion.trackingArchitecture.Tracker
    
    % Copyright 2020 The MathWorks, Inc.

    properties(Abstract)
        FuserIndex
    end
    
    methods(Abstract)
        confirmedTracks = step(obj,tracks, time);
        tf = isLocked(obj);
        reset(obj);
        release(obj);
        clonedTrackFuser = clone(obj);
        inds = sourceIndices(obj);
    end
end