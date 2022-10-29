classdef(Abstract) Tracker < handle
    %fusion.trackingArchitecture.Tracker Interface definition for trackingArchitecture tracker
    % This abstract class defines the interface used for a tracker that
    % interfaces with the trackingArchitecture feature.
    % 
    % To add a tracker to the trackingArchitecture, it must inherit from
    % this class.
    %
    % It must implement the following abstract properties:
    %   TrackerIndex - Provide the tracker index
    %
    % It must implement the following abstract methods (see syntax below):
    %   step     - Run the tracker
    %   isLocked - Returns true if the tracker was stepped
    %   reset    - Reset the states of the tracker
    %   release  - Allow property value and input characterstics changes
    %   clone    - Create a copy of the tracker
    %
    % The following syntax is expected for each method:
    %   tracks = step(obj,detections,time)
    %   tf = isLocked(obj)
    %   reset(obj)
    %   release(obj)
    %   clonedTracker = clone(obj)
    %
    % Example: Add a tracker that inherits from this class to a trackingArchitecture
    % arch = trackingArchitecture;
    % % To add an object that inherits from this class, you would follow
    % % the lines commented out below:
    % % myTracker = tracker;
    % % addTracker(arch, myTracker, 'SensorIndices', [1 2]);
    %
    % See also: trackingArchitecture,fusion.trackingArchitecture.TrackFuser
    
    % Copyright 2020 The MathWorks, Inc.

    properties(Abstract)
        TrackerIndex
    end
    
    methods(Abstract)
        confirmedTracks = step(obj,detections, time);
        tf = isLocked(obj);
        reset(obj)
        release(obj)
        clonedTracker = clone(obj)
    end
end