classdef adsbReceiver< matlab.System
%adsbReceiver Automatic Dependent Surveillance - Broadcast receiver model
%   adsbrx = adsbReceiver returns an ADS-B receiver model. The model
%   maintains a list of tracked objects, associates ADS-B messages to
%   them, and outputs updated tracks based on these messages.
%
%   adsbrx = adsbReceiver('Name', value) configures the ADS-B receiver by
%   specifying its properties as name-value pair arguments. Unspecified
%   properties have default values. See the list of properties below.
%
%   adsbReceiver properties:
%
%   ReceiverIndex    - Unique identifier of the receiver
%   MaxNumTracks     - Maximum number of ADS-B tracks
%   NumTracks        - Number of maintained ADS-B tracks (Read-only)
%
%   Step method syntax: click on <a href="matlab:help('adsbReceiver/stepImpl')">step</a>
%
%   adsbReceiver methods:
%     clone             - Creates a copy of the adsbTransponder
%     deleteTrack       - Deletes ADS-B tracks
%     isLocked          - Locked status (logical)
%     release           - Allows property value and input characteristics changes
%     <a href="matlab:help matlab.System/reset   ">reset</a>             - Resets states of the adsbTransponder
%     <a href="matlab:help('adsbReceiver/stepImpl')">step</a>              - Generates ADS-B messages
%
%   Example:
%
%   %Create an ADS-B receiver
%   adsbRx = adsbReceiver;
%   % Define ADS-B messages from transponder AAF123
%   % ADS-B transponder is assumed unsynced to UTC - set Time to NaN
%   airbornePositionMessage = struct('ICAO','AAF123','Time', NaN,...
%      'Latitude',70, 'Longitude',30,'Altitude',2000);
%   % Define receive time as t0
%   t0 = 0;
%   %Receive message
%   [track, incomplete] = adsbRx(airbornePositionMessage, t0);
%   % No velocity information yet therefore a track cannot be
%   % etablished
%   disp(incomplete)
%   % Now create a new ADS-B message from the same transponder, which contains
%   % the Airborne Velocity Type. The message is received at t1 = 1 s
%   airborneVelocityMessage = struct('ICAO', 'AAF123','Time',NaN,...
%      'Vnorth',250,'Veast',0,'ClimbRate',-1);
%   t1 = 1;
%   [track, incomplete] = adsbRx(airborneVelocityMessage, t1);
%   %The receiver can now format the combination of the two messages into an
%   %objectTrack with full state
%   disp(track);
%
%   See also: adsbCategory, adsbTransponder, objectTrack

     
    %   Copyright 2020-2021 The MathWorks, Inc.

    methods
        function out=adsbReceiver
        end

        function out=confirmTracks(~) %#ok<STOUT>
            % Check for full state
        end

        function out=deleteTrack(~) %#ok<STOUT>
            %deleteTrack Delete a track managed by the adsbReceiver
            % deleted = deleteTrack(obj, id) deletes the track specified by
            % id from the receiver. id can be either the ICAO identifier
            % from an input ADS-B message or its corresponding TrackID. The
            % deleted flag returns true if a track with the same id
            % existed and was deleted. If a track with that id did not
            % exist, the deleted flag is false and a warning is issued.
            %
            % Note: the adsbReceiver must be updated at least once to be
            % able to delete an ADS-B track.
        end

        function out=extractCovariance(~) %#ok<STOUT>
            % extractCovariance Extract the covariance information from an
            % ADS-B message
            %
            % covariance = extractCovariance(adsbReceiver, message) parses
            % the ads-b message, specified as an ads-b message struct and
            % returns a 6-by-6 covariance matrix, covariance. The
            % covariance corresponds to the 6-element state [X, Vx, Y, Vy,
            % Z, Vz], where, [X, Y, Z] are the ECEF coordinates of the
            % position vector and [Vx, Vy, Vz] are the ECEF coordinates of
            % the velocity vector.
        end

        function out=extractStates(~) %#ok<STOUT>
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=icao2trackID(~) %#ok<STOUT>
            % return unique id for each icao address
        end

        function out=initializeMessages(~) %#ok<STOUT>
            % Save messages with new unique icao
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=pDeleteTrack(~) %#ok<STOUT>
            % protected deleteTrack function. No validation done at this
            % level. index is the index into obj.pMessageList and
            % obj.pICAOMap to be deleted
        end

        function out=packageICAOMapOutput(~) %#ok<STOUT>
        end

        function out=packageInfo(~) %#ok<STOUT>
        end

        function out=packageTracks(~) %#ok<STOUT>
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
        end

        function out=sortMessages(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
            %STEP Creates, updates, and deletes ADS-B tracks
            %   The step method is responsible for managing received
            %   messages and outputing tracks.
            %   1 - The method sorts incoming ADS-B messages by their
            %       timestammp and discards messages without ICAO address.
            %   2 - Updates previous information from matching ICAO addresses
            %   3 - Stores new information.
            %   4 - Promotes ADS-B data to objectTrack when there is enough
            %       information to form a complete state with 3D position
            %       and velocity information.
            %
            %   tracks = STEP(receiver, messages, time) updates the receiver
            %   with a list of ADS-B messages to the time of reception
            %   specified by time. It returns an array of confirmed tracks
            %   if the messages contained enough information. ADS-B
            %   messages must be specified as an array of ADS-B message
            %   structs as defined <a href="matlab:help fusion.internal.interfaces.DataStructures/adsbMessageStruct">here</a>.
            %   Each message can generate at most one track. If the
            %   information contained in a message does not allow to form a
            %   full track state, no track will be output for this message.
            %
            %   [tracks, incomplete] = STEP(...) additionally outputs an
            %   array of message structs, incomplete, containing information received
            %   from transponders for which it is not possible yet to derive a full
            %   state.
            %
            %   [tracks, incomplete, info] = STEP(...) additionally
            %   outputs an information struct, info, which contain the following
            %   fields:
            %
            %   Discarded        - List of indices of discarded messages
            %   IcaoToTrackID    - Mapping of ICAO addresses to TrackIDs
        end

        function out=structToTracks(~) %#ok<STOUT>
            % Default Object Track has proper State size
        end

        function out=updateMessages(~) %#ok<STOUT>
        end

        function out=validICAO(~) %#ok<STOUT>
        end

        function out=validateMessageStruct(~) %#ok<STOUT>
            % messages must be a struct array
        end

    end
    properties
        %MaxNumTracks   Maximum number of ADS-B tracks
        %   Set the maximum number of ADS-B tracks the adsbReceiver can maintain
        %   as a positive real integer.
        %
        %   Default: 100
        MaxNumTracks;

        %NumTracks  Number of ADS-B tracks
        %   The total number of ADS-B tracks the adsbReceiver is maintaining.
        %
        %   This value is calculated by the receiver and is read-only.
        NumTracks;

        %ReceiverIndex Unique identifier of the receiver
        %   Specify the unique index associated with this receiver in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        %   Default: uint32(0)
        ReceiverIndex;

        %pGVATable Gemoetric Vertical Accuracy table
        pGVATable;

        %pICAOMap Mapping between ICAO and TrackID
        % pICAOMap is a N element string array. Each element is an
        % ICAO id defined as 6-element char vectors
        pICAOMap;

        %pICAOTemplate template to build ICAO char arrays
        pICAOTemplate;

        % pLastTimeStamp Keeps the last time to which the receiver was updated
        pLastTimeStamp;

        %pLastTrackID last trackID used
        pLastTrackID;

        %pMessageTemplate template to build messages
        pMessageTemplate;

        %pMessageList Internal list of ADS-B messages
        % pMessageList is of length N where N is the number of rows in
        % pICAOMap. pMessageList(i) is an ADS-B struct. Messages are
        % converted to objectTracks upon promotion in the step method.
        pMessagesList;

        %pNACpTable Navigation Accuracy for Position table
        pNACpTable;

        %pNACvTable Navigation Accuracy for Velocity table
        pNACvTable;

        %pTrackIDMap Mapping between ICAO and TrackID
        % pTrackID is a N-element vector of uint32 track IDs.
        pTrackIDMap;

        %pTrackTemplate template to build tracks
        pTrackTemplate;

    end
end
