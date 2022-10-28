classdef adsbReceiver < matlab.System
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
    %     reset             - Resets states of the adsbTransponder
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
    
    % Reference:
    % ICAO, Doc. "9871: Technical Provisions for Mode S Services and Extended Squitter, AN/464." (2008).
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties
        %ReceiverIndex Unique identifier of the receiver
        %   Specify the unique index associated with this receiver in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        %   Default: uint32(0)
        ReceiverIndex = uint32(0)
    end
    
    properties (Nontunable)
        %MaxNumTracks   Maximum number of ADS-B tracks
        %   Set the maximum number of ADS-B tracks the adsbReceiver can maintain
        %   as a positive real integer.
        %
        %   Default: 100
        MaxNumTracks (1, 1)  {mustBeNumeric, mustBeFinite, mustBePositive, mustBeInteger} = 100
    end
    
    
    properties(SetAccess = protected)
        %NumTracks  Number of ADS-B tracks
        %   The total number of ADS-B tracks the adsbReceiver is maintaining.
        %
        %   This value is calculated by the receiver and is read-only.
        NumTracks = 0;
        
    end
    
    properties(Access = protected)
        
        % pLastTimeStamp Keeps the last time to which the receiver was updated
        pLastTimeStamp (1,1)
        
        %pNACpTable Navigation Accuracy for Position table
        pNACpTable = [inf 18.52e3 7.408e3 3.704e3 1852 926 555.6 185.2 92.6 30 10 3]
        
        %pNACvTable Navigation Accuracy for Velocity table
        pNACvTable = [inf 10 3 1 0.3]
        
        %pGVATable Gemoetric Vertical Accuracy table
        pGVATable = [inf 150 45]
        
        %pICAOMap Mapping between ICAO and TrackID
        % pICAOMap is a N element string array. Each element is an
        % ICAO id defined as 6-element char vectors
        pICAOMap
        
        %pTrackIDMap Mapping between ICAO and TrackID
        % pTrackID is a N-element vector of uint32 track IDs.
        pTrackIDMap
        
        %pMessageList Internal list of ADS-B messages
        % pMessageList is of length N where N is the number of rows in
        % pICAOMap. pMessageList(i) is an ADS-B struct. Messages are
        % converted to objectTracks upon promotion in the step method.
        pMessagesList
        
        %pLastTrackID last trackID used
        pLastTrackID
        
        %pMessageTemplate template to build messages
        pMessageTemplate
        
        %pTrackTemplate template to build tracks
        pTrackTemplate
        
        %pICAOTemplate template to build ICAO char arrays
        pICAOTemplate = '      '
        
    end
    
    %% Public methods
    methods
        function obj = adsbReceiver(varargin)
            setProperties(obj,nargin, varargin{:});
        end
        
        function set.ReceiverIndex(obj,value)
            validateattributes(value,{'numeric'},{'real','finite','nonsparse',...
                'nonnegative','integer','scalar'},class(obj),'ReceiverIndex')
            obj.ReceiverIndex = uint32(value);
        end
        
        function deleted = deleteTrack(obj, id)
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
            
            % Validate input
            validateattributes(id,{'string','char','numeric'},{},...
                'deleteTrack','id');
            if isnumeric(id)
                validateattributes(id, {'numeric'},{'real','positive','scalar','integer'},...
                    'deleteTrack','id');
                trackid = id;
            else
                icaostr = string(id);
                validateattributes(icaostr,{'string'},{'scalar'},...
                    'deleteTrack','id');
                trackid = obj.pTrackIDMap(strcmp(obj.pICAOMap, id));
            end
            
            if coder.target('MATLAB')
                index = find(obj.pTrackIDMap == trackid, 1);
            else
                index = find(obj.pTrackIDMap == repmat(trackid(1),size(obj.pTrackIDMap)), 1);
            end
            
            if isempty(index)
                coder.internal.warning('fusion:adsb:TrackIDNotFound',id);
                deleted = false;
            else
                deleted = pDeleteTrack(obj, index(1));
            end
            
        end
    end
    
    %% System Object implementation
    methods (Access = protected)
        function releaseImpl(obj)
            obj.NumTracks = 0;
        end
        
        function setupImpl(obj, ~, time)
            obj.pLastTimeStamp = cast(-eps, 'like',time);
            % Build all templates
            messageTemplate = fusion.internal.interfaces.DataStructures.adsbMessageStruct;
            messageTemplate.Age = uint32(1); %objectTrack Age starts at 1
            obj.pMessageTemplate = messageTemplate;
            objAttrTemplate = struct('Callsign','        ','Category',adsbCategory(0));
            obj.pTrackTemplate = toStruct(objectTrack('ObjectAttributes',objAttrTemplate));
        end
        
        function resetImpl(obj)
            obj.pMessagesList = repmat(obj.pMessageTemplate, 1, obj.MaxNumTracks);
            obj.pLastTrackID = uint32(0);
            obj.pLastTimeStamp = cast(-eps, 'like',obj.pLastTimeStamp);
            obj.pICAOMap = repmat({obj.pICAOTemplate},obj.MaxNumTracks, 1);
            obj.pTrackIDMap = zeros(1, obj.MaxNumTracks, 'uint32');
            obj.NumTracks = 0;
        end
        
        function [tracksOut, incomplete, info] = stepImpl(obj, messages, time)
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
            
            % Error out if the time input is not greater than obj.
            coder.internal.errorIf(time <= obj.pLastTimeStamp, ...
                'fusion:adsb:TimeMustIncrease','step');
            
            % Validate and pad messages
            messages = validateMessageStruct(obj, messages);
            
            % Discard messages without valid icao
            keep = validICAO(obj, messages);
            if any(keep)
                messages = messages(keep);
                
                % Sort messages by time
                messages = sortMessages(obj, messages, time);
                
                % Determine new and existing ICAO addresses
                [trackIDs, new]  = icao2trackID(obj,messages);
                
                % Initialize new messages
                initTrackIDs = initializeMessages(obj, messages(new), trackIDs(new));
                
                % Update existing messages
                updateMessages(obj, messages(~new));
                
                % Check complete tracks among the incoming messages
                complete = confirmTracks(obj, [trackIDs(~new), initTrackIDs]);
                
                % Output confirmed tracks
                tracks = packageTracks(obj, trackIDs(complete));
                
                % Output buffered messages
                if all(complete)
                    incomplete = repmat(obj.pMessageTemplate, 1, 0);
                else
                    incomplete = obj.pMessagesList(trackIDs(~complete));
                end
            else
                tracks = repmat(obj.pTrackTemplate, 1, 0);
                incomplete = repmat(obj.pMessageTemplate, 1, 0);
            end
            
            % Format tracks
            if ~coder.target('MATLAB')
                % keep struct for codegen
                tracksOut = tracks;
            else
                tracksOut = structToTracks(obj,tracks);
            end
            
            % Package info output
            info = packageInfo(obj, keep);
            
            % Record time
            obj.pLastTimeStamp = time;
        end
        
        function num = getNumInputsImpl(~)
            num = 2;
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            
            s.pNACpTable       = obj.pNACpTable;
            s.pNACvTable       = obj.pNACvTable;
            s.pGVATable        = obj.pGVATable;
            
            s.pICAOTemplate    = obj.pICAOTemplate;
            
            if isLocked(obj)
                s.pMessageTemplate = obj.pMessageTemplate;
                s.pTrackTemplate   = obj.pTrackTemplate;
                
                s.pLastTimeStamp   = obj.pLastTimeStamp;
                s.pICAOMap         = obj.pICAOMap;
                s.pTrackIDMap      = obj.pTrackIDMap;
                s.pLastTrackID     = obj.pLastTrackID;
                s.pMessagesList    = obj.pMessagesList;
                s.NumTracks        = obj.NumTracks;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            
            obj.pNACpTable       = s.pNACpTable;
            obj.pNACvTable       = s.pNACvTable;
            obj.pGVATable        = s.pGVATable;
            
            obj.pICAOTemplate    = s.pICAOTemplate;
            
            if wasLocked
                obj.pTrackTemplate   = s.pTrackTemplate;
                obj.pMessageTemplate = s.pMessageTemplate;
                
                obj.pLastTimeStamp   = s.pLastTimeStamp;
                obj.pICAOMap         = s.pICAOMap;
                obj.pTrackIDMap      = s.pTrackIDMap;
                obj.pLastTrackID     = s.pLastTrackID;
                obj.pMessagesList    = s.pMessagesList;
                obj.NumTracks        = s.NumTracks;
            end
            
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
    end
    
    %%
    methods(Access = protected)
        function keep = validICAO(~, messages)
            has6char = arrayfun(@(x) numel(char(x.ICAO)), messages) == 6;
            coder.internal.errorIf(any(~has6char), 'fusion:adsb:InvalidString','ICAO',6);
            isempty6char = strcmp({messages.ICAO},'      ');
            keep = has6char & ~isempty6char;
        end
        
        function deleted = pDeleteTrack(obj,index)
            % protected deleteTrack function. No validation done at this
            % level. index is the index into obj.pMessageList and
            % obj.pICAOMap to be deleted
            
            temp = [obj.pMessagesList(1:index - 1), obj.pMessagesList(index+1:end),  obj.pMessageTemplate];
            obj.pMessagesList = temp(1:numel(obj.pMessagesList));
            obj.pICAOMap{index} = '      ';
            obj.pTrackIDMap(index) = uint32(0);
            obj.NumTracks = obj.NumTracks - 1;
            deleted = true;
            
        end
        
        function sortedMessages = sortMessages(obj, messages, time)
            if isempty(messages)
                sortedMessages = messages;
            else
                messageTimes = arrayfun(@(x) x.Time, messages);
                maxTime = max(messageTimes,[],2, 'omitnan');
                minTime = min(messageTimes,[],2, 'omitnan');
                
                coder.internal.errorIf(minTime <= obj.pLastTimeStamp || maxTime > time,...
                    'fusion:adsb:MessageTimeMismatch','step')
                
                [sortedTimes, sortedIndices] = sort(messageTimes);
                sortedMessages = messages(sortedIndices);
                % Messages without time info use receiver time
                for i=1:numel(sortedMessages)
                    if isnan(sortedTimes(i))
                        sortedMessages(i).Time = time;
                    end
                end
            end
        end
        
        function [trackID,new] = icao2trackID(obj, messages)
            % return unique id for each icao address
            N = numel(messages);
            trackID = zeros(1,N,'uint32');
            new = zeros(1,N,'logical');
            icaos = repmat({obj.pICAOTemplate}, 1, N);
            for i =1:N
                % find if this address has been received before
                mapID = find(strcmp(obj.pICAOMap,messages(i).ICAO),1);
                % check for redundant messages
                alreadySeen = any(strcmp(icaos, messages(i).ICAO));
                icaos{i} = messages(i).ICAO;
                if ~isempty(mapID)
                    id = obj.pTrackIDMap(mapID);
                elseif ~any(alreadySeen)
                    obj.pLastTrackID = obj.pLastTrackID + 1;
                    id = obj.pLastTrackID;
                    new(i) = true;
                else
                    id = trackID(find(alreadySeen, 1));
                end
                trackID(i) = id;
            end
            
        end
        
        function initTrackIDs = initializeMessages(obj, messages, trackids)
            % Save messages with new unique icao
            N = numel(messages);
            Navail = obj.MaxNumTracks - obj.NumTracks;
            Nadd = min(Navail, N);
            if N > Navail
                coder.internal.warning('fusion:adsb:MaxNumTracksReached', 'MaxNumTracks');
            end
            for i=1:Nadd
                obj.pICAOMap{obj.NumTracks + i} =  char(messages(i).ICAO);
                obj.pTrackIDMap(obj.NumTracks + i) = trackids(i);
                obj.pMessagesList(obj.NumTracks + i) = messages(i);
            end
            obj.NumTracks = obj.NumTracks + Nadd;
            initTrackIDs = trackids(1:Nadd);
        end
        
        function updateMessages(obj, messages)
            n = numel(messages);
            for i=1:n
                logid = strcmp(obj.pICAOMap,messages(i).ICAO);
                mess = obj.pMessagesList(logid);
                mess = fusion.internal.adsb.copyMessageData(mess(1), messages(i));
                mess.Age = mess.Age+1;
                obj.pMessagesList(logid) = mess;
            end
        end
        
        function covariance = extractCovariance(obj, nacp, nacv, gva ,lat,lon)
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
            
            epu = obj.pNACpTable(nacp+1);
            evu = obj.pNACvTable(nacv+1);
            alterror = obj.pGVATable(gva+1);
            
            Phorizon = min(epu^2 / 2, 5e8);
            Pvertical = min(alterror^2 / 2, 5e8);
            Vhorizon = min(evu^2 / 2, 1e5);
            Vvertical = 1e5; % arbitrary large number
            
            Rned2ecef = fusion.internal.frames.ned2ecefrotmat(lat,lon);
            Pxx_ned = diag([Phorizon, Phorizon, Pvertical]);
            Pvv_ned = diag([Vhorizon, Vhorizon, Vvertical]);
            Pxx_ecef = Rned2ecef * Pxx_ned * Rned2ecef';
            Pvv_ecef = Rned2ecef * Pvv_ned * Rned2ecef';
            
            R = [1 0 0 0 0 0 ; 0 0 0 1 0 0; 0 1 0 0 0 0;...
                0 0 0 0 1 0; 0 0 1 0 0 0; 0 0 0 0 0 1];
            
            covariance = R*blkdiag(Pxx_ecef, Pvv_ecef)*R';
        end
        
        function tracks = packageTracks(obj, complete)
            confMessages = obj.pMessagesList(complete);
            tracks = repmat(obj.pTrackTemplate, 1, numel(confMessages));
            [ids, time, ~, ~, nacp, nacv, gva, attributes ] = ...
                fusion.internal.adsb.parseMessage(confMessages);
            for i=1:numel(confMessages)
                track = tracks(i);
                trackid = obj.pTrackIDMap(strcmp(obj.pICAOMap, ids{i}));
                track.TrackID = trackid(1);
                track.UpdateTime = time(i);
                % state size should not change here
                track.State = fusion.internal.adsb.extractState(confMessages(i));
                track.StateCovariance = extractCovariance(obj, nacp(i), nacv(i), ...
                    gva(i), confMessages(i).Latitude, confMessages(i).Longitude);
                track.ObjectAttributes =attributes{i} ;
                track.Age = confMessages(i).Age;
                track.SourceIndex = obj.ReceiverIndex;
                tracks(i) = track;
            end
        end
        
        function confirmed = confirmTracks(obj, trackIDs)
            % Check for full state
            states = extractStates(obj);
            allconfirmed = all(~isnan(states));
            confirmed = allconfirmed(trackIDs);
        end
        
        function states = extractStates(obj)
            states = zeros(6,obj.NumTracks);
            for i=1:obj.NumTracks
                message = obj.pMessagesList(i);
                states(:,i) = fusion.internal.adsb.extractState(message);
            end
        end
        
        function messagesOut = validateMessageStruct(obj, messagesIn)
            % messages must be a struct array
            validateattributes(messagesIn,{'struct'},{'vector', 'nonempty'},...
                'adsbReceiver');
            % Each struct must at least have the following fields
            % - ICAO
            % - Time
            s = messagesIn(1);
            
            flag = isfield(s, 'ICAO') && isfield(s, 'Time');
            coder.internal.errorIf( isempty(flag) || ~flag,...
                'fusion:adsb:InvalidMessage');
            
            % Pad messages to conform to template.
            messagesOut = repmat(obj.pMessageTemplate,1, numel(messagesIn));
            for i=1:numel(messagesOut)
                messagesOut(i) = fusion.internal.adsb.copyMessageData(messagesOut(i),messagesIn(i));
            end
        end
        
        function map = packageICAOMapOutput(obj)
            icaos = obj.pICAOMap;
            tids  = obj.pTrackIDMap;
            map = repmat(struct('ICAO',obj.pICAOTemplate,'TrackID',uint32(0)),1, obj.NumTracks);
            for i=1:obj.NumTracks
                map(i).ICAO = icaos{i};
                map(i).TrackID = tids(i);
            end
        end
        
        function info = packageInfo(obj, keep)
            templateIcaoToTrackID = repmat(struct('ICAO',obj.pICAOTemplate,'TrackID',uint32(0)),1,1);
            info = struct('Discarded',zeros(1,0,'uint32'),...
                'IcaoToTrackID',templateIcaoToTrackID);
            coder.varsize('info.Discarded','info.IcaoToTrackID',[1 obj.MaxNumTracks],[0 1]);
            % MATLAB always return a 0-0 empty, force 1-0 empty for
            % consistent output with codegen
            if all(keep)
                info.Discarded = zeros(1,0,'uint32');
            else
                info.Discarded = cast(find(~keep(:)),'uint32')';
            end
            info.IcaoToTrackID = packageICAOMapOutput(obj);
        end
        
        function tracksOut = structToTracks(obj,tracks)
            % Default Object Track has proper State size
            tracksOut = repmat(objectTrack('SourceIndex',obj.ReceiverIndex), 1, numel(tracks));
            for i=1:numel(tracksOut)
                tracksOut(i).TrackID = tracks(i).TrackID;
                tracksOut(i).UpdateTime = tracks(i).UpdateTime;
                tracksOut(i).State = tracks(i).State;
                tracksOut(i).StateCovariance = tracks(i).StateCovariance;
                tracksOut(i).ObjectAttributes = tracks(i).ObjectAttributes ;
                tracksOut(i).Age = tracks(i).Age;
            end
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end
