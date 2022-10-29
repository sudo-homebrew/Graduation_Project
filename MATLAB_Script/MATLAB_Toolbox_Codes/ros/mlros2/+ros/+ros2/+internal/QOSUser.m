classdef (Abstract) QOSUser < handle
%This class is for internal use only. It may be removed in the future.

%QOSUser Base class for all ROS 2 entities that use QOS settings
%   Several ROS entities need to set and get quality of service settings
%   that determine the communication behavior over the ROS 2 network. This
%   class managers several related properties and utility methods common to
%   QOS users.
%
%   Sub-classes must re-implement the abstract "getServerInfo" method.

%   Copyright 2019 The MathWorks, Inc.

    properties (Dependent, SetAccess = protected)
        %History - The message queue mode
        History

        %Depth - The message queue size
        Depth

        %Reliability - The delivery guarantee of messages
        Reliability

        %Durability - The persistence of messages
        Durability
    end

    properties (Constant, Access = protected)
        %HistoryValues - Possible values for History property
        HistoryValues = {'keeplast', 'keepall'}

        %ReliabilityValues - Possible values for Reliability property
        ReliabilityValues = {'reliable', 'besteffort'}

        %DurabilityValues - Possible values for Durability property
        DurabilityValues = {'transientlocal', 'volatile'}
    end

    % All dependent properties are read from the server
    methods
        function history = get.History(obj)
        %get.History Custom getter for History property

        % Allow errors to be thrown from getServerInfo
            info = getServerInfo(obj);
            history = obj.HistoryValues{info.qoshistory};
        end

        function depth = get.Depth(obj)
        %get.Depth Custom getter for Depth property

        % Allow errors to be thrown from getServerInfo
            info = getServerInfo(obj);
            depth = double(info.qosdepth);
        end

        function reliability = get.Reliability(obj)
        %get.Reliability Custom getter for Reliability property

        % Allow errors to be thrown from getServerInfo
            info = getServerInfo(obj);
            reliability = obj.ReliabilityValues{info.qosreliability};
        end

        function durability = get.Durability(obj)
        %get.Durability Custom getter for Durability property

        % Allow errors to be thrown from getServerInfo
            info = getServerInfo(obj);
            durability = obj.DurabilityValues{info.qosdurability};
        end
    end

    methods (Access = protected)
        function parser = addQOSToParser(obj, parser, className)
        %addQOSToParser Add QOS names and defaults to input parse
        % QOS settings empty by default to use ROS 2 defaults.
        % className is the name to be shown in error messages if the
        % arguments parsed are invalid.

            addParameter(parser, 'History', '', ...
                         @(x) validateStringParameter(x, ...
                                                      obj.HistoryValues, ...
                                                      className, ...
                                                      'History'))
            addParameter(parser, 'Depth', [], ...
                         @(x) validateattributes(x, ...
                                                 {'numeric'}, ...
                                                 {'scalar', 'nonnegative', 'finite'}, ...
                                                 className, ...
                                                 'Depth'))
            addParameter(parser, 'Reliability', '', ...
                         @(x) validateStringParameter(x, ...
                                                      obj.ReliabilityValues, ...
                                                      className, ...
                                                      'Reliability'))
            addParameter(parser, 'Durability', '', ...
                         @(x) validateStringParameter(x, ...
                                                      obj.DurabilityValues, ...
                                                      className, ...
                                                      'Durability'))

            function validateStringParameter(value, options, className, name)
            % Separate function to suppress output and just validate
                validatestring(value, options, className, name);
            end
        end

        function qosSettings = getQosSettings(obj, qosInputs)
        %getQosSettings Handle input of possible QOS values
        %   Return a struct only containing explicitly set values, set as
        %   integers corresponding to the ROS 2 middleware enumerations

        % Non-existent fields in the QOS structure will result in
        % the default QOS setting values being used
        % validatestring has already guaranteed unique match with allowed
        % values, so now just needs index
            qosSettings = struct;
            if ~isempty(qosInputs.History)
                historyVal = char(qosInputs.History);
                historyIdx = find(strncmpi(historyVal, ...
                                           obj.HistoryValues, ...
                                           numel(historyVal)));
                qosSettings.history = int32(historyIdx);
            end
            if ~isempty(qosInputs.Depth)
                qosSettings.depth = uint64(qosInputs.Depth);
            end
            if ~isempty(qosInputs.Reliability)
                reliabilityVal = char(qosInputs.Reliability);
                reliabilityIdx = find(strncmpi(reliabilityVal, ...
                                               obj.ReliabilityValues, ...
                                               numel(reliabilityVal)));
                qosSettings.reliability = int32(reliabilityIdx);
            end
            if ~isempty(qosInputs.Durability)
                durabilityVal = char(qosInputs.Durability);
                durabilityIdx = find(strncmpi(durabilityVal, ...
                                              obj.DurabilityValues, ...
                                              numel(durabilityVal)));
                qosSettings.durability = int32(durabilityIdx);
            end
        end
    end

    methods (Abstract, Access = protected)
        %getServerInfo Retrieve object properties from the node server
        %   The output, INFO, must be a struct containing properties
        %   "History", "Depth", "Reliability", and "Durability", with the
        %   corresponding value provided by the user. Validity of the
        %   values provided by the user ensured by addQOSToParser, if used
        %   with an inputParser.
        info = getServerInfo(obj)
    end

end
