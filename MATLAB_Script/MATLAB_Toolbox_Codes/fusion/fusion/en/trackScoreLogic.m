classdef trackScoreLogic< matlabshared.tracking.internal.fusion.TrackLogic
%trackScoreLogic Confirm and delete tracks based on track score
%   logic = trackScoreLogic creates a track score logic with default
%   confirmation and deletion thresholds.
%   A track will be confirmed if the current track score is greater than or
%   equal to the ConfirmationThreshold. Track score is also known as the
%   log likelihood of a track.
%   A track will be deleted if the current track score has decreased
%   relative to the maximum track score by the DeletionThreshold.
%
%   ... = trackScoreLogic('ConfirmationThreshold',a,'DeletionThreshold',b)
%   allows you to specify the confirmation or deletion threshold. Both a
%   and b must be scalars.
%   Unspecified properties will have default values.
%
%   Use init to initialize the track score with the first 'hit'. 
%   Use hit to update the track score with any subsequent 'hit'.
%   Use miss to update the track score with any subsequent 'miss'.
%
% trackScoreLogic properties:
%   ConfirmationThreshold   - parameters controlling track confirmation
%   DeletionThreshold       - parameters controlling track deletion
%   Score                   - the current track logic score (read only)
%   MaxScore                - the maximum track logic score (read only)
%
% trackScoreLogic methods:
%   init                - initialize the track logic with first 'hit'
%   hit                 - update the track logic with a subsequent 'hit'
%   miss                - update the track with a 'miss'
%   sync                - synchronize the trackScoreLogic object
%   mergeScores         - update the track score by track merging
%   checkConfirmation   - check if the track should be confirmed
%   checkDeletion       - check if the track should be deleted
%   output              - get the current track score
%   reset               - resets the object state
%   clone               - creates a copy of the object
%
% Class support:
% --------------
%   If either ConfirmationThreshold or DeletionThreshold is given on
%   construction as single precision, the other will be converted to single
%   precision and the class will use single precision.
%
% Example: 
% --------
% % Create a score-based track logic
% scoreLogic = trackScoreLogic('ConfirmationThreshold', 20, ...
%   'DeletionThreshold', -5)
% wasInitialized = false;
% pd = 0.9;   % Probability of detection
% pfa = 1e-6; % Probability of false alarm
% volume = 1; % The volume of a sensor detection bin
% beta = 0.1; % New target rate in a unit volume
%
% % Update the score-based logic 3 times. See how the score increases with
% % a 'hit' and decreases with a 'miss'. By the end of the 3rd update, the
% % confirmation flag will be true.
% for i = 1:3
%     likelihood = 0.05 + 0.05*rand(1); % likelihood of the measurement
%     detectedFlag = logical(mod(i,2)); % Only even updates are true
%     if detectedFlag && ~wasInitialized
%         init(scoreLogic, volume, beta);
%         wasInitialized = true;
%     elseif detectedFlag && wasInitialized
%         hit(scoreLogic, volume, likelihood);
%     else
%         miss(scoreLogic);
%     end
%     confFlag = checkConfirmation(scoreLogic);
%     disp(['Score and MaxScore: ', num2str(output(scoreLogic)), ...
%       '.  Confirmation Flag is: ',num2str(confFlag)])
% end
%
% % Update the logic with 3 'miss' updates. The deletion flag will turn
% % true by the end of the 3rd miss.
% for i = 1:3
%     miss(scoreLogic);
%     deleteFlag = checkDeletion(scoreLogic);
%     disp(['Score and MaxScore: ', num2str(output(scoreLogic)), ...
%       '.  Deletion Flag is: ',num2str(deleteFlag)])
% end
%
% See also: trackerGNN, trackerTOMHT, trackHistoryLogic

 
%   Copyright 2017-2019 The MathWorks, Inc.

    methods
        function out=trackScoreLogic
            % Constructor to the trackScoreLogic object
        end

        function out=checkConfirmation(~) %#ok<STOUT>
            %checkConfirmation Check if the track should be confirmed
            % tf = checkConfirmation(obj) returns true if the track should
            % be confirmed based on the track score.
        end

        function out=checkDeletion(~) %#ok<STOUT>
            %checkDeletion Check if the track should be deleted
            % tf = checkDeletion(obj) returns true if the track should be
            % deleted based on the track score.
        end

        function out=clone(~) %#ok<STOUT>
            %CLONE Create a copy of the current track logic object
            % newObj = clone(obj) returns a copy of obj.
        end

        function out=hit(~) %#ok<STOUT>
            %HIT Update the track score logic with a hit
            % HIT(obj, volume, lhood) updates the track score in a
            % case of a 'hit' using the default values: pd=0.9 and
            % pfa=1e-6. volume is the volume of the sensor detection bin.
            % For example, a 2-D radar will have a sensor bin volume of:
            %   (azimuth resolution) * (range) * (range resolution).
            % lhood is the likelihood of the detection being assigned
            % to the track. See the likelihood method of the tracking
            % filter.
            %
            % HIT(obj, volume, lhood, pd, pfa) allows you to specify
            % non-default values of pd and pfa.
        end

        function out=init(~) %#ok<STOUT>
            %INIT Initialize a track score logic
            % INIT(obj, volume, beta) initializes the score on the first
            % 'hit' using the default values: pd=0.9, pfa=1e-6. volume is
            % the volume of the sensor detection bin. For example, a 2-D
            % radar will have a sensor bin volume of:
            %   (azimuth res. in radians) * (range) * (range res.).
            % beta is the rate of new targets in a unit volume.
            % 
            % INIT(obj, volume, beta, pd, pfa) allows you to specify
            % non-default values of pd and pfa.
        end

        function out=loadobj(~) %#ok<STOUT>
            % Assign non-dependent public properties
        end

        function out=mergeScores(~) %#ok<STOUT>
            % mergeScores(thisLogic,thatLogic) updates the score of
            % thisLogic by merging the score of thatLogic. Both thisLogic
            % and thatLogic must be trackScoreLogic objects.
            %
            % Score merging increases the score of thisLogic by:
            %   log(1+exp(-(thisLogic-thatLogic)))
        end

        function out=miss(~) %#ok<STOUT>
            %Miss Update the track score logic with a miss
            % MISS(obj) updates the track score in a case of a 'miss' using
            % the default values: pd=0.9 and pfa=1e-6.
            %
            % MISS(obj, pd, pfa) allows you to specify non-default values
            % of pd and pfa.
        end

        function out=output(~) %#ok<STOUT>
            %OUTPUT  Return the current and maximum score of the object
            % [currentScore, maxScore] = OUTPUT(obj) returns the current
            % score, currentScore, and the maximum score, maxScore.
        end

        function out=reset(~) %#ok<STOUT>
            %RESET Reset the object state
            % RESET(obj) resets the object state
        end

        function out=sync(~) %#ok<STOUT>
            %SYNC Synchronize the probabilities of two objects
            % SYNC(this,that) synchronizes the probability of this object
            % to be the same as the probability of that object
        end

    end
    properties
        % ConfirmationThreshold - The minimum score for track confirmation
        %   Define the threshold for track confirmation as a real,
        %   positive, scalar. If the logic score is above this threshold,
        %   the track should be confirmed.
        %
        % Default: 20
        ConfirmationThreshold;

        % DeletionThreshold - The decrease in score for track deletion
        %   Define the threshold for track deletion as a real, negative,
        %   scalar. If the (current score - the maximum score) is more
        %   negative than this threshold, the track should be deleted.
        %
        % Default: -5
        DeletionThreshold;

        %MaxScore -  The maximum track score
        MaxScore;

        %Score -  The current track score
        Score;

    end
end
