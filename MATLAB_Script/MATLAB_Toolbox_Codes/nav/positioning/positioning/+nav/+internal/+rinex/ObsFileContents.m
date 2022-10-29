classdef (Hidden) ObsFileContents < nav.internal.rinex.FileContents
%OBSFILECONTENTS Definition of entries for RINEX obs files.
%
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        LOSS_OF_LOCK_INDICATOR = "_LLI";
        SIGNAL_STRENGTH_INDICATOR = "_SSI";

        %CommonEntries Entries names common to all obs records.
        CommonEntries = ["SatelliteID", "EpochFlag", ...
            "ReceiverClockOffset"];

        % Header line labels and spacing constants.
        % Labels and spacing from Table A5 in RINEX 3.05 standard document.

        MARKER_NAME_LABEL = "MARKER NAME";

        MARKER_NUMBER_LABEL = "MARKER NUMBER";
        

        MARKER_TYPE_LABEL = "MARKER TYPE";

        OBSERVER_AGENCY_LABEL = "OBSERVER / AGENCY";
        OBSERVER_START = 1;
        OBSERVER_END = 20;
        AGENCY_START = 21;
        AGENCY_END = 60;

        REC_INFO_LABEL = "REC # / TYPE / VERS";
        REC_INFO_NUM_START = 1;
        REC_INFO_NUM_END = 20;
        REC_INFO_TYPE_START = 21;
        REC_INFO_TYPE_END = 40;
        REC_INFO_VER_START = 41;
        REC_INFO_VER_END = 60;

        ANT_INFO_LABEL = "ANT # / TYPE";
        ANT_INFO_NUM_START = 1;
        ANT_INFO_NUM_END = 20;
        ANT_INFO_TYPE_START = 21;
        ANT_INFO_TYPE_END = 40;

        APPROX_POS_XYZ_LABEL = "APPROX POSITION XYZ";
        APPROX_POS_XYZ_START = 1;
        APPROX_POS_XYZ_END = 42;
        APPROX_POS_XYZ_FORMAT = "%14.4f";

        ANT_DELTA_HEN_LABEL = "ANTENNA: DELTA H/E/N";
        ANT_DELTA_HEN_START = 1;
        ANT_DELTA_HEN_END = 42;
        ANT_DELTA_HEN_FORMAT = "%14.4f";

        ANT_DELTA_XYZ_LABEL = "ANTENNA: DELTA X/Y/Z";
        ANT_DELTA_XYZ_START = 1;
        ANT_DELTA_XYZ_END = 42;
        ANT_DELTA_XYZ_FORMAT = "%14.4f";

        ANT_PHASE_LABEL = "ANTENNA: PHASECENTER";
        ANT_PHASE_SAT_SYS_START = 1;
        ANT_PHASE_SAT_SYS_END = 1;
        ANT_PHASE_OBS_CODE_START = 3;
        ANT_PHASE_OBS_CODE_END = 5;
        ANT_PHASE_CENTER_START = 6;
        ANT_PHASE_CENTER_END = 42;
        ANT_PHASE_CENTER_FORMAT = "%9.4f%14.4f%14.4f";

        ANT_B_SIGHT_LABEL = "ANTENNA: B.SIGHT XYZ";
        ANT_B_SIGHT_START = 1;
        ANT_B_SIGHT_END = 42;
        ANT_B_SIGHT_FORMAT = "%14.4f";

        ANT_ZERODIR_AZI_LABEL = "ANTENNA: ZERODIR AZI";
        ANT_ZERODIR_AZI_START = 1;
        ANT_ZERODIR_AZI_END = 14;
        ANT_ZERODIR_AZI_FORMAT = "%14.4f";

        ANT_ZERODIR_XYZ_LABEL = "ANTENNA: ZERODIR XYZ";
        ANT_ZERODIR_XYZ_START = 1;
        ANT_ZERODIR_XYZ_END = 42;
        ANT_ZERODIR_XYZ_FORMAT = "%14.4f";

        COM_XYZ_LABEL = "CENTER OF MASS: XYZ";
        COM_XYZ_START = 1;
        COM_XYZ_END = 42;
        COM_XYZ_FORMAT = "%14.4f";

        SIGNAL_STRENGTH_UNIT_LABEL = "SIGNAL STRENGTH UNIT";
        SIGNAL_STRENGTH_UNIT_START = 1;
        SIGNAL_STRENGTH_UNIT_END = 20;

        INTERVAL_LABEL = "INTERVAL";
        INTERVAL_START = 1;
        INTERVAL_END = 10;
        INTERVAL_FORMAT = "%10.3f";

        TIME_FIRST_OBS_LABEL = "TIME OF FIRST OBS";
        TIME_FIRST_OBS_DATE_START = 1;
        TIME_FIRST_OBS_DATE_END = 43;
        TIME_FIRST_OBS_ZONE_START = 49;
        TIME_FIRST_OBS_ZONE_END = 51;
        TIME_FIRST_OBS_DATE_FORMAT = "%6f%6f%6f%6f%6f%13.7f";

        TIME_LAST_OBS_LABEL = "TIME OF LAST OBS";
        TIME_LAST_OBS_DATE_START = 1;
        TIME_LAST_OBS_DATE_END = 43;
        TIME_LAST_OBS_ZONE_START = 49;
        TIME_LAST_OBS_ZONE_END = 51;
        TIME_LAST_OBS_DATE_FORMAT = "%6f%6f%6f%6f%6f%13.7f";

        RCV_CLOCK_OFFSET_APPLIED_LABEL = "RCV CLOCK OFFS APPL";
        RCV_CLOCK_OFFSET_APPLIED_START = 1;
        RCV_CLOCK_OFFSET_APPLIED_END = 6;

        DCBS_LABEL = "SYS / DCBS APPLIED";
        DCBS_SAT_SYS_START = 1;
        DCBS_SAT_SYS_END = 1;
        DCBS_PROGRAM_START = 3;
        DCBS_PROGRAM_END = 19;
        DCBS_SOURCE_START = 21;
        DCBS_SOURCE_END = 60;

        PCVS_LABEL = "SYS / PCVS APPLIED";
        PCVS_SAT_SYS_START = 1;
        PCVS_SAT_SYS_END = 1;
        PCVS_PROGRAM_START = 3;
        PCVS_PROGRAM_END = 19;
        PCVS_SOURCE_START = 21;
        PCVS_SOURCE_END = 60;

        SCALE_FACTOR_LABEL = "SYS / SCALE FACTOR";
        SCALE_FACTOR_SAT_SYS_START = 1;
        SCALE_FACTOR_SAT_SYS_END = 1;
        SCALE_FACTOR_FACTOR_START = 3;
        SCALE_FACTOR_FACTOR_END = 6;
        SCALE_FACTOR_NUM_OBS_START = 9;
        SCALE_FACTOR_NUM_OBS_END = 10;
        SCALE_FACTOR_OBS_START = 12;
        SCALE_FACTOR_OBS_END = 58;

        PHASE_SHIFT_LABEL = "SYS / PHASE SHIFT";
        PHASE_SHIFT_SAT_SYS_START = 1;
        PHASE_SHIFT_SAT_SYS_END = 1;
        PHASE_SHIFT_OBS_TYPE_START = 3;
        PHASE_SHIFT_OBS_TYPE_END = 5;
        PHASE_SHIFT_CORRECTION_START = 7;
        PHASE_SHIFT_CORRECTION_END = 14;
        PHASE_SHIFT_NUM_SATS_START = 17;
        PHASE_SHIFT_NUM_SATS_END = 18;
        PHASE_SHIFT_SATS_START = 20;
        PHASE_SHIFT_SATS_END = 58;
        PHASE_SHIFT_CORRECTION_FORMAT = "%8.5f";

        GLONASS_SLOT_FREQ_NUM_LABEL = "GLONASS SLOT / FRQ #";
        GLONASS_SLOT_FREQ_NUM_NUM_SATS_START = 1;
        GLONASS_SLOT_FREQ_NUM_NUM_SATS_END = 3;
        GLONASS_SLOT_FREQ_NUM_SAT_FREQ_START = 5;
        GLONASS_SLOT_FREQ_NUM_SAT_FREQ_END = 60;
        GLONASS_SLOT_FREQ_NUM_WIDTH = 8;
        GLONASS_SLOT_WIDTH = 3;
        GLONASS_FREQ_NUM_WIDTH = 2;

        GLONASS_COD_PHS_BIS_LABEL = "GLONASS COD/PHS/BIS";
        GLONASS_COD_PHS_BIS_NUM_ENTRIES = 4;
        GLONASS_COD_PHS_BIS_ENTRY_WIDTH = 13;
        GLONASS_COD_PHS_BIS_OBS_TYPE_START = 2; 
        GLONASS_COD_PHS_BIS_OBS_TYPE_END = 4;
        GLONASS_COD_PHS_BIS_BIAS_START = 6;
        GLONASS_COD_PHS_BIS_BIAS_END = 13;
        GLONASS_COD_PHS_BIS_BIAS_FORMAT = "%8.3f";

        NUM_SATS_LABEL = "# OF SATELLITES";
        NUM_SATS_START = 1;
        NUM_SATS_END = 6;

        NUM_OBS_LABEL = "PRN / # OF OBS";
        NUM_OBS_SAT_ID_START = 4;
        NUM_OBS_SAT_ID_END = 6;
        NUM_OBS_NUM_START = 7;
        NUM_OBS_NUM_END = 60;
        NUM_OBS_NUM_WIDTH = 6;
    end

    methods
        function header = parseHeaderImpl(~, header, rawheader)
            header.ObservationTypes = rawheader.ObservationTypes;
        end

        function [header, isParsed] = parseHeaderLineImpl(obj, header, label, lines)
            isParsed = true;
            switch label
                case obj.MARKER_NAME_LABEL
                    header = parseHeaderMarkerName(obj, header, lines);
                case obj.MARKER_NUMBER_LABEL
                    header = parseHeaderMarkerNumber(obj, header, lines);
                case obj.MARKER_TYPE_LABEL
                    header = parseHeaderMarkerType(obj, header, lines);
                case obj.OBSERVER_AGENCY_LABEL
                    header = parseHeaderObserverAgency(obj, header, lines);
                case obj.REC_INFO_LABEL
                    header = parseHeaderRecInfo(obj, header, lines);
                case obj.ANT_INFO_LABEL
                    header = parseHeaderAntInfo(obj, header, lines);
                case obj.APPROX_POS_XYZ_LABEL
                    header = parseHeaderApproxPosXYZ(obj, header, lines);
                case obj.ANT_DELTA_HEN_LABEL
                    header = parseHeaderAntDeltaHEN(obj, header, lines);
                case obj.ANT_DELTA_XYZ_LABEL
                    header = parseHeaderAntDeltaXYZ(obj, header, lines);
                case obj.ANT_PHASE_LABEL
                    header = parseHeaderAntPhase(obj, header, lines);
                case obj.ANT_B_SIGHT_LABEL
                    header = parseHeaderAntBSight(obj, header, lines);
                case obj.ANT_ZERODIR_AZI_LABEL
                    header = parseHeaderAntZeroDirAzi(obj, header, lines);
                case obj.ANT_ZERODIR_XYZ_LABEL
                    header = parseHeaderAntZeroDirXYZ(obj, header, lines);
                case obj.COM_XYZ_LABEL
                    header = parseHeaderCOMXYZ(obj, header, lines);
                case obj.SIGNAL_STRENGTH_UNIT_LABEL
                    header = parseHeaderSignalStrengthUnit(obj, header, lines);
                case obj.INTERVAL_LABEL
                    header = parseHeaderInterval(obj, header, lines);
                case obj.TIME_FIRST_OBS_LABEL
                    header = parseHeaderTimeFirstObs(obj, header, lines);
                case obj.TIME_LAST_OBS_LABEL
                    header = parseHeaderTimeLastObs(obj, header, lines);
                case obj.RCV_CLOCK_OFFSET_APPLIED_LABEL
                    header = parseHeaderRCVClockOffsetApplied(obj, header, lines);
                case obj.DCBS_LABEL
                    header = parseHeaderDCBS(obj, header, lines);
                case obj.PCVS_LABEL
                    header = parseHeaderPCVS(obj, header, lines);
                case obj.SCALE_FACTOR_LABEL
                    header = parseHeaderScaleFactor(obj, header, lines);
                case obj.PHASE_SHIFT_LABEL
                    header = parseHeaderPhaseShift(obj, header, lines);
                case obj.GLONASS_SLOT_FREQ_NUM_LABEL
                    header = parseHeaderGLONASSSlotFreqNum(obj, header, lines);
                case obj.GLONASS_COD_PHS_BIS_LABEL
                    header = parseHeaderGLONASSCODPHSBIS(obj, header, lines);
                case obj.NUM_SATS_LABEL
                    header = parseHeaderNumSats(obj, header, lines);
                case obj.NUM_OBS_LABEL
                    header = parseHeaderNumObs(obj, header, lines);
                otherwise
                    isParsed = false;
            end
        end

        function data = parseData(obj, rawdata, rawheader)
            obsTypes = rawheader.ObservationTypes;

            data = struct;
            numEpochVals = 6;
            epochStart = 2;
            epochIdx = (epochStart-1)+(1:numEpochVals);
            for ii = 1:numel(obsTypes)
                satSys = obsTypes(ii).SatelliteSystem;
                desc = obsTypes(ii).Descriptors;
                arrData = rawdata.(satSys).';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                descAndStatus = [desc; 
                    desc + obj.LOSS_OF_LOCK_INDICATOR; 
                    desc + obj.SIGNAL_STRENGTH_INDICATOR];
                fields = [obj.CommonEntries, descAndStatus(:).'];
                satSys = satCodeToName(satSys);
                data.(satSys) = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", fields);
                % Remove LLI fields for observation types that are NOT
                % phase (Phase types = L).
                extraIndices = (~startsWith(fields, "L") ...
                    & endsWith(fields, "_LLI"));
                data.(satSys)(:,extraIndices) = [];
            end
        end

        function header = parseHeaderMarkerName(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.MarkerName = strip(extractBetween(line, 1, obj.HEADER_LINE_LENGTH));
        end
        function header = parseHeaderMarkerNumber(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.MarkerNumber = strip(extractBetween(line, 1, obj.HEADER_LINE_LENGTH));
        end
        function header = parseHeaderMarkerType(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.MarkerType = strip(extractBetween(line, 1, obj.HEADER_LINE_LENGTH));
        end
        function header = parseHeaderObserverAgency(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.Observer = strip(extractBetween(line, obj.OBSERVER_START, obj.OBSERVER_END));
            header.Agency = strip(extractBetween(line, obj.AGENCY_START, obj.AGENCY_END));
        end
        function header = parseHeaderRecInfo(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.ReceiverNumber = strip(extractBetween(line, obj.REC_INFO_NUM_START, obj.REC_INFO_NUM_END));
            header.ReceiverType = strip(extractBetween(line, obj.REC_INFO_TYPE_START, obj.REC_INFO_TYPE_END));
            header.ReceiverVersion = strip(extractBetween(line, obj.REC_INFO_VER_START, obj.REC_INFO_VER_END));
        end
        function header = parseHeaderAntInfo(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.AntennaNumber = strip(extractBetween(line, obj.ANT_INFO_NUM_START, obj.ANT_INFO_NUM_END));
            header.AntennaType = strip(extractBetween(line, obj.ANT_INFO_TYPE_START, obj.ANT_INFO_TYPE_END));
        end
        function header = parseHeaderApproxPosXYZ(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            approxPos = textscan(extractBetween(line, obj.APPROX_POS_XYZ_START, obj.APPROX_POS_XYZ_END), obj.APPROX_POS_XYZ_FORMAT);
            header.ApproxPosition = approxPos{:}.';
        end
        function header = parseHeaderAntDeltaHEN(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            antennaDeltaHEN = textscan(extractBetween(line, obj.ANT_DELTA_HEN_START, obj.ANT_DELTA_HEN_END), obj.ANT_DELTA_HEN_FORMAT);
            header.AntennaDeltaHEN = antennaDeltaHEN{:}.';
        end
        function header = parseHeaderAntDeltaXYZ(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            antennaDeltaXYZ = textscan(extractBetween(line, obj.ANT_DELTA_XYZ_START, obj.ANT_DELTA_XYZ_END), obj.ANT_DELTA_XYZ_FORMAT);
            header.AntennaDeltaXYZ = antennaDeltaXYZ{:}.';
        end
        function header = parseHeaderAntPhase(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                phase = textscan(extractBetween(line, obj.ANT_PHASE_CENTER_START, obj.ANT_PHASE_CENTER_END), obj.ANT_PHASE_CENTER_FORMAT);
                antPhaseCenter(jj,1) = struct( ...
                    "SatelliteSystem", extractBetween(line, obj.ANT_PHASE_SAT_SYS_START, obj.ANT_PHASE_SAT_SYS_END), ...
                    "ObservationCode", extractBetween(line, obj.ANT_PHASE_OBS_CODE_START, obj.ANT_PHASE_OBS_CODE_END), ...
                    "PhaseCenter", vertcat(phase{:}).');
            end
            header.AntennaPhaseCenter = antPhaseCenter;
        end
        function header = parseHeaderAntBSight(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            antennaBSightXYZ = textscan(extractBetween(line, obj.ANT_B_SIGHT_START, obj.ANT_B_SIGHT_END), obj.ANT_B_SIGHT_FORMAT);
            header.AntennaBSightXYZ = antennaBSightXYZ{:}.';
        end
        function header = parseHeaderAntZeroDirAzi(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            antennaZeroDirAzimuth = textscan(extractBetween(line, obj.ANT_ZERODIR_AZI_START, obj.ANT_ZERODIR_AZI_END), obj.ANT_ZERODIR_AZI_FORMAT);
            header.AntennaZeroDirAzimuth = antennaZeroDirAzimuth{:}.';
        end
        function header = parseHeaderAntZeroDirXYZ(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            antennaZeroDirXYZ = textscan(extractBetween(line, obj.ANT_ZERODIR_XYZ_START, obj.ANT_ZERODIR_XYZ_END), obj.ANT_ZERODIR_XYZ_FORMAT);
            header.AntennaZeroDirXYZ = antennaZeroDirXYZ{:}.';
        end
        function header = parseHeaderCOMXYZ(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            centerOfMassXYZ = textscan(extractBetween(line, obj.COM_XYZ_START, obj.COM_XYZ_END), obj.COM_XYZ_FORMAT);
            header.CenterOfMassXYZ = centerOfMassXYZ{:}.';
        end
        function header = parseHeaderSignalStrengthUnit(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                ssu(jj) = strip(extractBetween(line, obj.SIGNAL_STRENGTH_UNIT_START, obj.SIGNAL_STRENGTH_UNIT_END));
            end
            header.SignalStrengthUnit = ssu;
        end
        function header = parseHeaderInterval(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            interval = textscan(extractBetween(line, obj.INTERVAL_START, obj.INTERVAL_END), obj.INTERVAL_FORMAT);
            header.Interval = interval{:};
        end
        function header = parseHeaderTimeFirstObs(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            dateNums = textscan(extractBetween(line, obj.TIME_FIRST_OBS_DATE_START, obj.TIME_FIRST_OBS_DATE_END), obj.TIME_FIRST_OBS_DATE_FORMAT);
            firstObsZone = extractBetween(line, obj.TIME_FIRST_OBS_ZONE_START, obj.TIME_FIRST_OBS_ZONE_END);
            fileSatSys = header.FileSatelliteSystem;
            % Convert satellite system time to UTC time.
            t = satSysTimeToUTCTime(dateNums{:}, firstObsZone, fileSatSys);
            header.FirstObsTime = t;
        end
        function header = parseHeaderTimeLastObs(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            dateNums = textscan(extractBetween(line, obj.TIME_LAST_OBS_DATE_START, obj.TIME_LAST_OBS_DATE_END), obj.TIME_LAST_OBS_DATE_FORMAT);
            lastObsZone = extractBetween(line, obj.TIME_LAST_OBS_ZONE_START, obj.TIME_LAST_OBS_ZONE_END);
            fileSatSys = header.FileSatelliteSystem;
            % Convert satellite system time to UTC time.
            t = satSysTimeToUTCTime(dateNums{:}, lastObsZone, fileSatSys);
            header.LastObsTime = t; %datetime(dateNums{:});
        end
        function header = parseHeaderRCVClockOffsetApplied(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            hasRCVClkOffset = str2double(extractBetween(line, obj.RCV_CLOCK_OFFSET_APPLIED_START, obj.RCV_CLOCK_OFFSET_APPLIED_END));
            header.HasReceiverClockOffset = (hasRCVClkOffset == 1);
        end
        function header = parseHeaderDCBS(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                dcbs(jj,1) = struct( ...
                    "SatelliteSystem", extractBetween(line, obj.DCBS_SAT_SYS_START, obj.DCBS_SAT_SYS_END), ...
                    "Program", strip(extractBetween(line, obj.DCBS_PROGRAM_START, obj.DCBS_PROGRAM_END)), ...
                    "Source", strip(extractBetween(line, obj.DCBS_SOURCE_START, obj.DCBS_SOURCE_END)));
            end
            header.DCBS = dcbs;
        end
        function header = parseHeaderPCVS(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                pcvs(jj,1) = struct( ...
                    "SatelliteSystem", extractBetween(line, obj.PCVS_SAT_SYS_START, obj.PCVS_SAT_SYS_END), ...
                    "Program", strip(extractBetween(line, obj.PCVS_PROGRAM_START, obj.PCVS_PROGRAM_END)), ...
                    "Source", strip(extractBetween(line, obj.PCVS_SOURCE_START, obj.PCVS_SOURCE_END)));
            end
            header.PCVS = pcvs;
        end
        function header = parseHeaderScaleFactor(obj, header, lines)
            % There can be multiple lines due to a continuation of
            % a line, or a different scale factor for observations.
            % Determine how many lines are a beginning line vs a
            % continuation line.
            fullLineIdx = find(~startsWith(lines, " "));
            numFullLines = numel(fullLineIdx);
            numLines = numel(lines);
            lineIdx = [fullLineIdx; numLines+1];
            for jj = numFullLines:-1:1
                idx = lineIdx(jj);
                % Scale factor definition.
                satSys = extractBetween(lines(idx), obj.SCALE_FACTOR_SAT_SYS_START, obj.SCALE_FACTOR_SAT_SYS_END);
                factor = str2double(extractBetween(lines(idx), obj.SCALE_FACTOR_FACTOR_START, obj.SCALE_FACTOR_FACTOR_END));
                numObs = str2double(extractBetween(lines(idx), obj.SCALE_FACTOR_NUM_OBS_START, obj.SCALE_FACTOR_NUM_OBS_END));
                % Observation types that scale factor applies to. 
                % May be on continuation lines.
                nextIdx = lineIdx(jj+1);
                obsTypes = extractBetween(lines(idx:(nextIdx-1)), obj.SCALE_FACTOR_OBS_START, obj.SCALE_FACTOR_OBS_END);
                obsTypes = arrayfun(@split, strip(obsTypes), "UniformOutput", false);
                obsTypes = vertcat(obsTypes{:}).';
                if (isnan(numObs) || (numObs == 0))
                    obsTypes = "ALL";
                end
                scaleFactor(jj,1) = struct( ...
                    "SatelliteSystem", satSys, ...
                    "Factor", factor, ...
                    "ObservationTypes", obsTypes);
            end
            header.ScaleFactor = scaleFactor;
        end
        function header = parseHeaderPhaseShift(obj, header, lines)
            % There can be multiple lines due to a continuation of
            % a line, or a different phase shift for satellite IDs.
            % Determine how many lines are a beginning line vs a
            % continuation line.
            fullLineIdx = find(~startsWith(lines, " "));
            numFullLines = numel(fullLineIdx);
            numLines = numel(lines);
            lineIdx = [fullLineIdx; numLines+1];
            for jj = numFullLines:-1:1
                idx = lineIdx(jj);
                % Phase shift definition.
                satSys = extractBetween(lines(idx), obj.PHASE_SHIFT_SAT_SYS_START, obj.PHASE_SHIFT_SAT_SYS_END);
                obsType = extractBetween(lines(idx), obj.PHASE_SHIFT_OBS_TYPE_START, obj.PHASE_SHIFT_OBS_TYPE_END);
                correction = textscan(extractBetween(lines(idx), obj.PHASE_SHIFT_CORRECTION_START, obj.PHASE_SHIFT_CORRECTION_END), obj.PHASE_SHIFT_CORRECTION_FORMAT);
                correction = correction{:};
                if isempty(correction)
                    correction = NaN;
                end
                numSats = str2double(extractBetween(lines(idx), obj.PHASE_SHIFT_NUM_SATS_START, obj.PHASE_SHIFT_NUM_SATS_END));
                % Satellite IDs that phase shift applies to. May be
                % on continuation lines.
                nextIdx = lineIdx(jj+1);
                satIDs = extractBetween(lines(idx:(nextIdx-1)), obj.PHASE_SHIFT_SATS_START, obj.PHASE_SHIFT_SATS_END);
                satIDs = arrayfun(@split, strip(satIDs), "UniformOutput", false);
                satIDs = vertcat(satIDs{:}).';
                if (isnan(numSats) || (numSats == 0))
                    satIDs = "ALL";
                end
                phaseShift(jj,1) = struct( ...
                    "SatelliteSystem", satSys, ...
                    "ObservationType", obsType, ...
                    "Correction", correction, ...
                    "SatelliteIDs", satIDs);
            end
            header.PhaseShift = phaseShift;
        end
        function header = parseHeaderGLONASSSlotFreqNum(obj, header, lines)
            numSats = str2double(extractBetween(lines(1), obj.GLONASS_SLOT_FREQ_NUM_NUM_SATS_START, obj.GLONASS_SLOT_FREQ_NUM_NUM_SATS_END));
            % Create a string of satellite and frequency numbers.
            rawNums = extractBetween(lines, obj.GLONASS_SLOT_FREQ_NUM_SAT_FREQ_START, obj.GLONASS_SLOT_FREQ_NUM_SAT_FREQ_END);
            rawNums = join(rawNums, "");
            % Define the starting and ending indices for satellite
            % and frequency numbers.
            satIdxStart = (0:(numSats-1))*(obj.GLONASS_SLOT_FREQ_NUM_WIDTH-1) + 1;
            satIdxEnd = satIdxStart + (obj.GLONASS_SLOT_WIDTH-1);
            freqIdxStart = satIdxStart + (obj.GLONASS_SLOT_WIDTH+1);
            freqIdxEnd = freqIdxStart + (obj.GLONASS_FREQ_NUM_WIDTH-1);
            % Extract all the satellite and frequency numbers.
            getNums = @(startIdx, stopIdx) arrayfun( ...
                @(a,b) extractBetween(rawNums, a, b), startIdx, stopIdx);
            satNums = getNums(satIdxStart, satIdxEnd);
            freqNums = str2double(getNums(freqIdxStart, freqIdxEnd));

            header.GLONASSFrequencyNumbers = struct( ...
                "Slot", satNums, "FrequencyNumber", freqNums);
        end
        function header = parseHeaderGLONASSCODPHSBIS(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            for jj = obj.GLONASS_COD_PHS_BIS_NUM_ENTRIES:-1:1
                entryOffset = (jj-1)*obj.GLONASS_COD_PHS_BIS_ENTRY_WIDTH;
                obsTypes(1,jj) = extractBetween(line, entryOffset+obj.GLONASS_COD_PHS_BIS_OBS_TYPE_START, entryOffset+obj.GLONASS_COD_PHS_BIS_OBS_TYPE_END);
                bias = textscan(extractBetween(line, entryOffset+obj.GLONASS_COD_PHS_BIS_BIAS_START, entryOffset+obj.GLONASS_COD_PHS_BIS_BIAS_END), obj.GLONASS_COD_PHS_BIS_BIAS_FORMAT);
                bias = bias{:};
                if isempty(bias)
                    bias = NaN;
                end
                biases(1,jj) = bias;
            end
            header.GLONASSCodePhaseBias = struct( ...
                "ObservationTypes", obsTypes, ...
                "Bias", biases);
        end
        function header = parseHeaderNumSats(obj, header, lines)
            % There should only be one line for this header, but
            % take the first line if there are duplicates due to a
            % malformed file.
            line = lines(1);
            header.NumSatellites = str2double(extractBetween(line, obj.NUM_SATS_START, obj.NUM_SATS_END));
        end
        function header = parseHeaderNumObs(obj, header, lines)
            fullLineIdx = find(~startsWith(lines, "    "));
            numFullLines = numel(fullLineIdx);
            numLines = numel(lines);
            lineIdx = [fullLineIdx; numLines+1];
            allObsTypes = header.ObservationTypes;
            allSatSys = string(vertcat(allObsTypes.SatelliteSystem));
            for jj = numFullLines:-1:1
                idx = lineIdx(jj);
                % Number of observations definition.
                satID = extractBetween(lines(idx), obj.NUM_OBS_SAT_ID_START, obj.NUM_OBS_SAT_ID_END);
                satSys = extract(satID, 1);
                numObs = numel(allObsTypes(allSatSys == satSys).Descriptors);
                % Observation counts that apply to satellite ID. 
                % May be on continuation lines.
                nextIdx = lineIdx(jj+1);
                obsCountsString = extractBetween(lines(idx:(nextIdx-1)), obj.NUM_OBS_NUM_START, obj.NUM_OBS_NUM_END);
                obsCountsString = join(obsCountsString, "");
                obsCounts = NaN(1, numObs);
                for kk = numObs:-1:1
                    numStart = 1 + obj.NUM_OBS_NUM_WIDTH*(kk-1);
                    numStop = obj.NUM_OBS_NUM_WIDTH*kk;
                    obsCounts(1,kk) = str2double(extractBetween(obsCountsString, numStart, numStop));
                end
                
                totalObservations(jj,1) = struct( ...
                    "SatelliteID", satID, ...
                    "NumObservations", obsCounts);
            end
            header.TotalObservations = totalObservations;
        end
    end
end

function name = satCodeToName(code)
name = 'GPS';
switch code
    case 'G'
        name = 'GPS';
    case 'E'
        name = 'Galileo';
    case 'R'
        name = 'GLONASS';
    case 'C'
        name = 'BeiDou';
    case 'J'
        name = 'QZSS';
    case 'I'
        name = 'NavIC';
    case 'S'
        name = 'SBAS';
end
end

function t = satSysTimeToUTCTime(y, M, d, h, m, s, satSysTime, fileSatSys)
%SATSYSTIMETOUTCTIME Convert date with satellite system time zone to
%   datetime with UTC time zone.
%
%   Valid satellite system times are:
%     GPS - GPS time system (GPS)
%     GAL - Galileo time system (GPS)
%     GLO - GLONASS time system (UTC)
%     BDT - BeiDou time system (BDS)
%     QZS - QZSS time system (GPS)
%     IRN - IRNSS/NavIC time system (GPS)

t = datetime(y, M, d, h, m, s, "TimeZone", "UTCLeapSeconds");

% Set the time system code to the file satellite system if the file is not
% mixed.
satSysTime = setDefaultSatSysTime(satSysTime, fileSatSys);

validSystemTimes = ["GPS", "GAL", "GLO", "BDT", "QZS", "IRN"];

if (any(matches(validSystemTimes, satSysTime)))
    tz = "UTC";
    needsTimeAdjustment = (satSysTime ~= "GLO");
    if (needsTimeAdjustment)
        if (satSysTime == "BDT")
            % Use the BeiDou start date.
            satSysStart = datetime(2006, 1, 1, 0, 0, 0, "TimeZone", "UTCLeapSeconds");
        else 
            % Use the GPS start date.
            satSysStart = datetime(1980, 1, 6, 0, 0, 0, "TimeZone", "UTCLeapSeconds");    
        end
        ls = leapseconds;
        ls.Date.TimeZone = "UTC";
        ls.Date.TimeZone = "UTCLeapSeconds";
        ignoredLeapSeconds = ls.CumulativeAdjustment(find(ls.Date < satSysStart, 1, "last"));
        dt = ls.CumulativeAdjustment(end) - ignoredLeapSeconds;

        t = t - dt;
    end
else
    % Time system code is invalid or empty. Warn and set the time zone 
    % to empty.
    warning(message("nav_positioning:rinexInternal:InvalidTimeSystemCode"));
    tz = "";
end

t.TimeZone = tz;
end

function satSysTime = setDefaultSatSysTime(satSysTime, fileSatSys)
% Only edit the value if the satSysTime string is empty.
if (strip(satSysTime) == "")
    switch fileSatSys
        case "G"
            satSysTime = "GPS";
        case "E"
            satSysTime = "GAL";
        case "R"
            satSysTime = "GLO";
        case "C"
            satSysTime = "BDT";
        case "I"
            satSysTime = "IRN";
        case "J"
            satSysTime = "QZS";
        case "S"
            satSysTime = "GPS";
    end
end
end
