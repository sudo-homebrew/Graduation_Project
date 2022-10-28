classdef (Hidden) FileContents
%FILECONTENTS Definition of contents for RINEX files.
%
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        FILE_VER_FIELD = "FileVersion";
        FILE_SAT_SYS_FIELD = "FileSatelliteSystem";
        FILE_TYPE_FIELD = "FileType";

        HEADER_LINE_LENGTH = 60;

        % Header line labels and spacing constants.
        % Labels and spacing from Table A5 in RINEX 3.05 standard document.

        COMMENT_LABEL = "COMMENT";

        PGM_RUN_BY_DATE_LABEL = "PGM / RUN BY / DATE";
        PGM_START = 1;
        PGM_END = 20;
        RUN_BY_START = 21;
        RUN_BY_END = 40;
        CREATION_DATE_START = 41;
        CREATION_DATE_END = 60;
        CREATION_DATE_FORMAT = "yyyyMMdd HHmmss ";
        LOCAL_TIME_ZONE = "LCL";
        UTC_TIME_ZONE = "UTC";
        TIME_ZONE_WIDTH = 4;

        LEAP_SECONDS_LABEL = "LEAP SECONDS";
        LEAP_SECONDS_START = 1;
        LEAP_SECONDS_END = 6;
        DELTA_TIME_LEAP_SECONDS_START = 7;
        DELTA_TIME_LEAP_SECONDS_END = 12;
        WEEK_NUMBER_START = 13;
        WEEK_NUMBER_END = 18;
        DAY_NUMBER_START = 19;
        DAY_NUMBER_END = 24;
        TIME_SYSTEM_ID_START = 25;
        TIME_SYSTEM_ID_END = 27;
    end

    methods (Sealed)
        function header = parseHeader(obj, rawheader)
            header = struct(obj.FILE_VER_FIELD, rawheader.FileVersion, ...
                obj.FILE_SAT_SYS_FIELD, rawheader.FileSatelliteSystem, ...
                obj.FILE_TYPE_FIELD, rawheader.FileType);
            % Add any file-specific fields to output struct.
            header = parseHeaderImpl(obj, header, rawheader);

            headerLines = rawheader.HeaderLines;
            if ~isempty(headerLines)
                labels = strip(extractAfter(headerLines, obj.HEADER_LINE_LENGTH));
                uniqueLabels = unique(labels);
            else
                % No header lines in the file, set unique labels to empty.
                uniqueLabels = string.empty;
            end
            for ii = 1:numel(uniqueLabels)
                label = uniqueLabels(ii);
                lines = headerLines(label == labels);
                switch label
                    case obj.PGM_RUN_BY_DATE_LABEL
                        header = parseHeaderPGMRunByDate(obj, header, lines);
                    case obj.COMMENT_LABEL
                        header = parseHeaderComment(obj, header, lines);
                    case obj.LEAP_SECONDS_LABEL
                        header = parseHeaderLeapSeconds(obj, header, lines);
                    otherwise
                        % Parse file-specific header lines.
                        [header, isParsed] = parseHeaderLineImpl(obj, header, label, lines);
                        if ~isParsed
                            for jj = 1:numel(lines)
                                warning(message("nav_positioning:rinexInternal:UnsupportedHeaderLine", lines(jj)));
                            end
                        end
                end
            end
        end
    end

    methods
        function header = parseHeaderImpl(obj, header, rawheader) %#ok<INUSL,INUSD> 
            % By default, do nothing.
        end

        function header = parseHeaderPGMRunByDate(obj, header, lines)
            % There should only be one line for this header,
            % but take the first line if there are duplicates
            % due to a malformed file.
            line = lines(1);
            pgm = strip(extractBetween(line, obj.PGM_START, obj.PGM_END));
            header.PGM = pgm;
            runBy = strip(extractBetween(line, obj.RUN_BY_START, obj.RUN_BY_END));
            header.RunBy = runBy;
            % Extract creation date. Time zone can either be
            % UTC or LCL (local). Since LCL is not a valid
            % input time zone code to datetime, it is removed
            % before constructing the datetime.
            dateString = extractBetween(line, obj.CREATION_DATE_START, obj.CREATION_DATE_END);
            dateFormat = obj.CREATION_DATE_FORMAT;
            if contains(dateString, obj.LOCAL_TIME_ZONE) ...
                    || endsWith(dateString, blanks(obj.TIME_ZONE_WIDTH))
                % Remove the local time zone code from the
                % input date string. Set the time zone to
                % empty/unspecified.
                dateString = replace(dateString, obj.LOCAL_TIME_ZONE, blanks(strlength(obj.LOCAL_TIME_ZONE)));
                timeZone = "";
            else % contains(dateString, obj.UTC_TIME_ZONE)
                % Add the abbreviated time zone offset to the
                % input format string. Set the time zone to
                % UTC.
                dateFormat = dateFormat + "z";
                timeZone = obj.UTC_TIME_ZONE;
            end
            creationDate = datetime(dateString, ...
                "InputFormat", dateFormat, ...
                "TimeZone", timeZone);
            header.CreationDate = creationDate;
        end
        function header = parseHeaderComment(obj, header, lines)
            header.Comments = extractBetween(lines, 1, obj.HEADER_LINE_LENGTH);
        end
        function header = parseHeaderLeapSeconds(obj, header, lines)
            % There should only be one line for this header,
            % but take the first line if there are duplicates
            % due to a malformed file.
            line = lines(1);
            header.LeapSecondParameters = struct( ...
                "LeapSeconds", str2double(extractBetween(line, obj.LEAP_SECONDS_START, obj.LEAP_SECONDS_END)), ...
                "DeltaTimeLeapSeconds", str2double(extractBetween(line, obj.DELTA_TIME_LEAP_SECONDS_START, obj.DELTA_TIME_LEAP_SECONDS_END)), ...
                "WeekNumber", str2double(extractBetween(line, obj.WEEK_NUMBER_START, obj.WEEK_NUMBER_END)), ...
                "DayNumber", str2double(extractBetween(line, obj.DAY_NUMBER_START, obj.DAY_NUMBER_END)), ...
                "TimeSystemID", extractBetween(line, obj.TIME_SYSTEM_ID_START, obj.TIME_SYSTEM_ID_END));
        end
    end

    methods (Abstract)
        [header, isParsed] = parseHeaderLineImpl(obj, header, label, lines);
        data = parseData(obj, rawdata, rawheader);
    end
end
