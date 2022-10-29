classdef (Hidden) NavFileContents < nav.internal.rinex.FileContents
%NAVFILECONTENTS Definition of entries for RINEX nav files.
%
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %GPS Navigation Message
        %   This is a timetable with the following variables:
        %     Time - GPS time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s)
        %     SVClockDrift - SV clock drift (s/s)
        %     SVClockDriftRate - SV clock drift rate (s/s^2)
        %     IODE - Issue of Data, Ephemeris
        %     Crs - Amplitude of the sine harmonic correction term to the orbit radius (m)
        %     Delta_n - Mean motion difference from computed value at reference time (rad/s)
        %     M0 - Mean anomaly at reference time (rad)
        %     Cuc - Amplitude of the cosine harmonic correction term to the argument of latitude (rad)
        %     Eccentricity - Eccentricity
        %     Cus - Amplitude of the sine harmonic correction term to the argument of latitude (rad)
        %     sqrtA - Square root of semi-major axis (sqrt(m))
        %     Toe - Time of Ephemeris (sec of GPS week)
        %     Cic - Amplitude of the cosine harmonic correction term to the angle of inclination (rad)
        %     OMEGA0 - Longitude of ascending node of orbit plane at weekly epoch (rad)
        %     Cis - Amplitude of the sine harmonic correction term to the angle of inclination (rad)
        %     i0 - Inclination angle at reference time (rad)
        %     Crc - Amplitude of cosine harmonic correction term to the orbit radius (m)
        %     omega - Argument of perigee (rad)
        %     OMEGA_DOT - Reference rate of right ascension (rad/s)
        %     IDOT - Rate of inclination angle (rad/s)
        %     L2ChannelCodes - Codes on L2 channel
        %     GPSWeek - GPS week number, with time of ephemeris, Toe, continuous number, not mod(1024)
        %     L2PDataFlag - L2 P data flag
        %     SVAccuracy - SV accuracy (m) (See GPS ICD Section 20.3.3.3.1.3)
        %     SVHealth - SV health
        %     TGD - Timing group delay (s)
        %     IODC - Issue of Data, Clock
        %     TransmissionTime - Transmission time of message
        %     FitInterval - Fit interval (hours)
        %     BRDCOrbit7Spare3 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare4 - Spare entry, may be used in a future version
        GPSRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVClockDrift";
            "SVClockDriftRate";
            "IODE";
            "Crs";
            "Delta_n";
            "M0";
            "Cuc";
            "Eccentricity";
            "Cus";
            "sqrtA";
            "Toe";
            "Cic";
            "OMEGA0";
            "Cis";
            "i0";
            "Crc";
            "omega";
            "OMEGA_DOT";
            "IDOT";
            "L2ChannelCodes";
            "GPSWeek";
            "L2PDataFlag";
            "SVAccuracy";
            "SVHealth";
            "TGD";
            "IODC";
            "TransmissionTime";
            "FitInterval";
            "BRDCOrbit7Spare3";
            "BRDCOrbit7Spare4"];
        
        %Galileo Navigation Message
        %   This is a timetable with the following variables:
        %     Time - Galileo (GAL) time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s), af0
        %     SVClockDrift - SV clock drift (s/s), af1
        %     SVClockDriftRate - SV clock drift rate (s/s^2), af2
        %     IODnav - Issue of Data of the nav batch
        %     Crs - Amplitude of the sine harmonic correction term to the orbit radius (m)
        %     Delta_n - Mean motion difference from computed value at reference time (rad/s)
        %     M0 - Mean anomaly at reference time (rad)
        %     Cuc - Amplitude of the cosine harmonic correction term to the argument of latitude (rad)
        %     Eccentricity - Eccentricity
        %     Cus - Amplitude of the sine harmonic correction term to the argument of latitude (rad)
        %     sqrtA - Square root of semi-major axis (sqrt(m))
        %     Toe - Time of Ephemeris (sec of GPS week)
        %     Cic - Amplitude of the cosine harmonic correction term to the angle of inclination (rad)
        %     OMEGA0 - Longitude of ascending node of orbit plane at weekly epoch (rad)
        %     Cis - Amplitude of the sine harmonic correction term to the angle of inclination (rad)
        %     i0 - Inclination angle at reference time (rad)
        %     Crc - Amplitude of cosine harmonic correction term to the orbit radius (m)
        %     omega - Argument of perigee (rad)
        %     OMEGA_DOT - Reference rate of right ascension (rad/s)
        %     IDOT - Rate of inclination angle (rad/s)
        %     DataSources - Data sources
        %     GALWeek - Galileo week number, with time of ephemeris, Toe, continuous number, not mod(1024)
        %     BRDCOrbit5Spare4 - Spare entry, may be used in a future version
        %     SISAccuracy - Signal in space accuracy (m)
        %     SVHealth - SV health
        %     BGDE5aE1 - BGD E5a/E1 (s)
        %     BGDE5bE1 - BGD E5b/E1 (s)
        %     TransmissionTime - Transmission time of message
        %     BRDCOrbit7Spare2 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare3 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare4 - Spare entry, may be used in a future version
        GalileoRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVClockDrift";
            "SVClockDriftRate";
            "IODnav";
            "Crs";
            "Delta_n";
            "M0";
            "Cuc";
            "Eccentricity";
            "Cus";
            "sqrtA";
            "Toe";
            "Cic";
            "OMEGA0";
            "Cis";
            "i0";
            "Crc";
            "omega";
            "OMEGA_DOT";
            "IDOT";
            "DataSources";
            "GALWeek";
            "BRDCOrbit5Spare4";
            "SISAccuracy";
            "SVHealth";
            "BGDE5aE1";
            "BGDE5bE1";
            "TransmissionTime";
            "BRDCOrbit7Spare2";
            "BRDCOrbit7Spare3";
            "BRDCOrbit7Spare4"];
        
        %GLONASS Navigation Message
        %   This is a timetable with the following variables:
        %     Time - UTC time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s), -TauN
        %     SVFrequencyBias - SV relative frequency bias, +GammaN
        %     MessageFrameTime - Message frame time in seconds of UTC week
        %     PositionX - Satellite position X (km)
        %     VelocityX - Satellite velocity X (km/s)
        %     AccelerationX - Satellite acceleration X (km/s^2)
        %     Health - Satellite health
        %     PositionY - Satellite position Y (km)
        %     VelocityY - Satellite velocity Y (km/s)
        %     AccelerationY - Satellite acceleration Y (km/s^2)
        %     FrequencyNumber - Frequency number
        %     PositionZ - Satellite position Z (km)
        %     VelocityZ - Satellite velocity Z (km/s)
        %     AccelerationZ - Satellite acceleration Z (km/s^2)
        %     AgeOperationInfo - Age of operation information (days)
        %     StatusFlags - Status flags
        %     GroupDelay - L1/L2 group delay difference (s)
        %     URAI - Raw accuracy index
        %     HealthFlags - Health flags
        GLONASSRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVFrequencyBias";
            "MessageFrameTime";
            "PositionX";
            "VelocityX";
            "AccelerationX";
            "Health";
            "PositionY";
            "VelocityY";
            "AccelerationY";
            "FrequencyNumber";
            "PositionZ";
            "VelocityZ";
            "AccelerationZ";
            "AgeOperationInfo";
            "StatusFlags";
            "GroupDelay";
            "URAI";
            "HealthFlags"];

        %BeiDou Navigation Message
        %   This is a timetable with the following variables:
        %     Time - BeiDou (BDT) time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s)
        %     SVClockDrift - SV clock drift (s/s)
        %     SVClockDriftRate - SV clock drift rate (s/s^2)
        %     AODE - Age of Data, Ephemeris
        %     Crs - Amplitude of the sine harmonic correction term to the orbit radius (m)
        %     Delta_n - Mean motion difference from computed value at reference time (rad/s)
        %     M0 - Mean anomaly at reference time (rad)
        %     Cuc - Amplitude of the cosine harmonic correction term to the argument of latitude (rad)
        %     Eccentricity - Eccentricity
        %     Cus - Amplitude of the sine harmonic correction term to the argument of latitude (rad)
        %     sqrtA - Square root of semi-major axis (sqrt(m))
        %     Toe - Time of Ephemeris (sec of GPS week)
        %     Cic - Amplitude of the cosine harmonic correction term to the angle of inclination (rad)
        %     OMEGA0 - Longitude of ascending node of orbit plane at weekly epoch (rad)
        %     Cis - Amplitude of the sine harmonic correction term to the angle of inclination (rad)
        %     i0 - Inclination angle at reference time (rad)
        %     Crc - Amplitude of cosine harmonic correction term to the orbit radius (m)
        %     omega - Argument of perigee (rad)
        %     OMEGA_DOT - Reference rate of right ascension (rad/s)
        %     IDOT - Rate of inclination angle (rad/s)
        %     BRDCOrbit5Spare2 - Spare entry, may be used in a future version
        %     BDTWeek - Galileo week number, with time of ephemeris, Toe, continuous number, not mod(1024)
        %     BRDCOrbit5Spare4 - Spare entry, may be used in a future version
        %     SVAccuracy - SV accuracy (m)
        %     SatH1 - SatH1
        %     TGD1 - TGD1 B1/B3 (s)
        %     TGD2 - TGD2 B2/B3 (s)
        %     TransmissionTime - Transmission time of message
        %     AODC - Age of Data, Clock
        %     BRDCOrbit7Spare3 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare4 - Spare entry, may be used in a future version
        BeiDouRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVClockDrift";
            "SVClockDriftRate";
            "AODE";
            "Crs";
            "Delta_n";
            "M0";
            "Cuc";
            "Eccentricity";
            "Cus";
            "sqrtA";
            "Toe";
            "Cic";
            "OMEGA0";
            "Cis";
            "i0";
            "Crc";
            "omega";
            "OMEGA_DOT";
            "IDOT";
            "BRDCOrbit5Spare2";
            "BDTWeek";
            "BRDCOrbit5Spare4";
            "SVAccuracy";
            "SatH1";
            "TGD1";
            "TGD2";
            "TransmissionTime";
            "AODC";
            "BRDCOrbit7Spare3";
            "BRDCOrbit7Spare4"];

        %NavIC/IRNSS Navigation Message
        %   This is a timetable with the following variables:
        %     Time - NavIC/IRNSS time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s)
        %     SVClockDrift - SV clock drift (s/s)
        %     SVClockDriftRate - SV clock drift rate (s/s^2)
        %     IODEC - Issue of Data, Ephemeris and Clock
        %     Crs - Amplitude of the sine harmonic correction term to the orbit radius (m)
        %     Delta_n - Mean motion difference from computed value at reference time (rad/s)
        %     M0 - Mean anomaly at reference time (rad)
        %     Cuc - Amplitude of the cosine harmonic correction term to the argument of latitude (rad)
        %     Eccentricity - Eccentricity
        %     Cus - Amplitude of the sine harmonic correction term to the argument of latitude (rad)
        %     sqrtA - Square root of semi-major axis (sqrt(m))
        %     Toe - Time of Ephemeris (sec of GPS week)
        %     Cic - Amplitude of the cosine harmonic correction term to the angle of inclination (rad)
        %     OMEGA0 - Longitude of ascending node of orbit plane at weekly epoch (rad)
        %     Cis - Amplitude of the sine harmonic correction term to the angle of inclination (rad)
        %     i0 - Inclination angle at reference time (rad)
        %     Crc - Amplitude of cosine harmonic correction term to the orbit radius (m)
        %     omega - Argument of perigee (rad)
        %     OMEGA_DOT - Reference rate of right ascension (rad/s)
        %     IDOT - Rate of inclination angle (rad/s)
        %     BRDCOrbit5Spare2 - Spare entry, may be used in a future version
        %     IRNWeek - IRN week number, with time of ephemeris, Toe, continuous number, not mod(1024)
        %     BRDCOrbit5Spare4 - Spare entry, may be used in a future version
        %     UserRangeAccuracy - User range accuracy (m)
        %     HealthFlags - Health flags
        %     TGD - Timing group delay (s)
        %     BRDCOrbit6Spare4 - Spare entry, may be used in a future version
        %     TransmissionTime - Transmission time of message
        %     BRDCOrbit7Spare2 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare3 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare4 - Spare entry, may be used in a future version
        NavICRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVClockDrift";
            "SVClockDriftRate";
            "IODEC";
            "Crs";
            "Delta_n";
            "M0";
            "Cuc";
            "Eccentricity";
            "Cus";
            "sqrtA";
            "Toe";
            "Cic";
            "OMEGA0";
            "Cis";
            "i0";
            "Crc";
            "omega";
            "OMEGA_DOT";
            "IDOT";
            "BRDCOrbit5Spare2";
            "IRNWeek";
            "BRDCOrbit5Spare4";
            "UserRangeAccuracy";
            "HealthFlags";
            "TGD";
            "BRDCOrbit6Spare4";
            "TransmissionTime";
            "BRDCOrbit7Spare2";
            "BRDCOrbit7Spare3";
            "BRDCOrbit7Spare4"];

        %QZSS Navigation Message
        %   This is a timetable with the following variables:
        %     Time - QZSS time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s)
        %     SVClockDrift - SV clock drift (s/s)
        %     SVClockDriftRate - SV clock drift rate (s/s^2)
        %     IODE - Issue of Data, Ephemeris
        %     Crs - Amplitude of the sine harmonic correction term to the orbit radius (m)
        %     Delta_n - Mean motion difference from computed value at reference time (rad/s)
        %     M0 - Mean anomaly at reference time (rad)
        %     Cuc - Amplitude of the cosine harmonic correction term to the argument of latitude (rad)
        %     Eccentricity - Eccentricity
        %     Cus - Amplitude of the sine harmonic correction term to the argument of latitude (rad)
        %     sqrtA - Square root of semi-major axis (sqrt(m))
        %     Toe - Time of Ephemeris (sec of GPS week)
        %     Cic - Amplitude of the cosine harmonic correction term to the angle of inclination (rad)
        %     OMEGA0 - Longitude of ascending node of orbit plane at weekly epoch (rad)
        %     Cis - Amplitude of the sine harmonic correction term to the angle of inclination (rad)
        %     i0 - Inclination angle at reference time (rad)
        %     Crc - Amplitude of cosine harmonic correction term to the orbit radius (m)
        %     omega - Argument of perigee (rad)
        %     OMEGA_DOT - Reference rate of right ascension (rad/s)
        %     IDOT - Rate of inclination angle (rad/s)
        %     L2ChannelCodes - Codes on L2 channel
        %     GPSWeek - GPS week number, with time of ephemeris, Toe, continuous number, not mod(1024)
        %     L2PDataFlag - L2 P data flag
        %     SVAccuracy - SV accuracy (m) (See GPS ICD Section 20.3.3.3.1.3)
        %     SVHealth - SV health
        %     TGD - Timing group delay (s)
        %     IODC - Issue of Data, Clock
        %     TransmissionTime - Transmission time of message
        %     FitIntervalFlag - Fit interval flag
        %     BRDCOrbit7Spare3 - Spare entry, may be used in a future version
        %     BRDCOrbit7Spare4 - Spare entry, may be used in a future version
        QZSSRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVClockDrift";
            "SVClockDriftRate";
            "IODE";
            "Crs";
            "Delta_n";
            "M0";
            "Cuc";
            "Eccentricity";
            "Cus";
            "sqrtA";
            "Toe";
            "Cic";
            "OMEGA0";
            "Cis";
            "i0";
            "Crc";
            "omega";
            "OMEGA_DOT";
            "IDOT";
            "L2ChannelCodes";
            "GPSWeek";
            "L2PDataFlag";
            "SVAccuracy";
            "SVHealth";
            "TGD";
            "IODC";
            "TransmissionTime";
            "FitIntervalFlag";
            "BRDCOrbit7Spare3";
            "BRDCOrbit7Spare4"];

        %SBAS Navigation Message
        %   This is a timetable with the following variables:
        %     Time - UTC time
        %     SatelliteID - Satellite number
        %     SVClockBias - SV clock bias (s), -TauN
        %     SVFrequencyBias - SV relative frequency bias, +GammaN
        %     TransmissionTime - Transmission time of message
        %     PositionX - Satellite position X (km)
        %     VelocityX - Satellite velocity X (km/s)
        %     AccelerationX - Satellite acceleration X (km/s^2)
        %     Health - Satellite health
        %     PositionY - Satellite position Y (km)
        %     VelocityY - Satellite velocity Y (km/s)
        %     AccelerationY - Satellite acceleration Y (km/s^2)
        %     AccuracyCode - Accuracy code (URA, m)
        %     PositionZ - Satellite position Z (km)
        %     VelocityZ - Satellite velocity Z (km/s)
        %     AccelerationZ - Satellite acceleration Z (km/s^2)
        %     IODN - Issue of Data, Navigation
        SBASRecordEntries = ["SatelliteID";
            "SVClockBias";
            "SVFrequencyBias";
            "TransmissionTime";
            "PositionX";
            "VelocityX";
            "AccelerationX";
            "Health";
            "PositionY";
            "VelocityY";
            "AccelerationY";
            "AccuracyCode";
            "PositionZ";
            "VelocityZ";
            "AccelerationZ";
            "IODN"];

        % Header line labels and spacing constants.
        % Labels and spacing from Table A5 in RINEX 3.05 standard document.
        
        IONO_CORR_LABEL = "IONOSPHERIC CORR";
        IONO_CORR_CORRECTION_TYPE_START = 1;
        IONO_CORR_CORRECTION_TYPE_END = 4;
        IONO_CORR_PARAMETERS_START = 6;
        IONO_CORR_PARAMETERS_END = 53;
        IONO_CORR_TIME_MARK_START = 55;
        IONO_CORR_TIME_MARK_END = 55;
        IONO_CORR_SVID_START = 57;
        IONO_CORR_SVID_END = 58;
        IONO_CORR_PARAMETERS_FORMAT = "%12.4f";

        TIME_SYS_CORR_LABEL = "TIME SYSTEM CORR";
        TIME_SYS_CORR_CORRECTION_TYPE_START = 1;
        TIME_SYS_CORR_CORRECTION_TYPE_END = 4;
        TIME_SYS_CORR_PARAMETER_1_START = 6;
        TIME_SYS_CORR_PARAMETER_1_END = 22;
        TIME_SYS_CORR_PARAMETER_2_START = 23;
        TIME_SYS_CORR_PARAMETER_2_END = 38;
        TIME_SYS_REF_TIME_START = 40;
        TIME_SYS_REF_TIME_END = 45;
        TIME_SYS_REF_WEEK_NUMBER_START = 47;
        TIME_SYS_REF_WEEK_NUMBER_END = 50;
        TIME_SYS_CORR_SVID_START = 52;
        TIME_SYS_CORR_SVID_END = 56;
        TIME_SYS_CORR_UTCID_START = 58;
        TIME_SYS_CORR_UTCID_END = 59;
        TIME_SYS_CORR_PARAMETER_1_FORMAT = "%17.10f";
        TIME_SYS_CORR_PARAMETER_2_FORMAT = "%16.9f";
    end

    methods
        function [header, isParsed] = parseHeaderLineImpl(obj, header, label, lines)
            isParsed = true;
            switch label
                case obj.IONO_CORR_LABEL
                    header = parseHeaderIonoCorr(obj, header, lines);
                case obj.TIME_SYS_CORR_LABEL
                    header = parseHeaderTimeSysCorr(obj, header, lines);
                otherwise
                    isParsed = false;
            end
        end

        function data = parseData(obj, rawdata, ~)
            data = struct;
            numEpochVals = 6;
            epochStart = 2;
            epochIdx = (epochStart-1)+(1:numEpochVals);
            if ~isempty(rawdata.G)
                arrData = rawdata.G.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.GPS = array2timetable(arrData, "RowTimes", t, "VariableNames", ...
                    obj.GPSRecordEntries);
            end
            if ~isempty(rawdata.E)
                arrData = rawdata.E.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.Galileo = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.GalileoRecordEntries);
            end
            if ~isempty(rawdata.R)
                arrData = rawdata.R.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.GLONASS = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.GLONASSRecordEntries);
            end
            if ~isempty(rawdata.C)
                arrData = rawdata.C.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.BeiDou = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.BeiDouRecordEntries);
            end
            if ~isempty(rawdata.I)
                arrData = rawdata.I.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.NavIC = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.NavICRecordEntries);
            end
            if ~isempty(rawdata.J)
                arrData = rawdata.J.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.QZSS = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.QZSSRecordEntries);
            end
            if ~isempty(rawdata.S)
                arrData = rawdata.S.';
                t = datetime(arrData(:,epochIdx));
                arrData(:,epochIdx) = [];
                data.SBAS = array2timetable(arrData, "RowTimes", t, ...
                    "VariableNames", ...
                    obj.SBASRecordEntries);
            end
        end

        function header = parseHeaderIonoCorr(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                ionoParams = textscan(extractBetween(line, obj.IONO_CORR_PARAMETERS_START, obj.IONO_CORR_PARAMETERS_END), obj.IONO_CORR_PARAMETERS_FORMAT);
                ionoCorr(jj,1) = struct( ...
                    "CorrectionType", extractBetween(line, obj.IONO_CORR_CORRECTION_TYPE_START, obj.IONO_CORR_CORRECTION_TYPE_END), ...
                    "Parameters", ionoParams{:}.', ...
                    "TimeMark", extractBetween(line, obj.IONO_CORR_TIME_MARK_START, obj.IONO_CORR_TIME_MARK_END), ...
                    "SVID", str2double(extractBetween(line, obj.IONO_CORR_SVID_START, obj.IONO_CORR_SVID_END)));
            end
            header.IonosphericCorrections = ionoCorr;
        end
        function header = parseHeaderTimeSysCorr(obj, header, lines)
            for jj = numel(lines):-1:1
                line = lines(jj);
                timeParam1 = textscan(extractBetween(line, obj.TIME_SYS_CORR_PARAMETER_1_START, obj.TIME_SYS_CORR_PARAMETER_1_END), obj.TIME_SYS_CORR_PARAMETER_1_FORMAT);
                timeParam2 = textscan(extractBetween(line, obj.TIME_SYS_CORR_PARAMETER_2_START, obj.TIME_SYS_CORR_PARAMETER_2_END), obj.TIME_SYS_CORR_PARAMETER_2_FORMAT);
                timeParams = [timeParam1{:}.', timeParam2{:}.'];
                timeCorr(jj,1) = struct( ...
                    "CorrectionType", extractBetween(line, obj.TIME_SYS_CORR_CORRECTION_TYPE_START, obj.TIME_SYS_CORR_CORRECTION_TYPE_END), ...
                    "Parameters", timeParams, ...
                    "ReferenceTime", str2double(extractBetween(line, obj.TIME_SYS_REF_TIME_START, obj.TIME_SYS_REF_TIME_END)), ...
                    "ReferenceWeekNumber", str2double(extractBetween(line, obj.TIME_SYS_REF_WEEK_NUMBER_START, obj.TIME_SYS_REF_WEEK_NUMBER_END)), ...
                    "SVID", extractBetween(line, obj.TIME_SYS_CORR_SVID_START, obj.TIME_SYS_CORR_SVID_END), ...
                    "UTCID", str2double(extractBetween(line, obj.TIME_SYS_CORR_UTCID_START, obj.TIME_SYS_CORR_UTCID_END)));
            end
            header.TimeSystemCorrections = timeCorr;
        end
    end
end
