%RINEXREAD Read data from RINEX file
%   DATA = RINEXREAD(FILENAME) returns a struct from a RINEX version 3
%   file, where FILENAME is the name of the RINEX file. The struct contains
%   a field for each satellite system read from the file. A field is a
%   timetable with a row for each record and a column for each entry in
%   that record. The content of a record depends on the RINEX file type and
%   the satellite system. 
%
%   For RINEX navigation message files:
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.GPSRecordEntries')">GPS Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.GalileoRecordEntries')">Galileo Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.GLONASSRecordEntries')">GLONASS Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.BeiDouRecordEntries')">BeiDou Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.NavICRecordEntries')">NavIC/IRNSS Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.QZSSRecordEntries')">QZSS Navigation Message</a>
%       <a href="matlab:help('nav.internal.rinex.NavFileContents.SBASRecordEntries')">SBAS Navigation Message</a>
%
%   For RINEX observation data files, each satellite system defines the
%   observation types that are in the timetable. Each observation has a
%   three-element descriptor containing the type, band, and attribute.
%   Possible types are:
%       C - Code/pseudorange
%       L - Phase
%       D - Doppler
%       S - Raw signal strength (carrier to noise ratio)
%       X - Receiver channel numbers
%   All observations have a signal strength indicator (SSI) and phase
%   measurements have a loss of lock indicator (LLI) as additional columns
%   in the timetable. For a full list of three-element descriptors, refer
%   to the "SYS / # / OBS TYPES" header label in table A2 of the RINEX 3.05
%   standard.
%
%   Example:
%       % Get GPS navigation message data from a RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_GN.rnx"; 
%       data = rinexread(filename);
%
%       % Get Galileo navigation message data from a RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_EN.rnx"; 
%       data = rinexread(filename);
%
%       % Get GLONASS navigation message data from a RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_RN.rnx";
%       data = rinexread(filename);
%
%       % Get BeiDou navigation message data from a RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_CN.rnx";
%       data = rinexread(filename);
%
%       % Get NavIC/IRNSS navigation message data from a RINEX file.
%       filename = "ARHT00ATA_R_20211750000_01D_IN.rnx";
%       data = rinexread(filename);
%
%       % Get QZSS navigation message data from a RINEX file.
%       filename = "ARHT00ATA_R_20211750000_01D_JN.rnx";
%       data = rinexread(filename);
%
%       % Get SBAS navigation message data from a RINEX file.
%       filename = "GOP600CZE_R_20211750000_01D_SN.rnx";
%       data = rinexread(filename);
%
%       % Get mixed observation data from a RINEX file.
%       filename = "GODS00USA_R_20211750000_01H_30S_MO.rnx";
%       data = rinexread(filename);
%
%   References:
%   [1] International GNSS Service (IGS), Daily 30-Second GPS Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Jun. 25,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_n_001.
%   [2] International GNSS Service (IGS), Daily 30-Second Galileo Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Jun. 25,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_l_001.
%
%   [3] International GNSS Service (IGS), Daily 30-Second GLONASS Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Aug. 19,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_g_001.
%
%   [4] International GNSS Service (IGS), Daily 30-Second BeiDou Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Aug. 19,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_f_001.
%
%   [5] International GNSS Service (IGS), Daily 30-Second NavIC/IRNSS
%       Broadcast Ephemeris Data, NASA Crustal Dynamics Data Information 
%       System (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: 
%       Aug. 19, 2021. [Online]. Available: 
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_i_001.
%
%   [6] International GNSS Service (IGS), Daily 30-Second QZSS Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Aug. 19,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_q_001.
%
%   [7] International GNSS Service (IGS), Daily 30-Second SBAS Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Aug. 19,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_h_001.
%
%   [8] International GNSS Service (IGS), Hourly 30-Second Observation
%       Data, NASA Crustal Dynamics Data Information System (CDDIS), 
%       Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Aug. 19, 2021. 
%       [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_hourly_o_001.
%
%   See also rinexinfo, gnssconstellation, pseudoranges.

 
%   Copyright 2021 The MathWorks, Inc.

