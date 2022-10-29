%RINEXINFO Get information about a RINEX file.
%   FILEINFO = RINEXINFO(FILENAME) returns a structure whose fields contain
%   information about the contents of a RINEX file. FILENAME is a character
%   vector or string scalar that specifies the name of the RINEX file.
%
%   Example:
%       % Query GPS navigation message RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_GN.rnx"; 
%       info = rinexinfo(filename);
%
%       % Query Galileo navigation message RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_EN.rnx"; 
%       info = rinexinfo(filename);
%
%       % Query GLONASS navigation message RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_RN.rnx";
%       data = rinexinfo(filename);
%
%       % Query BeiDou navigation message RINEX file.
%       filename = "GODS00USA_R_20211750000_01D_CN.rnx";
%       data = rinexinfo(filename);
%
%       % Query NavIC/IRNSS navigation message RINEX file.
%       filename = "ARHT00ATA_R_20211750000_01D_IN.rnx";
%       data = rinexinfo(filename);
%
%       % Query QZSS navigation message RINEX file.
%       filename = "ARHT00ATA_R_20211750000_01D_JN.rnx";
%       data = rinexinfo(filename);
%
%       % Query SBAS navigation message RINEX file.
%       filename = "GOP600CZE_R_20211750000_01D_SN.rnx";
%       data = rinexinfo(filename);
%
%       % Query mixed observation data RINEX file.
%       filename = "GODS00USA_R_20211750000_01H_30S_MO.rnx";
%       data = rinexinfo(filename);
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
%   See also rinexread, gnssconstellation, pseudoranges.

 
%   Copyright 2021 The MathWorks, Inc.

