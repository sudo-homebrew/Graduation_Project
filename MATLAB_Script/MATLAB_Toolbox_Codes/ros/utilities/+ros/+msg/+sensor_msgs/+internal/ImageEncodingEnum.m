classdef ImageEncodingEnum < uint8
%This class is for internal use only. It may be removed in the future.

%ImageEncodingEnum Enumeration class for image encodings

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    enumeration
        enc_rgb8            (1)
        enc_rgba8           (2)
        enc_rgb16           (3)
        enc_rgba16          (4)
        enc_bgr8            (5)
        enc_bgra8           (6)
        enc_bgr16           (7)
        enc_bgra16          (8)
        enc_mono8           (10)
        enc_mono16          (11)
        enc_32fc1           (12)
        enc_32fc2           (13)
        enc_32fc3           (14)
        enc_32fc4           (15)
        enc_64fc1           (16)
        enc_64fc2           (17)
        enc_64fc3           (18)
        enc_64fc4           (19)
        enc_8uc1            (20)
        enc_8uc2            (21)
        enc_8uc3            (22)
        enc_8uc4            (23)
        enc_8sc1            (24)
        enc_8sc2            (25)
        enc_8sc3            (26)
        enc_8sc4            (27)
        enc_16uc1           (28)
        enc_16uc2           (29)
        enc_16uc3           (30)
        enc_16uc4           (31)
        enc_16sc1           (32)
        enc_16sc2           (33)
        enc_16sc3           (34)
        enc_16sc4           (35)
        enc_32sc1           (36)
        enc_32sc2           (37)
        enc_32sc3           (38)
        enc_32sc4           (39)
        enc_bayer_rggb8     (40)
        enc_bayer_bggr8     (41)
        enc_bayer_gbrg8     (42)
        enc_bayer_grbg8     (43)
        enc_bayer_rggb16    (44)
        enc_bayer_bggr16    (45)
        enc_bayer_gbrg16    (46)
        enc_bayer_grbg16    (47)
    end
end
