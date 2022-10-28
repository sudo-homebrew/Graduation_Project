function validateInputsImpl(~, pos, vel)
%VALIDATEINPUTSIMPL Step method input validation for gnssSensor object

%   Copyright 2020 The MathWorks, Inc.

%#codegen

validateattributes(pos, {'single', 'double'}, ...
    {'real', '2d', 'ncols', 3});
validateattributes(vel, {'single', 'double'}, ...
    {'real', '2d', 'ncols', 3, 'nrows', size(pos, 1)});
end
