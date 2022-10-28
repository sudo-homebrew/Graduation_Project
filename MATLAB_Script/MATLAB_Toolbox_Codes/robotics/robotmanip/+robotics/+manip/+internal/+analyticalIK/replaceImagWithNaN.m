function q = replaceImagWithNaN(qToCheck)
%replaceImagWithNaN Replace imaginary and empty elements with NaNs
%   This function replaces imaginary values with NaNs. This is useful when
%   the element is part of a matrix, and rendering one element of the
%   matrix imaginary will make the entire matrix imaginary. Furthermore, it
%   may be used to filter invalid solutions.
 
%   Copyright 2020 The MathWorks, Inc.
 
    if isempty(qToCheck)
        q = NaN;
    elseif ~isEqualWithinTolerance(imag(qToCheck), 0)
        q = NaN;
    else
        q = real(qToCheck);
    end
 
end
