function pcross = skew( p )
%This function is for internal use only. It may be removed in the future.

%SKEW Convert the cross product of a 3D vector into a skew symmetric matrix
%   (skew-symmetrify)
%   skew(p)* --> p x

%   Copyright 2016 The MathWorks, Inc.

%#codegen

    pcross = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0]; 

end

