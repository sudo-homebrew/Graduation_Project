function tf = isequalNZ(a,b)
% This function is for internal use and may be removed or modified
%
%isequalNZ Check equality of nonzero elements of two inputs
%   tf = isequalNZ(a,b) for two inputs, a and b of the same size, returns
%   an array tf of the same size as a and b, where each element in the
%   array is true if both conditions are satisfied:
%   1) a(i) == b(i)
%   2) a(i) ~= 0
%
% Example:
%   % The following should return [false,false,true,false]
%   a = [0 1 2 3];
%   b = [1 0 2 4];
%   tf = fusion.internal.isequalNZ(a,b)

% Copyright 2018 The MathWorks, Inc.

%#codegen

tf = (a==b);
tf(:) = tf(:) & (a(:)~=zeros(1,'like',a));