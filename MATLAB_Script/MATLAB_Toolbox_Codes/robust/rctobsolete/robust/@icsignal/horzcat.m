%HORZCAT  Horizontal concatenation of ICSIGNAL objects.
%  
%MAT = HORZCAT(MAT1,MAT2,...) performs a concatenation operation of 
%  MAT = [MAT1 , MAT2, ...].
%
%See also: VERTCAT

function out = horzcat(varargin)
% Copyright 2003-2004 The MathWorks, Inc.

if nargin==1
    out = varargin{1};
else
    error('HORZCAT not allowed for ICSIGNAL');
end