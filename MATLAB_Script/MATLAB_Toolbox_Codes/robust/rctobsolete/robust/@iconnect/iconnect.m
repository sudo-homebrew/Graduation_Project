%ICONNECT Creates empty ICONNECT (interconnection) object.
%    Interconnection objects (class ICONNECT) are an alternate
%    to SYSIC, and are used to build complex interconnections of
%    uncertain matrices and systems.
% 
%    An ICONNECT object has 3 fields to be set by the user, 'Input',
%    'Output' and 'Equation'.  'Input' and 'Output' are ICSIGNAL objects,
%    while 'Equation' is a cell-array of equality constraints (using EQUATE)
%    on ICSIGNAL objects.  Once these are specified, then the 'System'
%    property is the input/output model, implied by the constraints in
%    'Equation' relating the variables defined in 'Input' and 'Output'.
% 
% EXAMPLE
%    Create three scalar icsignals, r, e and y.  Create an empty ICONNECT
%    object.  Define the output of the interconnection to be [e;y], and
%    the input to be r.
%    Define two constraints among the variables: e = r-y, and
%    y = (2/s) e.  Get the transfer function representation of the 
%    relationship between the input (r) and the output [e;y].
% 
%    r = icsignal(1);
%    e = icsignal(1);
%    y = icsignal(1);
%    M = iconnect;
%    M.Input = r;
%    M.Output = [e;y];
%    M.Equation{1} = equate(e,r-y);
%    M.Equation{2} = equate(y,tf(2,[1 0])*e);
%    tf(M.System)
% 
%   Each equation represents an equality constraint among the variables.
%   Once the input and output variables are chosen, the implicit relationship
%   between them is made explicit using IMP2EXP.
% 
%   See also EQUATE, ICSIGNAL, ICZERO, SYSIC

%   Copyright 2004-2011 The MathWorks, Inc.

function A = iconnect(varargin)

A.Equation = cell(0,1);
A.Input = icsignal;
A.Output = icsignal;
A = class(A,'iconnect',gsref());

ni = nargin;
if floor(ni/2)==ceil(ni/2)
   for i=1:ni/2
      A = set(A,varargin{2*(i-1)+1},varargin{2*i});
   end
end
