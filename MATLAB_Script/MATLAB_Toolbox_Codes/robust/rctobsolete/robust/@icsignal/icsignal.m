function varargout = icsignal(sys,siglist,sigdim)
%ICSIGNAL   Construct an ICSIGNAL object of specified dimension
%
%   Y = ICSIGNAL(N) creates an ICSIGNAL of dimension N.  All ICSIGNALs
%   are to be thought of as column vectors, so SIZE(Y) returns [N 1].
%
%   Y = ICSIGNAL(N,'Name') creates an ICSIGNAL of dimension N, with
%   internal name 'Name'.
%
%   See also ICONNECT, SYSIC.

%   Copyright 2003-2011 The MathWorks, Inc.
% System (DOUBLE, or UMAT, or USS, or ...)
% SignalList (cellarray of names, Nx1)
% SignalDim (column vector of dimensions, Nx1)
superiorto('uss',...
   'umat','double','lti','tf','ss','zpk','atom',...
   'ureal','ultidyn','ucomplex','ucomplexm');

if nargin==2
   dim = sys;
   tmp = ss([],[],[],eye(dim));
   tmp.InputGroup = {1:dim siglist};
   tmp.OutputGroup = {1:dim siglist};
   out.System = tmp;
   out.SignalList = {siglist};
   out.SignalDim = [sys];
   varargout{1} = class(out,'icsignal',gsref());
elseif nargin==0
   out.System = eye(0);
   out.SignalList = cell(0,1);
   out.SignalDim = zeros(0,1);
   varargout{1} = class(out,'icsignal',gsref());
elseif nargin==3
   out.System = sys;
   out.SignalList = siglist;
   out.SignalDim = [sigdim];
   varargout{1} = class(out,'icsignal',gsref());
elseif nargin==1
   switch class(sys)
   case 'icsignal'
         varargout{1} = sys;
   case 'double'
         str = int2str(ceil(9999999*rand));
         lstr = length(str);
         pdd = '0'*ones(1,8-lstr);
         name = ['UNNAMED_' pdd str];
         out.System = eye(sys);
         out.SignalList = {name};
         out.SignalDim = [sys];
         out = class(out,'icsignal',gsref());
         varargout{1} = out;
   otherwise
      error('Invalid Syntax');
   end
else
   error('Invalid syntax');
end
