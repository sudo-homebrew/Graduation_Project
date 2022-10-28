%DISPLAY   Pretty-print for ICSGINAL objects.
% 
%    DISPLAY(ICS) is invoked by typing ICS followed
%    by a carriage return.  DISPLAY produces a custom
%    display of ICSGINAL objects.

function display(arg1)
% Copyright 2003-2004 The MathWorks, Inc.

disp(mkdstr(arg1))