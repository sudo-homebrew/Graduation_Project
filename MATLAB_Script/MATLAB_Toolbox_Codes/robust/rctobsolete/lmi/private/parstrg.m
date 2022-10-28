% called by MAGREQ

% Author: P. Gahinet  10/94
% Copyright 1995-2004 The MathWorks, Inc.

function [k,names]=parstrg(str)
% parses a string with blank or comma separators

names=[];
str=[blanks(1) strrep(str,',',' ') blanks(1)];

while ~isempty(str)
  [tok,str]=strtok(str);
  names=strstack(names,tok);
end

k=size(names,1);
