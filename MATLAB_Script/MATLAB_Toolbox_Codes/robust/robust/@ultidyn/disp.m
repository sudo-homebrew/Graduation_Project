function disp(blk)
% Display method.

%   Copyright 1986-2020 The MathWorks, Inc.

nyu = iosize(blk);
switch blk.Type_
   case 'G'
      MsgID = 'Robust:umodel:ultidyn9';
   case 'P'
      MsgID = 'Robust:umodel:ultidyn10';
end
M = message(MsgID,blk.Name,nyu(1),nyu(2),sprintf('%.3g',blk.Bound_));
fprintf('  %s\n\n',getString(M))
