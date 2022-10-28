function disp(blk)
% Display method.

%   Copyright 1986-2020 The MathWorks, Inc.
% Variable name
Nominal = num2str(blk.NominalValue_,'%.3g');
switch blk.Mode_
   case 'Radius'
      M = message('Robust:umodel:ucomplex6',...
         blk.Name,Nominal,sprintf('%.3g',blk.Radius_));
   case 'Percentage'
      M = message('Robust:umodel:ucomplex7',...
         blk.Name,Nominal,sprintf('%.3g',blk.Percentage));
end
fprintf('  %s\n\n',getString(M))
