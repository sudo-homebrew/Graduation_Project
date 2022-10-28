function disp(blk)
% Display method.

%   Copyright 1986-2020 The MathWorks, Inc.

% Block info
Nominal = sprintf('%.3g',blk.NominalValue_);
switch blk.Mode_
   case 'PlusMinus'
      PM = blk.PlusMinus_;
      M = message('Robust:umodel:ureal13',...
         blk.Name,Nominal,sprintf('[%.3g,%.3g]',PM(1),PM(2)));
   case 'Range'
      R = blk.Range;
      M = message('Robust:umodel:ureal12',...
         blk.Name,Nominal,sprintf('[%.3g,%.3g]',R(1),R(2)));
   case 'Percentage'
      P = blk.Percentage;
      M = message('Robust:umodel:ureal14',...
         blk.Name,Nominal,sprintf('[%.3g,%.3g]',P(1),P(2)));
end
fprintf('  %s\n\n',getString(M))
