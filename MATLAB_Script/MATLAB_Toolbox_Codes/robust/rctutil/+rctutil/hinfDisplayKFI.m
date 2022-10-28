function hinfDisplayKFI(INFO)
% Displays progress report for full-information GAMMA iterations.

%   Copyright 2018 The MathWorks, Inc.
X = INFO.X;
GAM = INFO.GAM;
if INFO.PASS
   fprintf('  %s    %8.1e     p\n',localDispGam(GAM,INFO),max([0,min(eig(X))]))
else
   if hasInfNaN(X)
      xmin = Inf;
   else
      xmin = min(eig(X));
   end
   fprintf('  %s    %8.1e #   f\n',localDispGam(GAM,INFO),xmin)
end

%-----------------------------------
function s = localDispGam(GAM,INFO)
% Displays GAMMA value
% NOTE: length(S) = ndigits+5.
ndigits = INFO.ndigits;
if isinf(GAM)
   offset = floor(ndigits/2)+1;
   s = [blanks(offset) 'Inf' blanks(ndigits+2-offset)];
else
   GAMFORM = sprintf('%%.%de',ndigits-1);
   s = sprintf(GAMFORM,GAM);
end