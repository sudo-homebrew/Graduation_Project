function musynDispSummary( IA, iter, opt )
% Display iteration summary and return matrix of display data.

%   Copyright 2019 The MathWorks, Inc.
if ~strcmp(opt.Display,'off')
   % Build summary
   MixedMu = strcmpi(opt.MixedMU,'on');
   ShowFull = strcmp(opt.Display,'full');
   if iter==1 || ShowFull
      if MixedMu
         fprintf('\n\nDG-K ITERATION SUMMARY:\n')
         fprintf('-------------------------------------------------------------------\n')
         fprintf('                       Robust performance                 Fit order\n')
         fprintf('-------------------------------------------------------------------\n')
         fprintf('  Iter         K Step       Peak MU       DG Fit           D      G\n')
      else
         fprintf('\n\nD-K ITERATION SUMMARY:\n')
         fprintf('-----------------------------------------------------------------\n')
         fprintf('                       Robust performance               Fit order\n')
         fprintf('-----------------------------------------------------------------\n')
         fprintf('  Iter         K Step       Peak MU       D Fit             D\n')
      end
   end
   if ShowFull
      start = 1;
   else
      start = iter;
   end
   for ct=start:iter
      if MixedMu
         if IA(ct).gamma<Inf
            fprintf('  %3d       %9.4g    %9.4g    %9.4g          %3d    %3d\n',...
               ct,IA(ct).gamma,IA(ct).PeakMu,IA(ct).PeakMuFit,...
               IA(ct).FitOrder(1),IA(ct).FitOrder(2));
         else
            fprintf('  %3d       %9.4g    %9.4g    %9.4g          %3d    %3d\n',...
               ct,Inf,Inf,Inf,NaN,NaN);
         end
      else
         if IA(ct).gamma<Inf
            fprintf('  %3d       %9.4g    %9.4g    %9.4g           %3d\n',...
               ct,IA(ct).gamma,IA(ct).PeakMu,IA(ct).PeakMuFit,IA(ct).FitOrder(1));
         else
            fprintf('  %3d       %9.4g    %9.4g    %9.4g           %3d\n',...
               ct,Inf,Inf,Inf,NaN);
         end
      end
   end
end