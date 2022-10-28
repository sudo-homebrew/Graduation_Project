function hinfDisplayKC(INFO)
% Displays progress report for output-feedback GAMMA iterations.

%   Copyright 2018 The MathWorks, Inc.
X = INFO.X;
Y = INFO.Y;
GAM = INFO.GAM;

if INFO.PASS
   rhoXY = max(eig(X*Y))/GAM^2;
   fprintf('  %s    %8.1e    %8.1e     %9.3e     p\n',...
      localDispGam(GAM,INFO),max([0,min(eig(X))]),max([0,min(eig(Y))]),max([0,rhoXY]))
else
   if hasInfNaN(X)
      xmin = Inf;  xflag = '#';
   else
      xeig = eig(X);
      xmin = min(xeig);
      if xmin<-1e-8*(1+max(abs(xeig)))
         xflag = '#';
      else
         xflag = ' ';
      end
   end
   if hasInfNaN(Y)
      ymin = Inf;  yflag = '#';
   else
      yeig = eig(Y);
      ymin =  min(yeig);
      if ymin<-1e-8*(1+max(abs(yeig)))
         yflag = '#';
      else
         yflag = ' ';
      end
   end
   if isfinite(xmin) && isfinite(ymin)
      rhoxy = max(real(eig(X*Y)))/GAM^2;
      if rhoxy>=1
         xyflag = '#';
      else
         xyflag = ' ';
      end
   else
      rhoxy = NaN;  xyflag = '#';
   end
   fprintf('  %s    %8.1e %s  %8.1e %s   %9.3e %s   f\n',...
      localDispGam(GAM,INFO),xmin,xflag,ymin,yflag,rhoxy,xyflag)
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

