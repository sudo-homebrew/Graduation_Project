function PLOTS = musynShowMU(uppermu,DGInfo,DGCL,CLg,MixedMU,PLOTS)
% Plot MU and scaled closed-loop norms

%   Copyright 2019 The MathWorks, Inc.

% Configure plot
sopt = sigmaoptions('cstprefs');
sopt.MagUnits = 'abs';
sopt.MagScale = 'log';
sopt.YLabel.String = getString(message('Robust:design:dgview11'));
sopt.Grid = 'on';
[Mw,w,Ts] = frdata(CLg);
Nw = numel(w);

% UPPERMUS = relaxed upper bound for smoothed D,G
uppermuS = DGInfo.ub;

% Norm of D(jw)*CL(jw)/D(jw)
if MixedMU
   Drw = DGInfo.Dr;
   Dcw = DGInfo.Dc;
   gainSCL = zeros(Nw,1);
   for ct=1:Nw
      DL = chol(Drw(:,:,ct));
      DR = chol(Dcw(:,:,ct));
      gainSCL(ct) = norm(DL*Mw(:,:,ct)/DR);
   end
end

% Scaled closed-loop norm after fitting
muPeakS = max(uppermuS);
[sv,f] = sigma(DGCL,{w(1) w(end)}); 
DGCLg = frd(muPeakS*sv(1,:),f,Ts);

% Build plot
uppermu = frd(uppermu,w,Ts);
uppermuS = frd(uppermuS,w,Ts);
f = PLOTS.MU;
if isempty(f) || ~ishandle(f)
   f = figure('IntegerHandle','off','NumberTitle','off',...
      'Name',getString(message('Robust:design:dgview12')));
   PLOTS.MU = f;
else
   figure(f);
end
if MixedMU
   sigmaplot(uppermu,uppermuS,DGCLg,'g--',frd(gainSCL,w,Ts),'k--',sopt), grid
   legend('MU upper bound','Scaled CL for D,G data',...
      'Scaled CL for fitted D,G',...
      'Scaled CL for fitted D only','Location','SouthWest')
else
   sigmaplot(uppermu,uppermuS,DGCLg,'g--',sopt), grid
   legend('MU upper bound','Scaled CL for D data ',...
      'Scaled CL for fitted D','Location','SouthWest')
end
% else
%    % Show mixed MU if MixedMU=off and there are UREAL blocks (expensive)
%    [mixedMuBnds,~] = mussv(CLg,BLK);
%    upperMixedMu = mixedMuBnds(1,1);
%    sigmaplot(uppermu,gainSCL,DGCLg,'g--',upperMixedMu,'k--',sopt), grid
%    legend('MU upper bound','Scaled CL for D data ',...
%          'Scaled CL for fitted D','Mixed MU UB','Location','SouthWest')
% end
grid, title(getString(message('Robust:design:dgview13')))
