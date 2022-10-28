function W1 = getNCFWeight(Gd,wc,G)
% Construct W1 such that G*W1 has loop shape Gd in the following sense
%   * min singular value of G*W1 > |Gd| for w<wc
%   * max singular value of G*W1 < |Gd| for w>wc
% Gd is SISO and G is SISO or MIMO.

% Copyright 2021 The MathWorks, Inc.
[zd,pd,kd,Ts] = zpkdata(Gd,'v'); % Ts>0
ny = size(G,1);  % number of feedback loops

% Compute magnitude profiles Wlow and Whigh that enforce:
%   sigma_min(Wlow*Gs) > |Gd| for w<=wc
%   sigma_max(Whigh*Gs) < |Gd| for w>=wc
w = wc * [1e-5;1e-4;1e-3;1e-2;3.16e-2;1e-1;0.21;0.46;1;2.15;4.64;1e1;3.16e1;1e2;1e3;1e4;1e5];
if Ts==0
   s = complex(0,w);
else
   w = w(w<pi/Ts);
   s = exp(complex(0,w*Ts));
end
h = zpkfresp(zd,pd,kd,s,true);
gamd = abs(h(:));
svG = sigma(G,w);
% Make log-average of svd(G) equal to zero at wc
StaticGain = 1/prod(svG(:,w==wc))^(1/ny);
svG = svG * StaticGain;
Wlow = max(1,gamd./svG(ny,:)'); % REVISIT: REMOVE MAX(1,?
Whigh = min(1,gamd./svG(1,:)'); % REVISIT: REMOVE MIN(1,?
[w,Wmag] = localMergeProfiles(w,Wlow,Whigh,wc);

% Enforce properness (end slope <=0)
Wmag(end) = min(Wmag(end-1),Wmag(end));

% Fit rational W(s) to this magnitude data
W1 = ss(TuningGoal.fitMagProfile(w,Wmag));
W1 = W1 * (StaticGain/abs(evalfr(W1,1i*wc)));  % enforce |W1(j*wc)| = 1
if Ts~=0
   W1 = TuningGoal.resampleWeight(W1,Ts);
end

%-------------------------------------------------------------------
function [w,Wmag] = localMergeProfiles(w,Wlow,Whigh,wc)
% Connect low- and high-frequency magnitude profiles for W(s) in a way
% that minimizes the slope at the wc transition in the MIMO case.
%w0 = w;
nw = numel(w);
ic = find(w==wc);
if abs(1-Wlow(ic)/Whigh(ic))<0.25
   % No jump (SISO case) or small jump (well conditioned MIMO)
   Wmag = [Wlow(1:ic,:) ; Whigh(ic+1:nw,:)];
else
   % Smooth jump at w=wc
   Slope = -1;
   Gain = (w/wc).^Slope;
   ilow = find(Wlow(1:ic-1)<Gain(1:ic-1),1,'last');
   ihigh = find(Whigh(ic+1:nw)>Gain(ic+1:nw),1,'first');
   while isempty(ilow) || isempty(ihigh)
      % Increase slope until (w/wc)^Slope intersects both curves
      Slope = Slope-0.5;
      Gain = (w/wc).^Slope;
      ilow = find(Wlow(1:ic-1)<Gain(1:ic-1),1,'last');
      ihigh = find(Whigh(ic+1:end)>Gain(ic+1:nw),1,'first');
   end
   %fprintf('Slope = %.1e\n',Slope)
   w = w([1:ilow , ic+ihigh:nw]);
   Wmag = [Wlow(1:ilow,:) ; Whigh(ic+ihigh:nw,:)];
end
% figure(2), clf, subplot(211)
% W1 = TuningGoal.fitMagProfile(w,Wmag);
% W1 = W1/abs(evalfr(W1,1i*wc));  % enforce |W1(j*wc)| = 1
% sigma(frd(Wlow,w0),frd(Whigh,w0),frd(Wmag,w),'k--',W1,{wc/1e5,wc*1e5}), grid
% legend('Wlow','Whigh','W1','Fit')
