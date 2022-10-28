% function jgfit = gfitgdat(magdata,ord,weight)
%
% GFITDAT fits a transfer function (magnitude and phase)
% to real data, MAGDATA, with a supplied frequency
% domain weighting function, WEIGHT. Both these should
% be VARYING matrices, with identical INDEPENDENT
% VARIABLE values.  Calls REALFIT (or FITSYS) to fit the real data.
% Returns a SISO system jgfit which is a fit to jg

% P. M. Young, December, 1996.
% GJB modified 12 Sept 08

%   Copyright 2009 The MathWorks, Inc.

function sys = gfitgdat(d,ord,wt)

fitflag = 1;  % toggles between  0 - use FITSYS
%                                1 - use REALFIT (based on LP approach)

if nargin < 2
   disp(['usage: jgfit = gfitgdat(magdata,ord,weight)']);
   return
end
%disp(' Inside GFITGDAT')

if d.Ts==0
    dflag = 0;
else
    dflag = d.Ts;
end

[drows,dcols] = size(d);
if ~isa(d,'frd')
   error(['MAGDATA should be a FRD object']);
end

omega = d.Frequency;
numpts = length(omega);
if nargin==2
   wt = frd(1,omega);
   wt.Ts = d.Ts;
end

if ~isa(wt,'frd')
   error(['WEIGHT should be a FRD object']);
end

d = real(d.ResponseData(:));
wt = abs(wt.ResponseData(:));
dflag = abs(dflag);
ord = 2*floor(ord/2);

if fitflag==0
	if ord >= 2
		d = frd(d,omega,dflag);
		wt = frd(wt,omega,dflag);
		sys = fitsys(sqrt(-1)*d,ord,wt); % pass jG to fit
        % now we fix this to ensure the phase is pure imaginary
		if dflag~=0
			sys = d2cbil(sys);
		end
		[aa,bb,cc,dd] = ssdata(sys);
        	[num,den] = ss2tf(aa,bb,cc,dd);
		sizfit = length(den);
		if sizfit > 0
			sizjunk = ceil(sizfit/2);
			junk = [zeros(1,sizjunk);ones(1,sizjunk)];
			junk = junk(:)';
			junkf = fliplr(junk);
			num = fliplr(fliplr(num).*junk(1:sizfit));
			den = fliplr(fliplr(den).*junkf(1:sizfit));
			sys = tf(num,den);
			if dflag~=0
				sys = c2dbil(sys);
			end
		else
			sys = 0;
		end
	else
		sys = 0;
    end
else
    % first map the frequency from discrete to continuous time 
    % if necessary.  Use EXACTLY the same transformation for frequency 
    % warping as co2di since that is how we will send the system back
	if dflag~=0
		omega = abs((1/dflag)*tan(0.5*omega*dflag));
	end
    % Now form dw = d/w and wtw = wt*w for fitting.  Thus when we fit these 
    % with a real g and then form sg we get the correct weighted fit 
    % to jg (s=jw).
	wclip = 0.25; % sets value for clipping wt if too small (if 0 no effect)
	wtemp = wt;
	wmax = max(wtemp);
	wtemp = max([wtemp wclip*wmax*ones(numpts,1)]')';
	wt = wtemp;
	indexw = find(abs(omega)>=1e-8);	% might want to change tolerance 
	d = d(indexw);
	wt = wt(indexw);
	omega = omega(indexw);		% removes omega=0 from freq resp
	if isempty(omega)
		error('  Frequency response must contain non-zero frequencies')
	end 
	dw = d./omega;
	wtw = wt.*omega;
	dw = frd(dw,omega,dflag);
	wtw = frd(wtw,omega,dflag);

    % Note we don't pass ord directly because REALFIT
    % fits the numerator with 2*ord and the denominator 
    % with 2*(ord+1) so that sg is proper and pure imaginary
    % If less than two states just fit with zero

   	if ord >= 2
      	ordn = floor(ord/2)-1;
      	sys = grealfit(dw,[.26,.02,ordn,ordn],wtw); %%% XXX need to convert
      	[aa,bb,cc,dd] = ssdata(sys);
        % Now form sg from g, unless it is constant (zero)
      	if isempty(aa)
         	sys = ss(0);
      	else
         	sys = ss(aa,bb,cc*aa,cc*bb);
      	end
   	else
      	sys = ss(0);
   	end
    % Now we have a fit to jg in sys.  It just remains to 
    % map it back to discrete if necessary
	if dflag~=0
		sys = c2dbil(sys);
        sys.Ts = dflag;
	end
end
