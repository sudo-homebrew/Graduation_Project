% function [gostopd,skipg,op,iblk,ord,irow,icol,perctol,maxord,m1,m2,m1g,m2g] = ...
% 		fitchosd(c,blk,blkr,iblk,irow,icol,fitstatd,fitstatg,pimp,curord,...
% 			perctol,maxord);

%   Copyright (c) 1991-96 by MUSYN Inc. and The MathWorks

%   Modified for mixed case - P. M. Young, December, 1996.

function [gostopd,skipg,op,iblk,ord,irow,icol,perctol,maxord,m1,m2,m1g,m2g] = ...
		fitchosd(c,blk,blkr,iblk,irow,icol,fitstatd,fitstatg,pimp,curord,...
			perctol,maxord);

	skipg = 0;
	num = str2num(c);
	m1g = [];
	m2g = [];

	gostopd=[]; op=[]; ord=[]; m1=[]; m2=[];

	if ~isempty(num) && ~isnan(num)
		if floor(num)==ceil(num) && num >= 0
			ord = num;
			op = getString(message('Robust:obsolete:gfitchosd_op1'));
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_1',ord));
			m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
			gostopd = 1;
		else
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_2'));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		end
	elseif strcmp(deblank(c),'ch')
		gostopd = 1;
		op = getString(message('Robust:obsolete:gfitchosd_op3'));
		disp(getString(message('Robust:obsolete:gfitchosd_disp')));

		if any(blkr(:,1)<0)
            disp(getString(message('Robust:obsolete:gfitchosd_disp1')));
		end
		
        disp(getString(message('Robust:obsolete:gfitchosd_disp2')));
		m1 = [];
		m2 = [];
	elseif strcmp(deblank(c),'apf')	% autofit (overrides your choises)
		m1 = getString(message('Robust:obsolete:gfitchosd_m1_3'));
		op = getString(message('Robust:obsolete:gfitchosd_op4'));
		gostopd = 1;
		m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
	elseif strcmp(deblank(c),'s')	% 
		m1 = ' ';
		op = getString(message('Robust:obsolete:gfitchosd_op5'));
		gostopd = 1;
		m2 = ' ';
	elseif strcmp(deblank(c),'nd')		% moves to next d
		nblk = size(blk,1);
		if blk(iblk,2) == 0 && blk(iblk,1) > 1
			if irow == blk(iblk,1) && icol == blk(iblk,1)
				irow = 1;
				icol = 1;
				if iblk == nblk
					iblk = 1;
				else
					iblk = iblk + 1;
				end
			elseif irow == blk(iblk,1) && icol < blk(iblk,1)
				irow = 1;
				icol = icol + 1;
			elseif irow < blk(iblk,1)
				irow = irow + 1;
			end
		else
			irow = 1;
			icol = 1;
			if iblk == nblk
				iblk = 1;
			else
				iblk = iblk + 1;
			end
		end
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		gostopd = 1;
		if fitstatd(pp) == 1
			op = getString(message('Robust:obsolete:gfitchosd_op6'));
		else
			op = getString(message('Robust:obsolete:gfitchosd_op7'));
		end
		m1 = getString(message('Robust:obsolete:gfitchosd_m1_4'));
		m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
	elseif strcmp(deblank(c),'nb')		% moves to (1,1) of next block
		nblk = size(blk,1);
		if iblk == nblk
			iblk = 1;
		else
			iblk = iblk + 1;
		end
		irow = 1;
		icol = 1;
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		gostopd = 1;
		if fitstatd(pp) == 1
			op = getString(message('Robust:obsolete:gfitchosd_op6'));
		else
			op = getString(message('Robust:obsolete:gfitchosd_op7'));
		end
		m1 = getString(message('Robust:obsolete:gfitchosd_m1_5'));
		m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
	elseif strcmp(deblank(c),'mg')  % moves to G fitting
		if any(blkr(:,1)<0)
			gostopd = 0;
			m1g = getString(message('Robust:obsolete:gfitchosd_m1g'));
			m2g = getString(message('Robust:obsolete:gfitchosd_m2'));
			pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
			if fitstatd(pp) == 1
				op = getString(message('Robust:obsolete:gfitchosd_op6'));
			else
				op = getString(message('Robust:obsolete:gfitchosd_op7'));
			end
		else 
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_16'));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		end
	elseif strcmp(deblank(c),'e')		% exits (if done)
		if any(fitstatd~=1) 
			fl = min(find(fitstatd==0));
			bnum = max(find(pimp<=fl));
			bp = pimp(bnum);
			colnum = floor((fl-bp)/blk(bnum,1)) + 1;
			rownum = fl - bp - (colnum-1)*blk(bnum,1) + 1;
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_6',bnum,rownum,colnum));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		elseif any(fitstatg~=1) 
			fl = min(find(fitstatg==0));
			bnum = max(find(pimp<=fl));
			bp = pimp(bnum);
			colnum = floor((fl-bp)/blk(bnum,1)) + 1;
			rownum = fl - bp - (colnum-1)*blk(bnum,1) + 1;
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_7',bnum,rownum,colnum));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		else
			op = getString(message('Robust:obsolete:gfitchosd_op6')); % in case you come back in (for mixed)
			gostopd = 0;
			skipg = 1;
		end
	elseif strcmp(deblank(c),'u')		% increment order
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		op = getString(message('Robust:obsolete:gfitchosd_op1'));
		gostopd = 1;
		if fitstatd(pp) == 1
			ord = curord + 1;
		else
			ord = 0;
		end
		m1 = getString(message('Robust:obsolete:gfitchosd_m1_8',ord));
		m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
	elseif strcmp(deblank(c),'d')		% decrement order
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		if curord > 0 && fitstatd(pp) == 1
			ord = curord - 1; 
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_9',ord));
			op = getString(message('Robust:obsolete:gfitchosd_op1'));
			gostopd = 1;
		else
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_10'));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		end
		m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
	else
		dbc = deblank(c);
		if length(dbc) >2
			if strcmp(dbc(1:2),'mx')
				num = str2num(dbc(3:length(dbc)));
				if floor(num)==ceil(num) && num >= 0
					maxord = num;
					op = getString(message('Robust:obsolete:gfitchosd_op3'));
					m1 = getString(message('Robust:obsolete:gfitchosd_m1_11'));
					m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
					gostopd = 1;
				else
					m1 = getString(message('Robust:obsolete:gfitchosd_m1_12'));
					op = getString(message('Robust:obsolete:gfitchosd_op2'));
					gostopd = 1;
				end
			elseif strcmp(dbc(1:2),'at')
				num = str2num(dbc(3:length(dbc)));
				if num > 1
					perctol = num;
					op = getString(message('Robust:obsolete:gfitchosd_op3'));
					m1 = getString(message('Robust:obsolete:gfitchosd_m1_13'));
					m2 = getString(message('Robust:obsolete:gfitchosd_m2'));
					gostopd = 1;
				else
					m1 = getString(message('Robust:obsolete:gfitchosd_m1_14'));
					op = getString(message('Robust:obsolete:gfitchosd_op2'));
					gostopd = 1;
				end
			else
				m1 = getString(message('Robust:obsolete:gfitchosd_m1_15'));
				op = getString(message('Robust:obsolete:gfitchosd_op2'));
				gostopd = 1;
			end
		else
			m1 = getString(message('Robust:obsolete:gfitchosd_m1_15'));
			op = getString(message('Robust:obsolete:gfitchosd_op2'));
			gostopd = 1;
		end
	end
