% function [gostopg,gostopmain,op,iblk,ord,irow,icol,perctol,maxord,m1,m2,m1d,m2d] = ...
% 		fitchosg(c,blk,blkr,iblk,irow,icol,fitstatd,fitstatg,pimp,curord,...
% 			perctol,maxord);

%   Copyright (c) 1991-96 by MUSYN Inc. and The MathWorks

%   Modified for G scaling - P. M. Young, December, 1996.
%   GJB modified 11 Sept 2008

function [gostopg,gostopmain,op,iblk,ord,irow,icol,perctol,maxord,m1,m2,m1d,m2d] = ...
		fitchosg(c,blk,blkr,iblk,irow,icol,fitstatd,fitstatg,pimp,curord,...
			perctol,maxord)
        
	indexg = find(blkr(:,1)<0);
	numg = length(indexg);
	curg = find(indexg==iblk);
	gostopmain = 1;
	num = str2num(c);
	m1d = [];
	m2d = [];

	gostopg=[]; op=[]; ord=[]; m1=[]; m2=[];

	if ~isempty(num) && ~isnan(num)
		if floor(num)==ceil(num) && num >= 0
			ord = num;
			if irow==icol
				ord = 2*floor(ord/2);
			end
			op = getString(message('Robust:obsolete:gfitchosg_op1'));
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_1',ord));
			m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
			gostopg = 1;
		else
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_2'));
			op = getString(message('Robust:obsolete:gfitchosg_op2'));
			gostopg = 1;
		end
	elseif strcmp(deblank(c),'ch')
		gostopg = 1;
		op = getString(message('Robust:obsolete:gfitchosg_op3'));
        disp(getString(message('Robust:obsolete:gfitchosg_disp')));
		m1 = [];
		m2 = [];
	elseif strcmp(deblank(c),'apf')	% autofit (overrides your choises)
		m1 = getString(message('Robust:obsolete:gfitchosg_m1_3'));
		op = getString(message('Robust:obsolete:gfitchosg_op4'));
		gostopg = 1;
		m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
	elseif strcmp(deblank(c),'s')	% 
		m1 = ' ';
		op = getString(message('Robust:obsolete:gfitchosg_op5'));
		gostopg = 1;
		m2 = ' ';
	elseif strcmp(deblank(c),'ng')		% moves to next g
		if blk(iblk,2) == 0 && blk(iblk,1) > 1
			if irow == blk(iblk,1) && icol == blk(iblk,1)
				irow = 1;
				icol = 1;
				if iblk == indexg(numg)
					iblk = indexg(1);
				else
					iblk = indexg(curg + 1);
				end
			elseif irow == icol
				irow = 1;
				icol = icol + 1;
			elseif irow < icol
				irow = irow + 1;
			end
		else
			irow = 1;
			icol = 1;
			if iblk == indexg(numg)
				iblk = indexg(1);
			else
				iblk = indexg(curg + 1);
			end
		end
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		gostopg = 1;
		if fitstatg(pp) == 1
			op = getString(message('Robust:obsolete:gfitchosg_op6'));
		else
			op = getString(message('Robust:obsolete:gfitchosg_op7'));
		end
		m1 = getString(message('Robust:obsolete:gfitchosg_m1_4'));
		m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
	elseif strcmp(deblank(c),'nb')		% moves to (1,1) of next block
		if iblk == indexg(numg)
			iblk = indexg(1);
		else
			iblk = indexg(curg + 1);
		end
		irow = 1;
		icol = 1;
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		gostopg = 1;
		if fitstatg(pp) == 1
			op = getString(message('Robust:obsolete:gfitchosg_op6'));
		else
			op = getString(message('Robust:obsolete:gfitchosg_op7'));
		end
		m1 = getString(message('Robust:obsolete:gfitchosg_m1_5'));
		m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
	elseif strcmp(deblank(c),'md')  % moves to D fitting
		gostopg = 0;
		m1d = getString(message('Robust:obsolete:gfitchosg_m1d'));
		m2d = getString(message('Robust:obsolete:gfitchosg_m2'));
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		if fitstatg(pp) == 1
			op = getString(message('Robust:obsolete:gfitchosg_op6'));
		else
			op = getString(message('Robust:obsolete:gfitchosg_op7'));
		end
	elseif strcmp(deblank(c),'e')		% exits (if done)
		if any(fitstatg~=1) 
			fl = min(find(fitstatg==0));
			bnum = max(find(pimp<=fl));
			bp = pimp(bnum);
			colnum = floor((fl-bp)/blk(bnum,1)) + 1;
			rownum = fl - bp - (colnum-1)*blk(bnum,1) + 1;
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_6',bnum,rownum,colnum));
			op = getString(message('Robust:obsolete:gfitchosg_op2'));
			gostopg = 1;
		elseif any(fitstatd~=1) 
			fl = min(find(fitstatd==0));
			bnum = max(find(pimp<=fl));
			bp = pimp(bnum);
			colnum = floor((fl-bp)/blk(bnum,1)) + 1;
			rownum = fl - bp - (colnum-1)*blk(bnum,1) + 1;
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_7',bnum,rownum,colnum));
			op = getString(message('Robust:obsolete:gfitchosg_op2'));
			gostopg = 1;
		else
			gostopg = 0;
			gostopmain = 0;
		end
	elseif strcmp(deblank(c),'u')		% increment order
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		op = getString(message('Robust:obsolete:gfitchosg_op1'));
		gostopg = 1;
		if fitstatg(pp) == 1
			if irow==icol
				ord = curord + 2;
				ord = 2*floor(ord/2);
			else
				ord = curord + 1;
			end
		else
			ord = 0;
		end
		m1 = getString(message('Robust:obsolete:gfitchosg_m1_8',ord));
		m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
	elseif strcmp(deblank(c),'d')		% decrement order
		pp = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
		if curord > 0 && fitstatg(pp) == 1
			if irow==icol
				ord = max([(curord-2) 0]);
				ord = 2*floor(ord/2);
			else
				ord = curord - 1; 
			end
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_9',ord));
			op = getString(message('Robust:obsolete:gfitchosg_op1'));
			gostopg = 1;
		else
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_10'));
            
			op = getString(message('Robust:obsolete:gfitchosg_op2'));
			gostopg = 1;
		end
		m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
	else
		dbc = deblank(c);
		if length(dbc) >2
			if strcmp(dbc(1:2),'mx')
				num = str2num(dbc(3:length(dbc)));
				if floor(num)==ceil(num) && num >= 0
					maxord = num;
					op = getString(message('Robust:obsolete:gfitchosg_op3'));
					m1 = getString(message('Robust:obsolete:gfitchosg_m1_11'));
					m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
					gostopg = 1;
				else
					m1 = getString(message('Robust:obsolete:gfitchosg_m1_12'));
					op = getString(message('Robust:obsolete:gfitchosg_op2'));
					gostopg = 1;
				end
			elseif strcmp(dbc(1:2),'at')
				num = str2num(dbc(3:length(dbc)));
				if num > 1
					perctol = num;
					op = getString(message('Robust:obsolete:gfitchosg_op3'));
					m1 = getString(message('Robust:obsolete:gfitchosg_m1_13'));
					m2 = getString(message('Robust:obsolete:gfitchosg_m2'));
					gostopg = 1;
				else
					m1 = getString(message('Robust:obsolete:gfitchosg_m1_14'));
					op = getString(message('Robust:obsolete:gfitchosg_op2'));
					gostopg = 1;
				end
			else
				m1 = getString(message('Robust:obsolete:gfitchosg_m1_15'));
				op = getString(message('Robust:obsolete:gfitchosg_op2'));
				gostopg = 1;
			end
		else
			m1 = getString(message('Robust:obsolete:gfitchosg_m1_15'));
			op = getString(message('Robust:obsolete:gfitchosg_op2'));
			gostopg = 1;
		end
	end
