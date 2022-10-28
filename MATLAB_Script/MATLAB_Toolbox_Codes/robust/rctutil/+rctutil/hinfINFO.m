classdef hinfINFO
   % Pseudo-structure for HINFSYN output INFO.
   
   %   Copyright 2018 The MathWorks, Inc.
   properties
      % GAMMA level used to compute K,X,Y
      gamma
      % Riccati solution Xoo
      X
      % Riccati solution Yoo
      Y
      % Gains of observer form of controller K
      Ku
      Kw
      Lx
      Lu
      % Regularized plant
      Preg
      % Parameterization of all GAMMA-suboptimal controllers
      AS
   end
   
   % Deprecated, here for backward compatibility
   properties (Dependent, Hidden)
      % Full information gain matrix KFI
      KFI
      % Full control gain matrix KFC
      KFC
      % H-infinity cost for full information KFI
      GAMFI
      % H-infinity cost for full control KFC
      GAMFC
   end
         
   properties (Access=protected)
      SYNDATA_
   end
   
   methods
      
      function this = hinfINFO(S)
         % Constructor
         this.SYNDATA_ = S;
      end
      
      function V = get.KFI(this)
         % On-demand GET method for KFI
         V = localComputeFI(this.SYNDATA_,this.gamma);
      end

      function V = get.GAMFI(this)
         % On-demand GET method for GAMFI
         [~,V] = localComputeFI(this.SYNDATA_,this.gamma);
      end
      
      function V = get.KFC(this)
         % On-demand GET method for KFC
         V = localComputeFC(this.SYNDATA_,this.gamma);
      end

      function V = get.GAMFC(this)
         % On-demand GET method for GAMFC
        [~,V] = localComputeFC(this.SYNDATA_,this.gamma);
      end
            
   end
   
end


function [KFI,GAMFI] = localComputeFI(S,GAM)
% Populates KFI and GAMFI fields
P = S.P;
nE = size(P,1)-S.nY;
[KFI,~,GAMFI] = hinffi(P(1:nE,:),S.nU,GAM);
end

function [KFC,GAMFC] = localComputeFC(S,GAM)
% Populates KFC and GAMFC fields
P = S.P;
nD = size(P,2)-S.nU;
[KFC,~,GAMFC] = hinffc(P(:,1:nD),S.nY,GAM);
end
