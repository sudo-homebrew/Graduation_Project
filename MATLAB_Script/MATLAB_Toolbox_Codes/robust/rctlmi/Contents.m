% Robust Control Toolbox - LMI Solvers.
% 
% Specification of systems of LMIs.
%   lmiedit   - Open the LMI Editor GUI.
%   setlmis   - Initialize the creation of a system of LMIs.
%   lmivar    - Create a new matrix-valued variable in LMI system.
%   lmiterm   - Add a term to a given LMI.
%   newlmi    - Add a new LMI to an LMI system.
%   getlmis   - Get the internal description of the LMI system.
%  
% LMI characteristics.
%   lmiinfo   - Get information about an existing system of LMIs.
%   lminbr    - Get the number of LMIs in an LMI system.
%   matnbr    - Get the number of matrix variables in an LMI system.
%   decnbr    - Get the number of decision variables in LMI system.
%   dec2mat   - Extract matrix variable value from vector of decision variables.
%   mat2dec   - Construct decision variables vector from matrix variable values.
%   decinfo   - Show how matrix variables depend on decision variables.
%  
% LMI solvers.
%   feasp     - Compute a solution to a given LMI system.
%   mincx     - Minimize a linear objective under LMI constraints.
%   gevp      - Solve generalized eigenvalue minimization problems.
%   defcx     - Specify c'x objectives for MINCX.
%  
% Validation of results.
%   evallmi   - Evaluate LMIs for given values of decision variables.
%   showlmi   - Return the left- and right-hand-side of evaluated LMIs.
%  
% Modification of systems of LMIs.
%   dellmi    - Delete an LMI from an LMI system.
%   delmvar   - Remove a matrix variable from an LMI system.
%   setmvar   - Instantiate an LMI matrix variable.
%
% Robustness analysis for parameter-dependent systems (P-system).
%   psys        - Specify a parameter-dependent system (P-system).
%   psinfo      - Query characteristics of a P-system.
%   ispsys      - True for parameter-dependent systems.
%   pvec        - Specify a vector of uncertain or time-varying parameters.
%   pvinfo      - Query characteristics of a parameter vector.
%   polydec     - Compute polytopic coordinates wrt. box corners.
%   uss         - Convert affine P-system to uncertain state-space model.
%   aff2pol     - Convert affine P-systems to polytopic representation.
%   quadstab    - Assess quadratic stability of parameter-dependent system.
%   quadperf    - Assess quadratic Hinf performance of P-systems.
%   pdlstab     - Test robust stability via parametric Lyapunov functions.
%   decay       - Compute quadratic decay rate.
%   pdsimul     - Simulate P-systems along parameter trajectories.

