% Robust Control Toolbox
% Version 6.11.1 (R2022a) 13-Nov-2021
% 
% Uncertainty modeling.
%   ureal       - Uncertain real parameter.
%   ucomplex    - Uncertain complex parameter.
%   ucomplexm   - Uncertain complex matrix.
%   ultidyn     - Uncertain linear time-invariant object.
%   umargin     - Uncertain gain/phase in feedback loop.
%   udyn        - Uncertain dynamics.
%   umat        - Uncertain matrix.
%   uss         - Uncertain state-space model.
%   ufrd        - Uncertain Frequency Response Data model.
%   randatom    - Create a random uncertain element.
%   randumat    - Create a random uncertain matrix.
%   randuss     - Create a random uncertain state-space model.
%   ucover      - Fit an uncertain model to a set of LTI responses.
%
% Manipulation of uncertain models.
%   isUncertain - True for uncertain models.
%   getNominal  - Nominal value of uncertain model.
%   uscale      - Scale uncertainty in normalized units.
%   actual2normalized - Transform actual values to normalized values.
%   normalized2actual - Transform normalized values to actual values.
%   normalize   - Normalize uncertain element.
%   simplify    - Simplify representation of an uncertain object.
%   usubs       - Substitute values for uncertain elements.
%   usample     - Generate random samples of an uncertain object.
%   gridureal   - Grid real parameters over their range.
%   lftdata     - Extract LFT data from uncertain object.
%   ssbal       - Diagonal state/uncertainty scaling for an uncertain system.
%
% Model order reduction.
%   ncfmr       - Normalized coprime factor model reduction.
%   bstmr       - Balanced stochastic truncation model reduction.
%   modreal     - State-space modal truncation/realization.
%   imp2ss      - Impulse response to state-space approximation.
%   
% Robustness and worst-case analysis.
%   loopsens    - Sensitivity functions of feedback loop.
%   diskmargin  - Disk-based stability margins of feedback loop.
%   robstab     - Robust stability of uncertain system.
%   robgain     - Robust performance of uncertain system.
%   wcnorm      - Worst-case norm of uncertain matrix.  
%   wcgain      - Worst-case gain of uncertain system.
%   wcdiskmargin - Worst-case disk margins of uncertain feedback loop.
%   diskmarginplot   - Graphical disk margin analysis. 
%   wcsigmaplot      - Worst-case singular value plot.
%   wcdiskmarginplot - Graphical worst-case disk margin analysis. 
%   mussv       - Bounds on the structured singular value (mu).
%   mussvextract- Extracts data from MUSSV output structure.
%   ncfmargin   - Normalized coprime stability margin of feedback loop.
%   gapmetric   - Gap and Vinnicombe gap metrics.
%   lncf,rncf   - Left/right normalized coprime factorizations.
%   popov       - Robust stability with Popov criterion.
%
% Loop shaping.
%   loopsyn     - H-infinity loop shaping design.
%   mixsyn      - H-infinity mixed-sensitivity design.
%   ncfsyn      - Glover-McFarlane loop shaping design.
%   makeweight  - Construct frequency-weighting function.
%   augw        - Augmented plant for mixed-sensitivity design.
%
% Controller synthesis.
%   h2syn       - H2 controller synthesis.
%   hinfsyn     - H-infinity controller synthesis.
%   hinfstruct  - H-infinity tuning of fixed-structure controllers
%   h2hinfsyn   - LMI-based H2/H-infinity controller synthesis.
%   ltrsyn      - Loop-transfer recovery controller synthesis.
%   makeweight  - Construct frequency-weighting function.
%   mkfilter    - Construct low-pass filter.
%
% Robust controller synthesis.
%   musyn       - Mu-synthesis of robust controllers.
%   musynperf   - Robust H-infinity performance optimized by musyn.
%
% Sampled-data systems.
%   sdhinfnorm  - Induced L2 norm of a sample-data system.
%   sdlsim      - Time response of sampled-data feedback system.
%   sdhinfsyn   - H-infinity synthesis of sampled-data controllers.
% 
% Gain scheduling.
%   hinfgs      - Design gain-scheduled H-infinity controllers.
%  
% LMI solvers.
%   Type "help rctlmi" for details.
%
% Simulink
%   RCTblocks   - Simulink blocks for uncertainty modeling and analysis.
%   ufind       - Find all uncertain variables in a Simulink model.
%   usample     - Generate random samples for uncertain variables.
%   ulinearize  - Linearize a Simulink model with Uncertain State Space blocks.
%
% Demonstrations.
%   Type "demo toolbox robust" for a list of available demos.

%   Copyright 1986-2021 The MathWorks, Inc.
