function [pthOut, kinematicInfo, solnInfo] = optimizeTEB(pthIn, ...
    obstList, options)
  %TimedElasticBandCarGraph creates a Timed Elastic Band optimizer for Car-like robots.
  %   Timed Elastic Band algorithm takes an SE2 path as input and optimizes
  %   it to reduce travel time while trying it's best to adhere to 11 soft
  %   constraints.  Soft constraints may be violated depending if their
  %   weight is too low.
  %
  %   The Timed Elastic Band algorithm iterates to perform 3 operation
  %       1. Adjust number of path and deltaT nodes in the graph by
  %          interpolation or removal. This also adjusts the values of the
  %          neighbor nodes.
  %       2. Update the obstacle constraints for input path or optimized
  %          path in previous iteration
  %       3. Optimize the path using a Non-Linear solver.
  %
  %   TimedElasticBandCarGraph leverages graph based optimizer and uses
  %   Levenberg-Marquardt as solver. It's primary functions is to compute
  %   the errors for all the constraints and their jacobian. It also
  %   provides and interface to be passed to solver as function handles for
  %   error(column vector), cost(scalar, error' * weight * error),
  %   Jacobian(matrix), and gradient(column vector) computation.
  %
  % [1] C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram:
  %     Efficient trajectory optimization using a sparse model. Proc. IEEE
  %     European Conference on Mobile Robots, Spain, Barcelona, 2013, pp.
  %     138–143.
  % [2] C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory
  %     Optimization and Control for Car-Like Robots, IEEE/RSJ International
  %     Conference on Intelligent Robots and Systems (IROS), Vancouver, BC,
  %     Canada, Sept. 2017.

  %   Copyright 2021 The MathWorks, Inc.

  %#codegen

  % Graph to store data required for computation of errors, cost,
  % gradient, and hessian.
  tebGraph = nav.algs.internal.TimedElasticBandCarGraph(options, ...
    obstList);

  % Max number of iterations solver should iterate for
  solverItr = options.MaxSolverIteration;

  % Number of times outer TEB loop should run for to perform core
  % activities of adjusting path pose and travel time, build graph, and
  % call solver for optimizing the path.
  outerIter = options.NumIteration;

  % Max number for elements in deltaT. Beyond that interpolation doesn't
  % happen.
  maxSamples = options.MaxPathStates - 1;

  % Properties concerning addition and deletion of pose and deltaT nodes
  % Reference value of deltaT to be around
  refDeltaT = options.ReferenceDeltaTime;

  % 1. Configure ErrorDampedLevenbergMarquardt for TEB.
  solver = configureSolver(solverItr, tebGraph);
  % 2. Initialize deltaT for state vector X.
  deltaT = initializeDeltaT(pthIn, options.MaxVelocity);

  % 3. outer loop for TEB
  % 3.0 Initialize variables
  pth = pthIn;
  numIter = 0;
  % to resolve codegen error. Since, it's not able to deduce that outerIter
  % can't be 0, it says that x is not defined on all paths.
  x = [reshape(pthIn(2:end-1, :)', [], 1); deltaT];
  for itr=1:outerIter

    % 3.1 Adjust number of pose and deltaT, check each deltaT value with
    % refDeltaT. If the deltaT > 1.1*refDeltaT then interpolate path and deltaT, if
    % deltaT < 0.9*refDeltaT then remove the pose and deltaT to merge it with
    % next nodes.
    [pth, deltaT] = adjustPoseDeltaT(pth, deltaT, refDeltaT, maxSamples);

    % 3.2 (re)build the graph with the adjusted path. Mainly update the
    % meta-data like Xdim, size of W, most importantly obstacle edges with
    % the updated path.
    build(tebGraph, pth, 2^(itr-1));

    % 3.3 Convert path and deltaT to X for solver. Ignore first and last
    %     pose as they are fixed. For the movable poses, convert each pose
    %     into column vector (in pth each row is a pose) and then stack
    %     the poses vertically, i.e. N-by-3 => 3N-by-1, deltaT is already a column
    %     vector, stack it vertically to the transformed path vector.
    x0 = [reshape(pth(2:end-1, :)', [], 1); deltaT];

    % 3.4 Solve using Levenberg-Marquardt solver. Look at configureSolver
    % function for how it's setup.
    [x, si] = solver.solve(x0);
    % Note the number of solver iteration in this outer iteration. Add it
    % to the running sum to compute total number of solver iterations.
    numIter = numIter + si.Iterations;

    % 3.5 Convert optimized X into path and deltaT. Convert state vector used
    % by solver to path and deltaT nodes, it's easier to calculate error and
    % jacobian in these formats.
    pth = x2path(tebGraph, x);
    deltaT = x2DeltaT(tebGraph, x);
  end

  % 4. Assign second output if required
  % 4.1 Assign path
  pthOut = pth;
  cost = 0;
  if(nargout >= 2)
    % 4.2 Assign deltaT, since we want to output timestamps, do a cumulative
    % sum on it. Assign first time stamp as zero as that's where the time
    % starts.
    t = [0; cumsum(deltaT)];

    % 4.3 Solver doesn't output the debug info. Recompute cost to update
    % tebGraph properties values for final x.
    cost = nav.algs.internal.TimedElasticBandCarGraph.cost(x, tebGraph);
    linvel = tebGraph.Velocity(:,1);
    angvel = tebGraph.Velocity(:,2);

    kinematicInfo = struct('TimeStamps',t,'Velocity', linvel,...
      'AngularVelocity', angvel);
  end


  % 5. Assign 3rd output if required
  if (nargout >= 3)
    %  Extract error terms for the final x.
    [~, err] = nav.algs.internal.TimedElasticBandCarGraph.error(x, tebGraph);
    solnInfo = struct('Cost',cost,'NumIteration',numIter, 'Err', err);
  end
end

function [adjustedPth,adjustedDeltaT] = adjustPoseDeltaT(pth, deltaT, deltaTRef, ...
    maxSamples)
  %adjustPoseDeltaT adds or removes pose and corresponding deltaTs based on deltaTRef
  %
  %   Essentially it iterates over the path multiple times, in each
  %   iteration it checks if any deltaT (travel time between consecutive
  %   nodes) is out of bounds [refDeltaT-deltaTHysteresis refDeltaT+deltaTHysteresis].
  %   If deltaT is above the upper bound a new pose is inserted, if deltaT is
  %   below lower bound the pose is merged with the next nodes.

  % Readable local variables
  minSamples = 3; % Can't compute acceleration error with 2 nodes.
  deltaTHysteresis = 0.1*deltaTRef; % hysteresis is 10% of reference value to avoid fluctuations

  % iterate over the path multiple times to ensure all deltaTs are around the
  % reference deltaT. Exit the loop if no nodes were added or removed.
  % 100 is just to ensure that if the addition-removal
  % cycle is stuck in infinite loop then there is an exit condition. In
  % most scenarios it shouldn't reach 100, 10 might work in normal
  % scenarios, 100 should cover some corner cases.
  for itr = 1:100

    % modified is set to true if a pose was added or removed. If it doesn't
    % even happen once then we can exit.
    modified = false;
    % k is the row id for pose and deltaT in the vectors
    k = 1;
    numDeltaT = numel(deltaT);
    while k <= numDeltaT

      % Determine if we should add or remove the current pose and deltaT.
      shouldAdd = (deltaT(k) > (deltaTRef + deltaTHysteresis)) ...
        && (numDeltaT < maxSamples);
      shouldRemove = (deltaT(k) < (deltaTRef - deltaTHysteresis)) ...
        && (numDeltaT > minSamples);

      if shouldAdd
        % If a new node is to be added then average the current and next
        % node and insert the averaged (new) node between current and next.
        % We average as we want to insert node interpolated at 0.5
        newDeltaT = 0.5 * deltaT(k); % to average time divide it in half
        % average position is just added and divided by 2, for orientation
        % it's a bit more complicated, see averageAngle for that.
        newPose = averagePose(pth(k, :), pth(k + 1, :));
        % Insert new pose
        pth = [pth(1:k, :); newPose; pth((k + 1):end, :)];
        % Change the current deltaT, as time required to travel to next pose
        % has been cut down in half as well.
        deltaT(k) = newDeltaT;
        % Insert new deltaT at the same index as pose.
        deltaT = [deltaT(1:k); newDeltaT; deltaT((k + 1):end)];
        modified = true; % Indicate insertion happened in this loop.
      elseif shouldRemove
        % Determine if it's last deltaT, as there is no next node to merge into.
        lastDeltaT = (k == numDeltaT);

        if lastDeltaT
          % Just add the value to penultimate(previous) deltaT
          deltaT(k - 1) = deltaT(k - 1) + deltaT(k);
          % and remove the last nodes.
          pth(k, :) = [];
          deltaT(k) = [];
        else % for all other deltaT's
          % Add time to the next one
          deltaT(k + 1) = deltaT(k + 1) + deltaT(k);
          % and remove  the current one
          pth(k + 1, :) = [];
          deltaT(k) = [];
        end
        modified = true;% Indicate insertion happened in this loop.
      end

      % update values used for exit criteria in while
      k = k + 1;
      numDeltaT = numel(deltaT);

    end

    if ~modified
      % If the pth and deltaT were not modified in this iteration then exit
      % as there is no point in continuing.
      break;
    end

  end

  % Throw error if path has more states than MaxPathStates. This would happen
  % if input path had more poses than MaxPathStates and based on value of
  % ReferenceDeltaTime sufficient number of poses couldn't be removed.
  % Compare num of poses with maxSamples + 1, because MaxPathStates =
  % maxSamples + 1.
  coder.internal.errorIf(length(pth) > (maxSamples + 1), ...
    'nav:navalgs:optimizepath:PoseMoreThanMaxPathStates');

  adjustedPth = pth;
  adjustedDeltaT = deltaT;

end

function deltaT = initializeDeltaT(pth, velocity)
  % row-wise(dim=2, Third arg) 2-norm (Second arg) between the states.
  distances = vecnorm(diff(pth(:,1:2)), 2, 2);
  deltaT = distances ./ velocity;
end

function solver = configureSolver(solverItr, tebGraph)
  %configureSolver configures ErrorDampedLevenbergMarquardt solver for use with TEB.
  %
  %   Mainly assign the right function handles, max iterations, and disable
  %   random seeds + bound constraints

  % Initialize the solver as LevenbergMarquardt.
  solver = robotics.core.internal.ErrorDampedLevenbergMarquardt(false);

  % Retrieve default param values, some of the would be modified and
  % set in the solver at the end.
  params = solver.getSolverParams();

  % Function handles from the TimedElasticBandCarGraph
  solver.CostFcn = @nav.algs.internal.TimedElasticBandCarGraph.cost;
  solver.SolutionEvaluationFcn = ...
    @nav.algs.internal.TimedElasticBandCarGraph.error;
  solver.GradientFcn = ...
    @nav.algs.internal.TimedElasticBandCarGraph.gradient;
  solver.RandomSeedFcn = @(x)100; % For codegen
  solver.BoundHandlingFcn = @(x, ~)x; % For codegen

  params.UseErrorDamping = false;
  % Configured by user.
  params.MaxNumIteration = solverItr;

  % Since the constraints are treated as soft-constraints, solver needs
  % to behave as an unconstrained solver.
  params.ConstraintsOn = false;
  % No Randomness required as the input path is the initial X.
  params.RandomRestart = false;

  % Set the params configured above.
  solver.setSolverParams(params);

  % solver should pass the graph as extra arg to the cost function (static)
  % so that it can call private methods to compute err, Jacobian, etc..
  solver.ExtraArgs = tebGraph;

end

function avgPose = averagePose(pose1, pose2)
  %averagePose is a helper function to average two SE(2) poses.
  %   Pose1 and Pose2 should be row vectors of length 3.
  avgPose = zeros(1, 3);
  avgPose(1:2) = (pose1(1:2) + pose2(1:2)) / 2;
  avgPose(3) = averageAngle(pose1(3), pose2(3));
end

function avgAngle = averageAngle(theta1, theta2)
  %averageAngle is a helper function to compute average of two
  %angles(orientation)

  % Similar to what g2o does.
  x = cos(theta1) + cos(theta2);
  y = sin(theta1) + sin(theta2);

  avgAngle = atan2(y, x);

end
