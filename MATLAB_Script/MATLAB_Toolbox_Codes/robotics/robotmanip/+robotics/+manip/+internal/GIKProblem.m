classdef GIKProblem < robotics.manip.internal.NonlinearLeastSquaresProblem & ...
                      robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    %#codegen
    
    %IKProblem
    
    properties (Constant, Access = private)
        Epsilon = eps;
    end
    
    properties (Dependent)
        
        %WeightMatrix
        WeightMatrix
        
        %EnforceJointLimits
        EnforceJointLimits
        
        %KinematicPath
        KinematicPath
    
    end
    
    properties (SetAccess = immutable)
        
        %Tree Instance of internal.RigidBodyTree
        Tree
        
        %Constraints
        Constraints
        
        %ConstraintTypeList
        ConstraintTypeList
        
        %NumConstraintObjects Number of kinematic constraints in the problem
        NumConstraintObjects
        
        %NumResiduals Number of elements in the residual vector
        NumResiduals
        
        %NumSlacks Number of slack variables for this problem
        %   In the current implementation, the number of slack variables is
        %   always equal to the number of residual elements. Conceptually,
        %   however, they are different quantities.
        NumSlacks
        
        %NumPositions Number of position variables for this problem
        NumPositions
        
        %NumVariables Total number of variables for this problem
        NumVariables
        
        %ResidualIndices Indices of each constraint's terms in the residual
        %vector. 
        ResidualIndices
        
        %SlackIndices Indices of each constraint's slack variables in the
        %decision variable vector
        SlackIndices
        
    end
    
    properties (Access = private)
        
        %EnforceJointLimitsInternal
        EnforceJointLimitsInternal

        %EqualityFlags True for constraint elements that have equal bounds
        EqualityFlags

        %LastX Stored input value from last call to the residuals method
        LastX

        %LastF Stored residual value from last call to the residuals method
        LastF

        %LastJ Stored Jacobian value from last call to the residuals method
        LastJ
        
    end
    
    methods
        
        function obj = GIKProblem(tree, constraintTypeList)
            %GIKProblem Constructs a GIKProblemObject. 
            obj.Tree = tree;
            obj.NumPositions = obj.Tree.PositionNumber;
            obj.NumConstraintObjects = numel(constraintTypeList);
            obj.Constraints = cell(1, obj.NumConstraintObjects);
            obj.ConstraintTypeList = constraintTypeList;
            residualIndices = cell(1, obj.NumConstraintObjects);
            slackIndices = cell(1, obj.NumConstraintObjects);
            equalityFlags = cell(1, obj.NumConstraintObjects);
            numResidualsTotal = 0;
            for i = coder.unroll(1:obj.NumConstraintObjects)
                switch constraintTypeList{i}
                    case 'orientation'
                        obj.Constraints{i} = robotics.manip.internal.OrientationTarget(obj.Tree);
                    case 'position'
                        obj.Constraints{i} = robotics.manip.internal.PositionTarget(obj.Tree);
                    case 'pose'
                        obj.Constraints{i} = robotics.manip.internal.PoseTarget(obj.Tree);
                    case 'cartesian'
                        obj.Constraints{i} = robotics.manip.internal.CartesianBounds(obj.Tree);
                    case 'aiming'
                        obj.Constraints{i} = robotics.manip.internal.AimingConstraint(obj.Tree);
                    case 'distance'
                        obj.Constraints{i} = ...
                            robotics.manip.internal.DistanceBoundsConstraint(obj.Tree);
                    case 'joint'
                        obj.Constraints{i} = robotics.manip.internal.JointPositionBounds(obj.Tree);
                    case 'jointbounds'
                        obj.Constraints{i} = robotics.manip.internal.JointPositionBounds(obj.Tree);
                    case 'prismaticjoint'
                        obj.Constraints{i} = robotics.manip.internal.PrismaticJointConstraint(obj.Tree);
                    case 'revolutejoint'
                        obj.Constraints{i} = robotics.manip.internal.RevoluteJointConstraint(obj.Tree);
                    case 'fixedjoint'
                        obj.Constraints{i} = robotics.manip.internal.FixedJointConstraint(obj.Tree);
                    otherwise
                        robotics.manip.internal.error('generalizedinversekinematics:ConstraintTypeInvalid',constraintTypeList{i});
                end
                numResiduals = obj.Constraints{i}.NumElements;
                residualIndices{i} = numResidualsTotal + (1:numResiduals);
                slackIndices{i} = obj.NumPositions + residualIndices{i};
                equalityFlags{i} = false(1, numResiduals);
                numResidualsTotal = numResidualsTotal + numResiduals;
            end
            obj.ResidualIndices = residualIndices;
            obj.SlackIndices = slackIndices;
            obj.EqualityFlags = equalityFlags;
            obj.NumResiduals = numResidualsTotal;
            obj.NumSlacks = numResidualsTotal;
            obj.NumVariables = obj.NumPositions + obj.NumSlacks;
            obj.DesignVariableBounds = [-Inf(obj.NumVariables,1), ...
                                          Inf(obj.NumVariables,1)];
            obj.EnforceJointLimits = true;
            obj.updateDesignVariableBounds();
            obj.initializeStoredValues();
        end
        
        function newobj = copy(obj)
            newobj = robotics.manip.internal.GIKProblem(obj.Tree, obj.ConstraintTypeList);
            newobj.EnforceJointLimits = obj.EnforceJointLimits;
            update(newobj, obj.Constraints{:});
            newobj.LastX = obj.LastX;
            newobj.LastF = obj.LastF;
            newobj.LastJ = obj.LastJ;
        end
        
        function update(obj, varargin)
            %update Update parameters of each constraint
            for i = 1:obj.NumConstraintObjects
                obj.Constraints{i}.update(varargin{i});
            end
            obj.updateDesignVariableBounds();
            obj.initializeStoredValues();
        end
        
        function [f, J] = residuals(obj, x)
            %residuals Concatenate residuals and Jacobians for all constraints
            if obj.isNewX(x)
                % Since x has changed since the last time this method was
                % called, we need to compute new values for f and J.
                [f, J] = obj.residualsInternal(x);
                
                % Update the stored values
                obj.LastX = x;
                obj.LastF = f;
                obj.LastJ = J;
            else
                % In this case, x has not changed since the last time this
                % method was called, and we can return the stored values.
                f = obj.LastF;
                J = obj.LastJ;
            end
        end
        
        function rc = randomSeed(obj)
            %randomSeed
            qr = randomJointPositions(obj.Tree);
            rc = [qr; obj.defaultSlackValues(qr)];
        end
        
        function s = defaultSlackValues(obj, q)
            %defaultSlackValues Generate a set of feasible slack values
            %   Slack values are chosen to be as close as possible to the
            %   ones that would make the residuals zero for joint
            %   positions, q, while remaining within the bounds.
            slackBounds = obj.DesignVariableBounds(obj.NumPositions+1:end,:);
            % Compute the residual vector, s, with the slack vector set to
            % zero.
            s = obj.residuals([q;zeros(obj.NumSlacks,1)]);
            % Setting the slack vector to s would give zero residuals (the
            % problem would be solved), so leave s unchanged where
            % possible, and coerce it to lie within the bounds where
            % necessary.
            s = min(slackBounds(:,2), max(slackBounds(:,1), s));
        end
        
        function violations = constraintViolations(obj,x)
            %checkConstraints Returns a struct of constraint violations
            numConstraints = obj.NumConstraintObjects;
            violationsCell = cell(1,numConstraints);
            q = x(1:obj.NumPositions);
            for i = 1:numConstraints
                violationsCell{i} = obj.Constraints{i}.violation(q);
            end
            violations = struct('Type', obj.ConstraintTypeList, ...
                                'Violation', violationsCell);
        end
        
        function value = get.KinematicPath(obj)
            coder.varsize('value', ...
                [1, obj.NumConstraintObjects*(obj.Tree.MaxNumBodies+1)], [0, 1]);
            value = [];
            for i = 1:obj.NumConstraintObjects
                value = [value, obj.Constraints{i}.KinematicPath]; %#ok<AGROW>
            end
            value = fliplr(unique(sort(value)));
        end
        
        function value = get.EnforceJointLimits(obj)
            value = obj.EnforceJointLimitsInternal;
        end
        
        function set.EnforceJointLimits(obj, value)
            obj.EnforceJointLimitsInternal = value;
            if obj.EnforceJointLimitsInternal
                obj.DesignVariableBounds(1:obj.NumPositions,:) = ...
                    obj.Tree.JointPositionLimits;
            else
                obj.DesignVariableBounds(1:obj.NumPositions,:) = ...
                    [-Inf(obj.NumPositions,1), Inf(obj.NumPositions,1)];
            end
        end
        
        function value = get.WeightMatrix(obj)
            value = eye(obj.NumResiduals);
            for i = 1:obj.NumConstraintObjects
                resIdx = obj.ResidualIndices{i};
                value(resIdx, resIdx) = diag(obj.Constraints{i}.Weights);
            end
        end
        
    end
    
    methods (Access = private)
        
        function updateDesignVariableBounds(obj)
            %updateDesignVariableBounds Get current bounds from constraints
            %   This function also adds a small amount of "slop" to the
            %   equality constraints to avoid having linearly dependent
            %   constraints active simultaneously.
            for i = 1:obj.NumConstraintObjects
                bounds = obj.Constraints{i}.BoundsInternal;
                equalityFlags = (abs(diff(bounds,1,2)) < obj.Epsilon);
                bounds(equalityFlags,1) = -Inf;
                bounds(equalityFlags,2) = Inf;
                obj.DesignVariableBounds(obj.SlackIndices{i},:) = bounds;
                obj.EqualityFlags{i} = equalityFlags;
            end
        end
        
        function [f, J] = residualsInternal(obj, x)
            %residualsInternal Concatenate residuals and Jacobians for all constraints
            numConstraints = obj.NumConstraintObjects;
            gCell = cell(numConstraints,1); % Cell array of constraint values
            JCell = cell(numConstraints,1); % Cell array of constraint Jacobians
            q = x(1:obj.NumPositions);
            for i = 1:numConstraints
                [gCell{i}, JCell{i}] = evaluate(obj.Constraints{i},q);
            end
            f = zeros(obj.NumResiduals, 1); % Residual vector
            J = zeros(obj.NumResiduals, obj.NumVariables); % Jacobian of residual vector
            for i = 1:numConstraints
                rows = obj.ResidualIndices{i};
                cols = obj.SlackIndices{i};
                J(rows,1:obj.NumPositions) = JCell{i};
                slacks = x(cols);
                % Ignore slack variables associated with equality
                % constraints, since they must be equal to the bounds on
                % those constraints.
                J(sub2ind(size(J),rows,cols)) = -double(~obj.EqualityFlags{i});
                slacks(obj.EqualityFlags{i}) = ...
                    obj.Constraints{i}.BoundsInternal(obj.EqualityFlags{i},1);
                % Residuals are the difference between the constraint
                % values and the slacks.
                f(rows,:) = gCell{i} - slacks;
            end
        end
        
        function flag = isNewX(obj, x)
            %isNewX Return true if x is not the same as obj.LastX
            %   This method is equivalent to any(x ~= obj.LastX), but is
            %   better suited for code generation.
            flag = true;
            for i = 1:obj.NumVariables
                % If this element is different, then x is different from
                % obj.LastX and we don't need to keep checking.
                if x(i) ~= obj.LastX(i)
                    return;
                end
            end
            % If we made it through the loop, then x == obj.LastX
            flag = false;
        end
        
        function initializeStoredValues(obj)
            %initializeStoredValues Store a consistent set of values
            obj.LastX = zeros(obj.NumVariables, 1);
            [obj.LastF, obj.LastJ] = obj.residualsInternal(obj.LastX);
        end
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % In codegen, IK solver can only be specified once
            props = {'NumConstraintObjects', 'ConstraintTypeList'};
        end
    end
end
