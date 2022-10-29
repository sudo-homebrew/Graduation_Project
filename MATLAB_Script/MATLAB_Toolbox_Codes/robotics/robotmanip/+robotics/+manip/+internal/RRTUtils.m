classdef RRTUtils
    %This class is for internal use only, and it maybe removed in the future.
    %RRTUtils Utilities for the RRT algorithm
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    methods(Static)        
        
        function [treeB, treeA] = swap(treeA, treeB)
            %swap Swaps the two input trees
        end
        
        function interpolatedStates = interpolateByResolution(path, stateSpace, resolution)
            %interpolateByResolution Interpolate between adjacent states in the path
            %   INTERPOLATEDSTATES = interpolateByResolution(PATH, STATESPACE, RESOLUTION)
            %   interpolates between adjacent states in the path at a specified
            %   distance resolution. PATH is a R-by-N matrix where R denotes the
            %   number of states in the path and N is the dimension of the
            %   STATESPACE.
            
            interpolatedStates = [];
            if(isempty(path))
                return;
            end
            for i = 1 : size(path, 1) -1
                dist = stateSpace.distance(path(i + 1, :), path(i, :));
                if(dist == 0)
                    interpolatedStates = [interpolatedStates; path(i, :)];
                    continue
                end
                ratios = (0:resolution:dist)/dist;
                interpStates = stateSpace.interpolate(path(i, :), path(i + 1, :), ratios);
                interpolatedStates = [interpolatedStates; interpStates(1:end-1, :)];
            end
            
            %Insert the last state.
            interpolatedStates = [interpolatedStates; path(end, :)];
        end
        
        function interpolatedStates = interpolateByNumber(path, stateSpace, numInterpolations)
            %interpolateByNumber Interpolate between adjacent states in the path
            %   INTERPOLATEDSTATES = interpolateByNumber(PATH, STATESPACE, NUMINTERPOLATIONS)
            %   inserts NUMINTERPOLATIONS number of interpolations  between
            %   adjacent states in the path. PATH is a R-by-N matrix where R
            %   denotes the number of states in the path and N is the dimension
            %   of the STATESPACE.
            
            interpolatedStates = [];
            if(isempty(path))
                return;
            end
            for i = 1 : size(path, 1) -1
                
                %Find the ratios given that numInterpolations are excluding the
                %states, themselves.
                ratios = linspace(0, 1, numInterpolations + 2);
                
                interpStates = stateSpace.interpolate(path(i, :), path(i + 1, :), ratios);
                interpolatedStates = [interpolatedStates; interpStates(1:end-1, :)];
            end
            
            %Insert the last state.
            interpolatedStates = [interpolatedStates; path(end, :)];
        end
        
        function shortenedPath = shorten(path, stateValidator, numIterations)
            %shorten Shortens the input path
            %   The shorten is a randomized shortening algorithm repeated for
            %   numIterations. With each iteration two edges in the path are
            %   selected. Next, a connection is tried to be made between one
            %   intermediate configuration each on the two randomly selected
            %   edges. A successful connection is made if the motion along that
            %   connection is valid.

            shortenedPath = path;
            if(isempty(path))
                return;
            end
            coder.varsize('shortenedPath')
            for iter = 1 : numIterations
                pathLength = size(shortenedPath, 1);

                %If there are only 3 nodes in the path, then there is no point
                %in shortening this path.
                if(pathLength < 3)
                    return;
                end

                %Select two nodes on the path except the last node.
                v = randi(pathLength - 1, [1, 2]);
                v1 = v(1); 
                v2 = v(2); 

                %If v2 doesn't succeed v1, then swap them, else if they
                %coincide, then skip this iteration
                if(v2 < v1)
                    [v1, v2] = robotics.manip.internal.RRTUtils.swap(v1, v2);
                elseif(v2 == v1)
                    continue;
                end

                %An edge is formed between a randomly selected vertex, and the
                %vertex following it. Form an edge for each of the randomly
                %selected vertices in the previous step.
                v1Prime = v1 + 1;
                v2Prime = v2 + 1;

                %Obtain the corresponding configuration of the vertices. This
                %will be used to obtain the intermediate configuration on the
                %edges.
                nodeV1 = shortenedPath(v1, :); nodeV1Prime = shortenedPath(v1Prime, :);
                nodeV2 = shortenedPath(v2, :); nodeV2Prime = shortenedPath(v2Prime, :);

                %Obtain the intermediate configurations
                nodeOnV1_V1Prime = stateValidator.StateSpace.interpolate(nodeV1, nodeV1Prime, rand);
                nodeOnV2_V2Prime = stateValidator.StateSpace.interpolate(nodeV2, nodeV2Prime, rand);
                
                %Validate the edge given the configurations
                if(stateValidator.isMotionValid(nodeOnV1_V1Prime, nodeOnV2_V2Prime))

                    %Newly validated edge, and the distance along the edge
                    newEdge = [nodeOnV1_V1Prime;
                        nodeOnV2_V2Prime;];
                    distEdge = ...
                        stateValidator.StateSpace.distance(nodeOnV1_V1Prime, nodeOnV2_V2Prime);
                    
                    %Before this edge is inserted, check if the distance
                    %along the path will be shortened. 
                    
                    %Compute distance between nodes (along the path) on v1' and v2.
                    distV1Prime_V2 = ...
                        robotics.manip.internal.RRTUtils.computePathDistance(...
                        shortenedPath, v1Prime, v2, stateValidator.StateSpace);

                    %Compute distance between the nodes on edge v1_v1', and v1'
                    distnodeOnV1_V1Prime_v1Prime = ...
                        stateValidator.StateSpace.distance(nodeOnV1_V1Prime, nodeV1Prime);

                    %Compute distance between the nodes on v2, and edge v2_v2'
                    distV2_nodeOnV2_V2Prime = ...
                        stateValidator.StateSpace.distance(nodeV2, nodeOnV2_V2Prime);

                    if(distEdge < ...
                            distnodeOnV1_V1Prime_v1Prime + distV1Prime_V2 + distV2_nodeOnV2_V2Prime)

                        %Insert the edge in the path and trim the intermediate edges
                        shortenedPath = [shortenedPath(1:v1, :);
                            newEdge;
                            shortenedPath(v2Prime:end, :)];
                    end
                end
            end
        end

        function dist = computePathDistance(path, v1, v2, stateSpace)
            %computePathDistance Computes the distance between two input indices on the path
            %   The function assumes that v1 and v2 are valid indices, and
            %   that v2 is greater than or equal to v1
            pathLength = size(path, 1);
            dist = 0;
            if(pathLength < 2)
                return;
            end
            for i = v1 : v2-1
                distBetweenAdjacentStates = ...
                    stateSpace.distance(path(i, :), path(i + 1, :));
                dist = dist + distBetweenAdjacentStates;
            end
        end
    end
end
