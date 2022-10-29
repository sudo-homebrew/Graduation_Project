classdef OSPABuiltins
    % This is an internal clas and may be removed or modified in a future
    % release.
    
    % Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    
    methods
        function dFcn = getDistanceFcn(~, motionModel, distance)
            % Return a function handle to compute the distance between
            % track and truth as a function of motion model and distance
            % type.
            switch motionModel
                case 'constvel'
                    switch distance
                        case 'posnees'
                            dFcn = @neesposcv;
                        case 'velnees'
                            dFcn = @neesvelcv;
                        case 'posabserr'
                            dFcn = @abserrposcv;
                        case 'velabserr'
                            dFcn = @abserrvelcv;
                    end
                case 'constacc'
                    switch distance
                        case 'posnees'
                            dFcn = @neesposca;
                        case 'velnees'
                            dFcn = @neesvelca;
                        case 'posabserr'
                            dFcn = @abserrposca;
                        case 'velabserr'
                            dFcn = @abserrvelca;
                    end
                case 'constturn'
                    switch distance
                        case 'posnees'
                            dFcn = @neesposct;
                        case 'velnees'
                            dFcn = @neesvelct;
                        case 'posabserr'
                            dFcn = @abserrposct;
                        case 'velabserr'
                            dFcn = @abserrvelct;
                    end
                case 'singer'
                    switch distance
                        case 'posnees'
                            dFcn = @neesposca;
                        case 'velnees'
                            dFcn = @neesvelca;
                        case 'posabserr'
                            dFcn = @abserrposca;
                        case 'velabserr'
                            dFcn = @abserrvelca;
                    end
            end
        end
       
        function truthFcn = getTruthIdentifier(~)
            truthFcn = @defaultTruthIdentifier;
        end
        
        function trackFcn = getTrackIdentifier(~)
            trackFcn = @defaultTrackIdentifier;
        end
    end
end
