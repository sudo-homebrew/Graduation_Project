classdef (Abstract) Model < robotics.manip.internal.InternalAccess
    %This function is for internal use only. It may be removed in the future.
    
    %MODEL
    
    %   Copyright 2021 The MathWorks, Inc.

    events
        ModelBecameDirty
    end

    methods (Abstract)
        
        %initialize this model for a new session
        initialize(obj);

    end

    methods (Access = protected)
        function notifyModelBecameDirty(obj)
            %notifyModelBecameDirty Notify that model is dirtied
            %   This method is triggered when the model values change in a
            %   way that isn't purely related to the views. Changes that
            %   are only observed in the view are ignored since these are
            %   purely cosmetic states that are not preserved between
            %   sessions.
            notify(obj, "ModelBecameDirty");
        end
    end
end
