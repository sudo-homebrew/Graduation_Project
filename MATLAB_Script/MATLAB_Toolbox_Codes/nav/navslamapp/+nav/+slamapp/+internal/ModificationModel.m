classdef ModificationModel < handle & ...
        matlab.mixin.SetGet & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %MODIFICATIONMODEL Backend for the modification tab

    % Copyright 2018 The MathWorks, Inc.

    properties
        %IsIncremental
        IsIncremental

        %UserFacingScanIdPair
        UserFacingScanIdPair
    end

    properties
        %X
        X

        %Y
        Y

        %Theta
        Theta

        %IsIgnored
        IsIgnored
    end

    properties (SetObservable)
        %Tentative
        Tentative

        %Mode
        Mode

        %XYLimit
        XYLimit

        %TabTitleName Current title for modification tab
        TabTitleName
    end

    events
    ModificationModel_SubscribeButtonDownCallback

    ModificationModel_RequestRefreshMapSlider
end


methods
    function obj = ModificationModel()
    %MODIFICATIONMODEL Constructor
        obj.XYLimit = 100;
    end

    function prepareModification(obj, isIncremental, isIgnored, relPose, xyLimit, scanIdPair)
    %prepareModification
        import robotics.appscore.internal.eventdata.VectorEventData

        obj.IsIncremental = isIncremental;
        obj.notify('ModificationModel_SubscribeButtonDownCallback', VectorEventData(obj.IsIncremental));
        pause(0.1);

        obj.IsIgnored = isIgnored;
        obj.X = relPose(1);
        obj.Y = relPose(2);
        obj.Theta = relPose(3);

        s.X = obj.X;
        s.Y = obj.Y;
        s.Theta = obj.Theta;
        s.IsIgnored = obj.IsIgnored;

        obj.Tentative = s;

        obj.Mode = 0; % linear
        obj.XYLimit = xyLimit;

        obj.UserFacingScanIdPair = scanIdPair;
    end

    function [infoStruct, extraInfoStruct] = saveProperties(obj)
    %saveProperties

    % basic properties
        infoStruct = saveProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj);

        % observable properties
        extraInfoStruct.Tentative = obj.Tentative;
        extraInfoStruct.Mode = obj.Mode;
        extraInfoStruct.XYLimit = obj.XYLimit;
        extraInfoStruct.TabTitleName = obj.TabTitleName;

    end

    function loadProperties(obj, infoStruct1, infoStruct2)
    %loadProperties
        loadProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj, infoStruct1);

        % load additional properties
        import robotics.appscore.internal.eventdata.VectorEventData
        obj.notify('ModificationModel_SubscribeButtonDownCallback', VectorEventData(obj.IsIncremental));
        pause(0.1);

        if ~isempty(infoStruct2.Tentative) % not all app sessions have gone through modification stage
            obj.Tentative = infoStruct2.Tentative;
            obj.Mode = infoStruct2.Mode;
            obj.XYLimit = infoStruct2.XYLimit; % for toolstrip spinners
            obj.TabTitleName = infoStruct2.TabTitleName;
        end
    end

end

methods % setters
    function set.Tentative(obj, val)
    %set.Tentative

        x = obj.checkXY(val.X);
        y = obj.checkXY(val.Y);
        th = obj.checkTheta(val.Theta);
        if ~isempty(x) && ~isempty(y) && ~isempty(th)
            obj.Tentative.X = x;
            obj.Tentative.Y = y;
            obj.Tentative.Theta = th;
            obj.Tentative.IsIgnored = val.IsIgnored;
        end
    end

end

methods
    function valOut = checkXY(obj, valIn)
    %checkXY
        valOut = [];
        if isscalar(valIn)
            if valIn > abs(obj.XYLimit)
                valIn = abs(obj.XYLimit);
            end
            if valIn < -abs(obj.XYLimit)
                valIn = -abs(obj.XYLimit);
            end
            valOut = valIn;
        end
    end

    function valOut = checkTheta(~, valIn)
    %checkTheta
        valOut = [];
        if isscalar(valIn)
            valOut = robotics.internal.wrapToPi(valIn);
        end
    end
end

end
