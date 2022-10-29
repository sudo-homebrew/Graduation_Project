function info = computeMsgStructReordering(adjustedPropNames, ...
                                           varlenArrays, varlenArrayInfoProps)
    %This function is for internal use only. It may be removed in the future.

    %  computeMsgStructReordering(...) computes how the fields of a
    %  MATLAB ROS message struct should be reordered to make it compatible with
    %  Simulink bus struct. This function is intended to be a helper for
    %  sim.createROSMsgInfoMap, and the inputs are values calculated in
    %  sim.createROSMsgInfoMap:processBus
    %
    %  Details:
    %
    %   When a ROS message struct (result of toStruct) is converted to a
    %   Simulink Bus,  fields will be added for variable-length array info.
    %   These have to be adjacent to the corresponding variable-length array
    %   (this ordering is defined in the Simulink bus object, and the bus
    %   struct passed to Simulink is required to have same ordering of fields).
    %
    %   This function calculates a single-shot permutation of the message
    %   struct that will apply the reordering + deletions at one go, using
    %   STRUCT2CELL and CELL2STRUCT.
    %
    %  INFO is a struct with the following fields:
    %
    %   doPermute : a flag to indicate whether a permutation is needed at all
    %   augmentedPropertyList : the list of fields (needed when calling  CELL2STRUCT)
    %   permutation : the permutation of the fields

    %   Copyright 2014-2018 The MathWorks, Inc.

    assert(numel(varlenArrays) == numel(varlenArrayInfoProps));

    % Append fields for variable-length arrays (i.e., the *_SL_Info fields)
    augmentedFldList = [adjustedPropNames(:) ; varlenArrayInfoProps(:)];

    % At this point, augmentedFldList has the order in which fields would
    % appear in the "raw" bus struct. Now figure out how to reorder the fields
    % so as to be compatible with the Simulink bus

    fldOrdering = [];
    for idx=1:numel(augmentedFldList)
        fldname = augmentedFldList{idx};

        switch fldname
          case varlenArrays
            % find the index for the corresponding varlenArrayInfoProp
            idxInVarlenArrays = strcmp(fldname, varlenArrays);
            idxOfInfoProp = find(strcmp(varlenArrayInfoProps{idxInVarlenArrays}, augmentedFldList));
            % put the InfoProp right after the corresponding item
            fldOrdering(end+1:end+2) = [idx idxOfInfoProp];
          case varlenArrayInfoProps
            % omit idx from indices (will be picked up via the
            % varlenArrays case)
          otherwise
            fldOrdering(end+1) = idx; %#ok<AGROW>
        end
    end

    % There shouldn't be any repetitions in fldOrdering
    assert(numel(fldOrdering) == length(unique(fldOrdering)));

    info.augmentedPropertyList = augmentedFldList;
    info.permutation = fldOrdering;
    info.doPermute = numel(varlenArrays) > 0;

end
