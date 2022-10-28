function qosProfile = setQOSProfile(rmwProfile, qosHist, qosDepth, qosReliability, qosDurability)
% SETQOSPROFILE Set the QoS profile values as specified
% This method uses the enumerations for history, durability and
% reliability values as specified in 'rmw/types.h' header.

%   Copyright 2021 The MathWorks, Inc.
%#codegen
coder.cinclude('mlros2_qos.h');
opaqueHeader = {'HeaderFile', 'rmw/types.h'};
if isequal(coder.internal.toLower(qosHist), 'keepall')
    history = coder.opaque('rmw_qos_history_policy_t', ...
        'RMW_QOS_POLICY_HISTORY_KEEP_ALL', opaqueHeader{:});
    qosDepth = double(intmax('int32'));
else
    history = coder.opaque('rmw_qos_history_policy_t', ...
        'RMW_QOS_POLICY_HISTORY_KEEP_LAST', opaqueHeader{:});
end
if isequal(coder.internal.toLower(qosReliability), 'reliable')
    reliability = coder.opaque('rmw_qos_reliability_policy_t', ...
        'RMW_QOS_POLICY_RELIABILITY_RELIABLE', opaqueHeader{:});
else
    reliability = coder.opaque('rmw_qos_reliability_policy_t', ...
        'RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT', opaqueHeader{:});
end
if isequal(coder.internal.toLower(qosDurability), 'transientlocal')
    durability = coder.opaque('rmw_qos_durability_policy_t', ...
        'RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL', opaqueHeader{:});
else
    durability = coder.opaque('rmw_qos_durability_policy_t', ...
        'RMW_QOS_POLICY_DURABILITY_VOLATILE', opaqueHeader{:});
end

depth = cast(qosDepth,'like',coder.opaque('size_t','0'));

% Use SET_QOS_VALUES macro to set the
% structure members of rmw_qos_profile_t structure, The macro takes in
% the rmw_qos_profile_t structure and assigns the history, depth,
% durability and reliability values specified on system block to the
% qos_profile variable in generated C++ code.
if coder.target('Rtw')
    coder.ceval('SET_QOS_VALUES', rmwProfile, history, depth, ...
        durability, reliability);
end
qosProfile = rmwProfile;
end

% LocalWords:  rmw qos keepall
