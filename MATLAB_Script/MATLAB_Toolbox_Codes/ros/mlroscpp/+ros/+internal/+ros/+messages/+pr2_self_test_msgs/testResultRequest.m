function [data, info] = testResultRequest
%TestResult gives an empty data for pr2_self_test_msgs/TestResultRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestResultRequest';
[data.RESULTPASS, info.RESULTPASS] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.RESULTFAIL, info.RESULTFAIL] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.RESULTHUMANREQUIRED, info.RESULTHUMANREQUIRED] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.HtmlResult, info.HtmlResult] = ros.internal.ros.messages.ros.char('string',0);
[data.TextSummary, info.TextSummary] = ros.internal.ros.messages.ros.char('string',0);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Plots, info.Plots] = ros.internal.ros.messages.pr2_self_test_msgs.plot;
info.Plots.MLdataType = 'struct';
info.Plots.MaxLen = NaN;
info.Plots.MinLen = 0;
data.Plots = data.Plots([],1);
[data.Params, info.Params] = ros.internal.ros.messages.pr2_self_test_msgs.testParam;
info.Params.MLdataType = 'struct';
info.Params.MaxLen = NaN;
info.Params.MinLen = 0;
data.Params = data.Params([],1);
[data.Values, info.Values] = ros.internal.ros.messages.pr2_self_test_msgs.testValue;
info.Values.MLdataType = 'struct';
info.Values.MaxLen = NaN;
info.Values.MinLen = 0;
data.Values = data.Values([],1);
info.MessageType = 'pr2_self_test_msgs/TestResultRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'RESULT_PASS';
info.MatPath{2} = 'RESULT_FAIL';
info.MatPath{3} = 'RESULT_HUMAN_REQUIRED';
info.MatPath{4} = 'html_result';
info.MatPath{5} = 'text_summary';
info.MatPath{6} = 'result';
info.MatPath{7} = 'plots';
info.MatPath{8} = 'plots.title';
info.MatPath{9} = 'plots.image';
info.MatPath{10} = 'plots.image_format';
info.MatPath{11} = 'params';
info.MatPath{12} = 'params.key';
info.MatPath{13} = 'params.value';
info.MatPath{14} = 'values';
info.MatPath{15} = 'values.key';
info.MatPath{16} = 'values.value';
info.MatPath{17} = 'values.min';
info.MatPath{18} = 'values.max';