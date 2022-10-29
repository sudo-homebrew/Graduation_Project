function ret = isCppPreserveClasses()
%ISCPPPRESERVECLASSES Return true if CppPreserveClasses config set option
%is set to true

%  Copyright 2021 The MathWorks, Inc.
%#codegen
    cfg = eml_option('CodegenBuildContext');
    if ~isempty(cfg)
        opt = coder.const(feval('getConfigProp',cfg,'CppPreserveClasses'));
        ret = ~isempty(opt) && opt(1);
    else
        ret = false;
    end
end
