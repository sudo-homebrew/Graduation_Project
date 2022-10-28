function updateBuildInfo (hCS, buildInfo, remoteBuild)
% Link against the cublas, cufft and cusolver libs when CUDA code
% generation enabled.

%   Copyright 2021-2022 The MathWorks, Inc.

if ros.codertarget.internal.isMATLABConfig(hCS)
    if ~isempty(hCS.GpuConfig)
        if hCS.GpuConfig.Enabled
            if ~isempty(hCS.DeepLearningConfig)
                targetDNN = hCS.DeepLearningConfig.TargetLibrary;
            else
                targetDNN = '';
            end
            updateMLGPUBuildInfo(targetDNN, buildInfo, remoteBuild);
        end
    end
else
    if strcmp(get_param(hCS,'GenerateGPUCode'),'CUDA')
        targetDNN = get_param(hCS,'DLTargetLibrary');
        updateMLGPUBuildInfo(targetDNN, buildInfo, remoteBuild);
    end
end

end

function updateMLGPUBuildInfo(taregtDNN, buildInfo, remoteBuild)
if remoteBuild
    if ispc
        buildInfo.removeLinkObjects('cudnn.lib',[],[]);
        buildInfo.removeLinkObjects('nvinfer.lib',[],[]);
    else
        buildInfo.removeLinkObjects('libcudnn.so',[],[]);
        buildInfo.removeLinkObjects('libnvinfer.so',[],[]);
    end
    buildInfo.addLinkFlags('-lcudnn');
    buildInfo.addLinkFlags('-lnvinfer');
    buildInfo.addLinkFlags('-lcublas');
    buildInfo.addLinkFlags('-lcusolver');
    buildInfo.addLinkFlags('-lcufft');
    buildInfo.addLinkFlags('-l${CUDA_LIBRARIES}');
else
    if ispc
        pathToCudaLib = fullfile(getenv('CUDA_PATH'),'lib','x64');
        buildInfo.addLinkObjects('cublas.lib', pathToCudaLib, '', true, true);
        buildInfo.addLinkObjects('cufft.lib', pathToCudaLib, '', true, true);
        buildInfo.addLinkObjects('cusolver.lib', pathToCudaLib, '', true, true);

        if strcmp(taregtDNN,'cudnn')
            pathToCuDNNLib = fullfile(getenv('NVIDIA_CUDNN'),'lib','x64');
            buildInfo.addLinkObjects('cudnn.lib', pathToCuDNNLib, '', true, true);
        elseif strcmp(taregtDNN,'tensorrt')
            pathToCuDNNLib = fullfile(getenv('NVIDIA_CUDNN'),'lib','x64');
            buildInfo.addLinkObjects('cudnn.lib', pathToCuDNNLib, '', true, true);

            pathToTensorRTLib = fullfile(getenv('NVIDIA_TENSORRT'),'lib','x64');
            buildInfo.addLinkObjects('nvinfer.lib', pathToTensorRTLib, '', true, true);
        end

    else
        pathToCudaLib = fullfile(getenv('CUDA_PATH'),'lib64');
        buildInfo.addLinkObjects('libcublas.so', pathToCudaLib, '', true, true);
        buildInfo.addLinkObjects('libcufft.so', pathToCudaLib, '', true, true);
        buildInfo.addLinkObjects('libcusolver.so', pathToCudaLib, '', true, true);

        if strcmp(taregtDNN,'cudnn')
            pathToCuDNNLib = fullfile(getenv('NVIDIA_CUDNN'),'lib64');
            buildInfo.addLinkObjects('libcudnn.so', pathToCuDNNLib, '', true, true);
        elseif strcmp(taregtDNN,'tensorrt')
            pathToCuDNNLib = fullfile(getenv('NVIDIA_CUDNN'),'lib64');
            buildInfo.addLinkObjects('libcudnn.so', pathToCuDNNLib, '', true, true);

            pathToTensorRTLib = fullfile(getenv('NVIDIA_TENSORRT'),'lib64');
            buildInfo.addLinkObjects('libnvinfer.so', pathToTensorRTLib, '', true, true);
        end
    end
end
end
