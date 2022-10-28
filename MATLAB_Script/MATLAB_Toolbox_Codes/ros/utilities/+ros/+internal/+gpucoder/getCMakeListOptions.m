function out = getCMakeListOptions(cpplibname, rosversion, cudaflags)
% Add required CMAKE flags to CUDA remote node generation

%   Copyright 2021 The MathWorks, Inc.
if strcmp(rosversion,'ROS1')
    hs = StringWriter;
    hs.addcr('enable_language(CUDA)');
    hs.addcr('include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})');
    hs.addcr('set_target_properties(%s PROPERTIES CUDA_SEPARABLE_COMPILATION ON)',cpplibname);
    hs.addcr('if (MSVC)');
    hs.addcr('  #This is a workaround for MSVC. The flag _VARIADIC_MAX is not passed to NVCC with -Xcomplier prepending.');
    hs.addcr('  remove_definitions(/D _VARIADIC_MAX=10)');
    hs.addcr('  #This is a workaround for MSVC. The flag /Zc:__cplusplus is not passed to NVCC with -Xcomplier prepending.');
    hs.addcr('  get_target_property(compileOptions %s COMPILE_OPTIONS)',cpplibname);
    hs.addcr('  string(REPLACE "/Zc:__cplusplus" "" FLAGS "${compileOptions}")');
    hs.addcr('  set_target_properties(%s PROPERTIES COMPILE_OPTIONS "${FLAGS}")',cpplibname);
    hs.addcr('  target_compile_options(%s PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: %s -DWIN32 -rdc=true -Xcudafe "--display_error_number --diag_suppress=unsigned_compare_with_zero --diag_suppress=field_without_dll_interface --diag_suppress=base_class_has_different_dll_interface" -Xcompiler="/DWIN32 /D_WINDOWS /D _VARIADIC_MAX=10 /Zc:__cplusplus /DBOOST_NO_CXX11_CONSTEXPR">)',cpplibname, cudaflags);
    hs.addcr('  target_compile_options(%s PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/D_VARIADIC_MAX=10 /DBOOST_NO_CXX11_CONSTEXPR /DBOOST_NO_CXX14_CONSTEXPR>)',cpplibname);
    hs.addcr('else()');
    hs.addcr(' target_compile_options(%s PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: %s -rdc=true -Xcudafe "--display_error_number --diag_suppress=unsigned_compare_with_zero --diag_suppress=field_without_dll_interface --diag_suppress=base_class_has_different_dll_interface" -Xcompiler=-fPIC>)',cpplibname, cudaflags);
    hs.addcr('endif()');
    out = hs.string;
else
    hs = StringWriter;
    hs.addcr('enable_language(CUDA)');
    hs.addcr('include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})');
    hs.addcr('set_target_properties(%s PROPERTIES CUDA_SEPARABLE_COMPILATION ON)',cpplibname);
    hs.addcr('if (MSVC)');
    hs.addcr('  target_compile_options(%s PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:%s -rdc=true -Xcudafe "--display_error_number --diag_suppress=unsigned_compare_with_zero --diag_suppress=field_without_dll_interface --diag_suppress=base_class_has_different_dll_interface" >)',cpplibname, cudaflags);
    hs.addcr('else()');
    hs.addcr(' target_compile_options(%s PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:%s -rdc=true -Xcudafe "--display_error_number --diag_suppress=unsigned_compare_with_zero --diag_suppress=field_without_dll_interface --diag_suppress=base_class_has_different_dll_interface" -Xcompiler=-fPIC>)',cpplibname, cudaflags);
    hs.addcr('endif()');
    out = hs.string;
end
end