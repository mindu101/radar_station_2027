include(FindPackageHandleStandardArgs)

find_package(CUDA REQUIRED)

set(TensorRT_ROOT /usr CACHE PATH "TensorRT root path")

# Also discover TensorRT installed under the user's home directory. The
# development headers and versioned runtime libraries may live in separate
# prefixes when the runtime was installed by pip.
file(GLOB _TensorRT_LOCAL_ROOTS "$ENV{HOME}/.local/opt/radar-tensorrt-*")
set(_TensorRT_SEARCH_ROOTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} ${_TensorRT_LOCAL_ROOTS})

# find_path/find_library trust cache entries even after their files are removed.
# Drop stale entries so reconfiguration can search an installed TensorRT again.
if(TensorRT_INCLUDE_DIRS AND NOT EXISTS "${TensorRT_INCLUDE_DIRS}/NvInfer.h")
    unset(TensorRT_INCLUDE_DIRS CACHE)
    unset(TensorRT_INCLUDE_DIRS)
endif()
foreach(_trt_library NVINFER NVINFER_PLUGIN NVONNXPARSER)
    if(TensorRT_LIBRARY_${_trt_library} AND NOT EXISTS "${TensorRT_LIBRARY_${_trt_library}}")
        unset(TensorRT_LIBRARY_${_trt_library} CACHE)
        unset(TensorRT_LIBRARY_${_trt_library})
    endif()
endforeach()

# find all include directories
find_path(TensorRT_INCLUDE_DIRS NvInfer.h
        HINTS ${_TensorRT_SEARCH_ROOTS}
        PATH_SUFFIXES include include/x86_64-linux-gnu usr/include/x86_64-linux-gnu)

# find all libraries
find_library(TensorRT_LIBRARY_NVINFER nvinfer
        HINTS ${_TensorRT_SEARCH_ROOTS}
        PATH_SUFFIXES lib lib/x86_64-linux-gnu usr/lib/x86_64-linux-gnu)
find_library(TensorRT_LIBRARY_NVINFER_PLUGIN nvinfer_plugin
        HINTS ${_TensorRT_SEARCH_ROOTS}
        PATH_SUFFIXES lib lib/x86_64-linux-gnu usr/lib/x86_64-linux-gnu)
#find_library(TensorRT_LIBRARY_NVCAFFE_PARSER nvcaffe_parser
#        HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
#        PATH_SUFFIXES lib)
find_library(TensorRT_LIBRARY_NVONNXPARSER nvonnxparser
        HINTS ${_TensorRT_SEARCH_ROOTS}
        PATH_SUFFIXES lib lib/x86_64-linux-gnu usr/lib/x86_64-linux-gnu)

# pip packages provide versioned .so files without the unversioned linker
# symlinks expected by find_library. Use the header's major version to select
# the matching runtime libraries.
if(TensorRT_INCLUDE_DIRS AND EXISTS "${TensorRT_INCLUDE_DIRS}/NvInferVersion.h")
    file(STRINGS "${TensorRT_INCLUDE_DIRS}/NvInferVersion.h" _TensorRT_MAJOR_LINE
            REGEX "^#define TRT_MAJOR_ENTERPRISE [0-9]+")
    if(NOT _TensorRT_MAJOR_LINE)
        file(STRINGS "${TensorRT_INCLUDE_DIRS}/NvInferVersion.h" _TensorRT_MAJOR_LINE
                REGEX "^#define NV_TENSORRT_MAJOR [0-9]+")
    endif()
    string(REGEX MATCH "[0-9]+$" _TensorRT_SONAME_MAJOR "${_TensorRT_MAJOR_LINE}")
endif()
if(_TensorRT_SONAME_MAJOR)
    file(GLOB _TensorRT_PYTHON_LIB_DIRS
            "$ENV{HOME}/miniforge3/envs/*/lib/python*/site-packages/tensorrt_libs")
    if(DEFINED ENV{CONDA_PREFIX})
        file(GLOB _TensorRT_ACTIVE_PYTHON_LIB_DIRS
                "$ENV{CONDA_PREFIX}/lib/python*/site-packages/tensorrt_libs")
        list(PREPEND _TensorRT_PYTHON_LIB_DIRS ${_TensorRT_ACTIVE_PYTHON_LIB_DIRS})
    endif()
    foreach(_TensorRT_LIB_NAME nvinfer nvinfer_plugin nvonnxparser)
        string(TOUPPER "${_TensorRT_LIB_NAME}" _TensorRT_LIB_KEY)
        if(TensorRT_LIBRARY_${_TensorRT_LIB_KEY} AND
           NOT EXISTS "${TensorRT_LIBRARY_${_TensorRT_LIB_KEY}}")
            unset(TensorRT_LIBRARY_${_TensorRT_LIB_KEY} CACHE)
            unset(TensorRT_LIBRARY_${_TensorRT_LIB_KEY})
        endif()
        if(NOT TensorRT_LIBRARY_${_TensorRT_LIB_KEY})
            find_file(TensorRT_LIBRARY_${_TensorRT_LIB_KEY}
                    NAMES "lib${_TensorRT_LIB_NAME}.so.${_TensorRT_SONAME_MAJOR}"
                    PATHS ${_TensorRT_PYTHON_LIB_DIRS} NO_DEFAULT_PATH)
        endif()
    endforeach()
endif()
#find_library(TensorRT_LIBRARY_NVPARSERS nvparsers
#        HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
#        PATH_SUFFIXES lib)
#find_library(TensorRT_LIBRARY_CUDART cudart
#        HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
#        PATH_SUFFIXES lib)

set(TensorRT_LIBS
        ${TensorRT_LIBRARY_NVINFER}
        ${TensorRT_LIBRARY_NVINFER_PLUGIN}
#        ${TensorRT_LIBRARY_NVCAFFE_PARSER}
        ${TensorRT_LIBRARY_NVONNXPARSER}
#        ${TensorRT_LIBRARY_NVPARSERS}
#        ${TensorRT_LIBRARY_CUDART}
        )
# find all source files
#aux_source_directory(${TensorRT_ROOT}/samples/common TensorRT_SOURCE)
#set(TensorRT_SOURCE ${TensorRT_ROOT}/samples/common/logger.cpp)

find_package_handle_standard_args(
        TensorRT DEFAULT_MSG TensorRT_INCLUDE_DIRS
        TensorRT_LIBRARY_NVINFER TensorRT_LIBRARY_NVINFER_PLUGIN
        TensorRT_LIBRARY_NVONNXPARSER)
