cmake_minimum_required(VERSION 3.21)
project(MakeUDF VERSION 0.1)

# =============================================================================
# ANSYS Fluent 安装位置与版本
# 可通过以下任一方式覆盖（优先级从高到低）：
#   1. cmake -DFLUENT_ROOT=... -DRELEASE=... -DLIB_RELEASE=...
#   2. 环境变量 FLUENT_ROOT
#   3. 下面的默认值
# =============================================================================
if(NOT DEFINED FLUENT_ROOT)
    if(DEFINED ENV{FLUENT_ROOT})
        set(_fdm_default_fluent_root "$ENV{FLUENT_ROOT}")
    else()
        set(_fdm_default_fluent_root "C:/Program Files/ANSYS Inc/v221/fluent")
    endif()
    set(FLUENT_ROOT "${_fdm_default_fluent_root}" CACHE PATH
        "ANSYS Fluent 安装根目录（其下应包含 ntbin/ 和 fluent<版本>/）")
endif()

set(FLUENT_ARCH "win64" CACHE STRING "Fluent 平台目录")

set(RELEASE "22.1.0" CACHE STRING
    "Fluent 版本号，例如 22.1.0 / 23.2.0 / 24.1.0")
set(LIB_RELEASE "2210" CACHE STRING
    "Fluent 库版本号，例如 2210 / 2320 / 2410（去掉版本号中的点）")
set(TOOL_PATH "${FLUENT_ROOT}/ntbin/${FLUENT_ARCH}")

# =============================================================================
# 算例类型：必须与 Fluent case 的求解器设置一致
# =============================================================================
set(DIMENSION "3d" CACHE STRING "2d 或 3d")
set(PRECISE "dp" CACHE STRING "dp（双精度）或留空（单精度）")
set(PARALLEL_NODE "msmpi" CACHE STRING "并行方式：intel / msmpi / none / net")

# 设置引入cmake模块
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/import)
include(CompilerConfig)
# CoolProp 当前未使用；如需要，取消下一行注释并补齐 import/CoolProp.dll 与 .lib
# include(CoolPropConfig)

add_subdirectory(src)
