cmake_minimum_required(VERSION 3.14)
project(qpoases_vendor)

# 确保生成位置无关代码和动态库
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS
    ON
    CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)
include(FetchContent)

FetchContent_Declare(
  qpoases_external
  GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f)

set(QPOASES_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(qpoases_external)

set_target_properties(
  qpOASES
  PROPERTIES NO_SONAME 1
             SOVERSION ""
             VERSION "")
# --- 新增：手动执行安装逻辑 ---

# 1. 安装头文件到 install/qpoases_vendor/include
install(DIRECTORY ${qpoases_external_SOURCE_DIR}/include/ DESTINATION include)

# 1. 安装库文件到 install/qpoases_vendor/lib qpOASES 是 FetchContent 下载后定义的 target 名称
install(TARGETS qpOASES DESTINATION lib)

# --- 导出配置 ---

# 导出安装后的路径
ament_export_include_directories(include)
ament_export_libraries(qpOASES)

ament_package()
