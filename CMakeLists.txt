cmake_minimum_required(VERSION 3.16)
project(robot_motor_protocol VERSION 1.0.0 LANGUAGES CXX)

# 強制 C++17 標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 即時控制系統優化標頭與警告
add_compile_options(
    -O3                             # 最高階編譯優化
    -Wall -Wextra -Wpedantic        # 嚴格代碼品質檢查
    -Wshadow -Wnon-virtual-dtor     # 預防潛在 Bug
    -fno-omit-frame-pointer        # 便於 RT 週期與 Perf 分析
)

# 定義 Motor Protocol 靜態庫
add_library(mit_protocol
    src/protocol/mit_protocol.cpp
)

# 設定公共 Include 目錄（Target-based）
target_include_directories(mit_protocol
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 定義測試執行檔
add_executable(test_mit_protocol
    test/test_mit_protocol.cpp
)

# 連結測試目標與庫
target_link_libraries(test_mit_protocol
    PRIVATE
        mit_protocol
)

# 啟用 CMake 測試架構
enable_testing()
add_test(NAME MITProtocolUnitTest COMMAND test_mit_protocol)