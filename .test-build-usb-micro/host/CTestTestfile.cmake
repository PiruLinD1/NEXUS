# CMake generated Testfile for 
# Source directory: C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro
# Build directory: C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/host
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[pulse_guard]=] "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/host/Debug/guard_test.exe")
  set_tests_properties([=[pulse_guard]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;11;add_test;C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[pulse_guard]=] "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/host/Release/guard_test.exe")
  set_tests_properties([=[pulse_guard]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;11;add_test;C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[pulse_guard]=] "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/host/MinSizeRel/guard_test.exe")
  set_tests_properties([=[pulse_guard]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;11;add_test;C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[pulse_guard]=] "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/host/RelWithDebInfo/guard_test.exe")
  set_tests_properties([=[pulse_guard]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;11;add_test;C:/Users/rugge/Documents/vex-vscode-projects/SX_SAWP/.test-build-usb-micro/CMakeLists.txt;0;")
else()
  add_test([=[pulse_guard]=] NOT_AVAILABLE)
endif()
