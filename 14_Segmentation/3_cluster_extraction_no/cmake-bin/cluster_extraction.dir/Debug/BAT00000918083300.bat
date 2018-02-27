@echo off
"C:\Program Files\CMake 2.8\bin\cmake.exe" "-HE:/CD_PCL_DEMO/14/14 chapter example code/3 cluster_extraction_no/source" "-BE:/CD_PCL_DEMO/14/14 chapter example code/3 cluster_extraction_no/cmake-bin" --check-stamp-file "E:\CD_PCL_DEMO\14\14 chapter example code\3 cluster_extraction_no\cmake-bin\CMakeFiles\generate.stamp"
if errorlevel 1 goto :VCReportError

if errorlevel 1 goto VCReportError
goto VCEnd
:VCReportError
echo Project : error PRJ0019: 某个工具从以下位置返回了错误代码: "Building Custom Rule E:/CD_PCL_DEMO/14/14 chapter example code/3 cluster_extraction_no/source/CMakeLists.txt"
exit 1
:VCEnd