@echo off
chcp 65001 >nul
setlocal

set WORK=C:\Users\13588\Desktop\learning\programing\STM32\C_Board_Assessment
set VS=C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools

call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

cd /d "%WORK%\_test"

cl /nologo /std:c++17 /EHsc /utf-8 /W3 /I"%WORK%\Tasks\inc" test_pc.cpp "%WORK%\Tasks\src\gm6020.cpp" "%WORK%\Tasks\src\pid.cpp" "%WORK%\Tasks\src\trajectory.cpp" /Fe:test_pc.exe
if errorlevel 1 exit /b 1

echo.
echo === RUN ===
test_pc.exe
echo EXITCODE=%errorlevel%
endlocal
