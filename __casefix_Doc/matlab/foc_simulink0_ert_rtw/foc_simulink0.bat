
set MATLAB=D:\tools\matlab2022

cd .

if "%1"=="" ("D:\tools\matlab2022\bin\win64\gmake"  -f foc_simulink0.mk all) else ("D:\tools\matlab2022\bin\win64\gmake"  -f foc_simulink0.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1