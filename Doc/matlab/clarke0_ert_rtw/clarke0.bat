
set MATLAB=E:\software_qrs\matlab

cd .

if "%1"=="" ("E:\software_qrs\matlab\bin\win64\gmake"  -f clarke0.mk all) else ("E:\software_qrs\matlab\bin\win64\gmake"  -f clarke0.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1