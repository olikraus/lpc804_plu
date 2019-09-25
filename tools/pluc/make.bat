echo off
rem make.bat
rem
set TARGET=pluc
set SRC=
for %%f in (*.c) do call set SRC=%%SRC%% %%f
echo on
%CC% -O %SRC% -o %TARGET%.exe
