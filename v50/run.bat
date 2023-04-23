cmake --build build --target all -j 18 

set dir=%~dp0
set main="%dir%build\main.exe"
set demo=Demo\SimpleDemo.exe

@REM cd C:\Users\10945\Desktop\Codecraft2023\Contest2\WindowsRelease

@REM robot_gui.exe -m maps\empty.txt "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" -f

@REM robot_gui.exe -m semi_maps\2.txt "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" -f


cd C:\Users\10945\Desktop\Codecraft2023\Contest3\WindowsRelease

robot_gui.exe -m maps\1.txt -f -d %main% %main%
@REM robot_gui.exe -m maps\2_p.txt -f -d %main% %demo%
