cmake --build build --target all -j 18 
@REM cd C:\Users\10945\Desktop\Codecraft2023\Contest1\TestContest1\WindowsRelease
cd C:\Users\10945\Desktop\Codecraft2023\Contest2\WindowsRelease

@REM robot_gui.exe -m maps\empty.txt "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" -f

@REM robot_gui.exe -m semi_maps\2.txt "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" -f


cd C:\Users\10945\Desktop\Codecraft2023\Contest3\WindowsRelease

robot_gui.exe -m maps\2.txt -f -d "C:\Users\10945\Desktop\Codecraft2023\Contest3\main_v30\build\main.exe" "C:\Users\10945\Desktop\Codecraft2023\Contest3\main_v30\build\main.exe"
@REM robot_gui.exe -m maps\2.txt -f -d "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\opp2.exe" 
@REM robot_gui.exe -m maps\2.txt -f "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\opp.exe"
@REM robot_gui.exe -m maps\2.txt -f -d Demo\SimpleDemo.exe "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe"
@REM robot_gui.exe -m maps\2.txt -f "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" Demo\SimpleDemo.exe 


@REM robot_gui.exe -m oldmaps\8.txt -f "C:\Users\10945\Desktop\Codecraft2023\Contest3\main\build\main.exe" Demo\SimpleDemo.exe 
