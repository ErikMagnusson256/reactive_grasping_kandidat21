Hur man testar så att repot fungerar på egen dator:

1. Öppna en terminal i linux (ctr+alt+T)
2. Navigera till där repot är nedladdat och gå in i catkin_kandidat mappen
3. I terminalen, skriv: catkin_make
4. I terminalen, skriv: source devel/setup.bashrc
5. I terminalen, skriv: roscore
6. Öppna ny terminal, skriv: rosrun test_pkg publisher_test.py
7. Öppna ny terminal, skriv: rostopic echo /counter

Förväntad beteende, de två första terminalfönstrena som öppnades bör visa någon text men inte göra någonting mer. Det sista terminalfönstret bör skriva ut siffror i ökande ordning.
