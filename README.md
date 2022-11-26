# mk_inz
Force controle system for 6-axis robot.

## hex.cpp
Na topicu "hex" publikuje dane z urzadzenia pomiarowego OnRobot Hex-e w formacie geometry_msgs::Wrench. Aby dzialal prawidlowo nalezy ustawic prawidlowy adres IP urzadzenia.

## controler.cpp
Na podstawie danych na topicu "hex" oblicza nastepne punkty trajektorii robota oraz publikuje je na topicu "sterowanie".
