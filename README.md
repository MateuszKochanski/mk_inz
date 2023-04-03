Jest to projekt wykonany w celu realizacji pracy inżynierskiej, której temat to: "Sterowanie robotem sześcioosiowym z wykorzystaniem urządzenia do pomiaru siły i momentu" 

Przygotowano dwa węzły ROS napisane w języku C++ (hexCommunicator i controller) oraz program sterujący robota napisany w języku Python. Zadaniem pierwszego z węzłów jest komunikacja z urządzeniem do pomiaru siły i momentu obrotowego, oraz publikacja odbieranych danych na kanale ROS. Drugi węzeł śledzi aktualną pozycję robota oraz siłę przyłożoną do końcówki ramienia. Na ich podstawie oblicza nowe pozycje oraz publikuje je na kanale ROS. Program uruchamiany na robocie odbiera wysyłanepozycje oraz przemieszcza do nich końcówkę manipulatora.

## hex.cpp
Na topicu "hex" publikuje dane z urzadzenia pomiarowego OnRobot Hex-e w formacie geometry_msgs::Wrench. Aby dzialal prawidlowo nalezy ustawic prawidlowy adres IP urzadzenia.
![Image](https://github.com/MateuszKochanski/mk_inz/blob/master/images/testy_.png)
## controler.cpp
Na podstawie danych na topicu "hex" oblicza nastepne punkty trajektorii robota oraz publikuje je na topicu "sterowanie".
<h1 align="center">

