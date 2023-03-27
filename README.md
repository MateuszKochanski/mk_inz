Jest to projekt wykonany w celu realizacji pracy inżynierskiej, której temat to: "Sterowanie robotem sześcioosiowym z wykorzystaniem urządzenia do pomiaru siły i momentu" 

Przygotowano dwa węzły ROS napisane w języku C++ (hexCommunicator i controller) oraz program sterujący robota napisany w języku Python. Zadaniem pierwszego z węzłów jest komunikacja z urządzeniem do pomiaru siły i momentu obrotowego, oraz publikacja odbieranych danych na kanale ROS. Drugi węzeł śledzi aktualną pozycję robota oraz siłę przyłożoną do końcówki ramienia. Na ich podstawie oblicza nowe pozycje oraz publikuje je na kanale ROS. Program uruchamiany na robocie odbiera wysyłanepozycje oraz przemieszcza do nich końcówkę manipulatora.

## hex.cpp
Na topicu "hex" publikuje dane z urzadzenia pomiarowego OnRobot Hex-e w formacie geometry_msgs::Wrench. Aby dzialal prawidlowo nalezy ustawic prawidlowy adres IP urzadzenia.

## controler.cpp
Na podstawie danych na topicu "hex" oblicza nastepne punkty trajektorii robota oraz publikuje je na topicu "sterowanie".
<h1 align="center">
<br>

</h1>

<h4 align="center">Sterowanie robotem sześcioosiowym z wykorzystaniem czujnika sił i momentów</h4>
<p align="center">
  <img src="https://user-images.githubusercontent.com/103144228/210076329-2891d736-58e7-4843-8c14-ad0009072327.jpg"  alt="Stanowisko">
</p>
<p align="center">
  <a >
    <img src="https://user-images.githubusercontent.com/103144228/210075130-8dc5dd48-2533-4d51-bcc5-c6910f5810b0.png"  alt="screenshot">
  </a>
  
</p>
<!--
## Project Overview 🎉

## Tech/framework used 🔧

| Tech                                                    | Description                              |
| ------------------------------------------------------- | ---------------------------------------- |
| [X](X)                           | XYZ   |
| [X](X)                           | XYZ   |
| [X](X)                           | XYZ   |


## Screenshots 📺

<p align="center">
    <img src="" alt="Screenshot">
</p>

<p align="center">
    <img src="" alt="Screenshot">
</p>

<p align="center">
    <img src="" alt="Screenshot">
</p>

## Code Example/Issues 🔍


## Installation 💾

## Available scripts

| Command                   | Description                   |     |
| ------------------------- | ----------------------------- | --- |
| `npm run start`           | Open local server             |     |
| `npm run build`           | Create optimized build        |     |
| `npm run test`            | Run tests                     |     |


## Live 📍

## License 🔱-->
