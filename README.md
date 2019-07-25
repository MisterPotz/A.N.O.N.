# Repository for a ROS-based robot

На данный момент робот:
    1) Реагирует на команды с клавиатуры (ручное управление)
    2) Способен доезжать до точки (2D Nav-goal) определенной через RVIZ.

Также репозиторий имеет два пэкэджа (actionlib_samples и actionlib_server) с примерами actionlib библиотеки.

## Getting Started

Для того, чтобы подготовить проект к запуску, необходимо:
#### 1) Иметь все представленные файлы в заданном пути:
```
 /<ИМЯ_WORKSPACE>/src/
```
#### 2) Иметь установленный пэкэдж:
```
ros-melodic-navigation
```
#### 3) Скомпилировать проект. Из корневой папки workspace:
```
catkin_make
```

## Prerequisites

Данный проект основан на ROS Melodic. Для запуска рекомендуется именно эта версия.


## Running the code

Для запуска используется следующая команда
```
roslaunch bot simple_bot.launch
```
#### 1) Для управления роботом с клавиатуры 
Cделать окно терминала, использовать клавиши: U, I, O, J, K, L, M, ',', '.';
#### 2) Для управления роботом через Rviz
 a) нажать кнопку 2D nav goal
 b) выбрать на карте goal


## Made by A.N.O.N. team
Александр Горностаев,
Никита Ляшенко,
Олег Дмитриев,
Никита Белоусов
