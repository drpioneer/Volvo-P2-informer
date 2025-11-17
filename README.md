Экспериментально-познавательный проект, появившийся из любопытства.

Программно-аппаратное решение на базе ArduinoNano + 2x MCP2515, подключаемое к CAN-шинам (высокоскоростной и низкоскоростной) легковых автомобилей Volvo на платформе P2 (в составе: Volvo S80 1998–2006, Volvo S60 2000–2009, Volvo V70 2000–2007, Volvo XC70 2000–2007, Volvo XC90 2002–2014), позволяющее собирать заранее заданные параметры из CAN-шин а/м и выводить их на штатную приборную панель а/м (модуль DIM). 

Управление устройством производится кнопкой INFO/RESET на левом подрулевом рычажке. Распознаются как короткие, так и длительные нажатия кнопки. В коде использованы идеи и наработки разных авторов, найденные на просторах интернета. Принципиальная работоспособность решения проверена на XC90 2011 модельного года. Предполагаю работоспособность прототипа без изменений кода для автомобилей Volvo P2 2005 и свежЕе. Для автомибилей Volvo P2 1998-2004 потребуется исправление параметров высокоскоростной CAN-шины и указание иных идентификаторов целевых модулей CAN. 
Проект допиливается по мере желания, сил и возможностей. Уверен, что код проекта несовершенен и содержит ошибки. Приветствуются советы и конструктивная критика по улучшению кода.

Схема электрических соединений:

![sch](https://github.com/user-attachments/assets/a9c4d5ac-88a2-4806-ad35-0f1725c3ff0b)

Прототип устройства:

![prototype](https://github.com/user-attachments/assets/880a559b-e174-4b7b-ac5e-add334906784)

Взрыв-схема предполагаемой кросс-платы в сборе:

<img src="https://github.com/user-attachments/assets/c637afbc-205e-4980-8052-bc60a41fc31e" />

<img src="https://github.com/user-attachments/assets/3238c791-a25b-4c7e-9276-64b939b83fe8" />

Покупные компоненты:

- https://aliexpress.ru/item/1005009497676801.html - 2шт
- https://aliexpress.ru/item/32915861640.html - 1шт
- https://aliexpress.ru/item/1005002538696313.html - 1шт
- https://aliexpress.ru/item/1005007870477550.html - 1шт

## Список использованной литературы:

1. https://hackingvolvo.blogspot.com/
2. https://github.com/hackingvolvo
3. https://hackingvolvoxc90.blogspot.com/
4. https://www.drive2.com/l/9769115
5. https://www.drive2.com/l/9824565/
6. https://www.drive2.com/l/9878147/
7. https://www.drive2.com/l/458462539873794364/
8. https://www.drive2.com/l/634000945637755124/
9. https://github.com/vtl/volvo-ddd
10. https://github.com/vtl/volvo-kenwood
11. https://www.drive2.com/l/712830156913249071/
12. https://www.drive2.com/l/717368116278996470/
13. https://www.drive2.com/l/523338948592799298/
14. https://github.com/olegel/VolvoCan
15. https://github.com/olegel/CANLogger
16. https://github.com/latonita/arduino-canbus-monitor
17. https://we.easyelectronics.ru/autoelectro/podklyuchenie-knopok-upravleniya-magnitoloy-po-can.html?ysclid=mg0iokknp5970394408
18. https://www.motor-talk.de/forum/der-can-bus-im-volvo-t4420570.html?page=5

Зеркало на DRIVE2: https://www.drive2.ru/l/718050088366114704/

**Используете код или понравилось решение - отметьте это звездочкой, Вам не сложно, а мне приятно!**
