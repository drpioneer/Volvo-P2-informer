# Reading & displaying parameters on the dashboard of the Volvo P2    Чтение и вывод параметров на приборную панель Volvo P2
![IMG_20251117_173650](https://github.com/user-attachments/assets/d68cac6f-8f70-4a17-8365-0c095f5f8ee4)

Software and hardware solution based on ArduinoNano +2x MCP2515, connected to CAN buses (high-speed and low-speed) of Volvo passenger cars on the P2 platform (as part of: Volvo S80 1998-2006, Volvo S60 2000-2009, Volvo V70 2000-2007, Volvo XC70 2000-2007, Volvo XC90 2002-2014), which allows you to collect preset parameters from the vehicle's CAN buses and output them to the vehicle's standard dashboard (DIM module).

The device is controlled by the INFO/RESET button on the left steering lever. Both short and long button presses are recognized. The code uses ideas and developments from various authors found on the Internet. The basic functionality of the solution has been tested on the XC90 2011 model year. I assume that the prototype will work without changing the code for Volvo P2 2005 and newer vehicles. For the Volvo P2 1998-2004 vehicles, it will be necessary to correct the parameters of the high-speed CAN bus and specify other identifiers of the target CAN modules. The project is completed according to the desire, strength and capabilities. I am sure that the project code is imperfect and contains errors. Tips and constructive criticism on improving the code are welcome.
---------------------
Программно-аппаратное решение на базе ArduinoNano + 2x MCP2515, подключаемое к CAN-шинам (высокоскоростной и низкоскоростной) легковых автомобилей Volvo на платформе P2 (в составе: Volvo S80 1998–2006, Volvo S60 2000–2009, Volvo V70 2000–2007, Volvo XC70 2000–2007, Volvo XC90 2002–2014), позволяющее собирать заранее заданные параметры из CAN-шин а/м и выводить их на штатную приборную панель а/м (модуль DIM). 

Управление устройством производится кнопкой INFO/RESET на левом подрулевом рычажке. Распознаются как короткие, так и длительные нажатия кнопки. В коде использованы идеи и наработки разных авторов, найденные на просторах интернета. Принципиальная работоспособность решения проверена на XC90 2011 модельного года. Предполагаю работоспособность прототипа без изменений кода для автомобилей Volvo P2 2005 и свежЕе. Для автомибилей Volvo P2 1998-2004 потребуется исправление параметров высокоскоростной CAN-шины и указание иных идентификаторов целевых модулей CAN. 
Проект допиливается по мере желания, сил и возможностей. Уверен, что код проекта несовершенен и содержит ошибки. Приветствуются советы и конструктивная критика по улучшению кода.

## Device prototype * Прототип устройства:
![prototype](https://github.com/user-attachments/assets/880a559b-e174-4b7b-ac5e-add334906784)

## Electrical connection diagram * Схема электрических соединений:
![sch](https://github.com/user-attachments/assets/a9c4d5ac-88a2-4806-ad35-0f1725c3ff0b)

## Explosion-circuit diagram of the cross-board assembly * Взрыв-схема кросс-платы в сборе:
<img src="https://github.com/user-attachments/assets/c637afbc-205e-4980-8052-bc60a41fc31e" />
<img src="https://github.com/user-attachments/assets/3238c791-a25b-4c7e-9276-64b939b83fe8" />

## Purchased components * Покупные компоненты:
- https://aliexpress.ru/item/1005009497676801.html - 2шт
- https://aliexpress.ru/item/32915861640.html - 1шт
- https://aliexpress.ru/item/1005002538696313.html - 1шт
- https://aliexpress.ru/item/1005007870477550.html - 1шт

## List of used literature * Список использованной литературы:
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

Mirror on DRIVE2 * Зеркало на DRIVE2: https://www.drive2.ru/l/718050088366114704/

**If you use the code or like the solution, mark it with an asterisk, it's not difficult for you, but I'm pleased!**

**Используете код или понравилось решение - отметьте это звездочкой, Вам не сложно, а мне приятно!**
