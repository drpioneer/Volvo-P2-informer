# Reading & displaying parameters on the dashboard of the Volvo P2
![IMG_20251117_173650](https://github.com/user-attachments/assets/d68cac6f-8f70-4a17-8365-0c095f5f8ee4)

Software and hardware solution based on ArduinoNano +2x MCP2515, connected to CAN buses (high-speed and low-speed) of Volvo passenger cars on the P2 platform (as part of: Volvo S80 1998-2006, Volvo S60 2000-2009, Volvo V70 2000-2007, Volvo XC70 2000-2007, Volvo XC90 2002-2014), which allows you to collect preset parameters from the vehicle's CAN buses and output them to the vehicle's standard dashboard (DIM module).

The device is controlled by the INFO/RESET button on the left steering lever. Both short presses (changing the current parameter) and long button presses (disabling/enabling the display on the dashboard screen) are recognized. The code uses ideas and developments from various authors found on the Internet. The basic functionality of the solution was tested on the XC90 2011 model year. I assume that the prototype will work without changing the code for Volvo P2 2005 and newer cars. For the Volvo P2 1998-2004 vehicle, it will be necessary to correct the parameters of the high-speed CAN bus and specify other identifiers of the target CAN modules. The project is completed according to the desire, strength and capabilities. I am sure that the project code is imperfect and contains errors. Tips and constructive criticism on improving the code are welcome.

## Device prototype
![prototype](https://github.com/user-attachments/assets/880a559b-e174-4b7b-ac5e-add334906784)

## Electrical connection diagram
![sch](https://github.com/user-attachments/assets/737c2150-9948-446e-b447-fc642b59534f)

## Explosion-circuit diagram of the cross-board assembly
<img src="https://github.com/user-attachments/assets/5415219c-383e-43d8-9707-bf82338e823e">
<img src="https://github.com/user-attachments/assets/ce9224c2-028c-481c-9bc8-398bd6714e60">

## Purchased components
- Arduino Nano - https://aliexpress.ru/item/32915861640.html - 1шт
- MCP2515 - https://aliexpress.ru/item/1005009497676801.html - 2шт
- DC-DC преобразователь (12В -> 5В) - https://aliexpress.ru/item/1005002538696313.html - 1шт
- Разъём OBD2 - https://aliexpress.ru/item/1005007870477550.html - 1шт

## List of used literature
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
17. https://we.easyelectronics.ru/autoelectro/podklyuchenie-knopok-upravleniya-magnitoloy-po-can.html
18. https://www.motor-talk.de/forum/der-can-bus-im-volvo-t4420570.html?page=5

Mirror on DRIVE2: https://www.drive2.ru/l/718050088366114704/

**If you use the code or like the solution, mark it with an asterisk, it's not difficult for you, but I'm pleased!**
