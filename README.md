# xDrip-ESP32
Проект с функциональностью wixel-xDrip, xBridge2 и Parakeet на контроллере ESP32<br>
<br>

Данный проект является развитием проекта Parakeet-A (https://github.com/Gorgy70/parakeet-A) и основан на следующих проектах:<br>
1. Parakeet от Jamorham (https://jamorham.github.io, https://github.com/jamorham/wixel-xDrip)<br>
2. XDrip от Emma Black (http://stephenblackwasalreadytaken.github.io/xDrip, https://github.com/StephenBlackWasAlreadyTaken/xDrip)<br>
3. CC2500-Project от Don Browne (https://github.com/brownedon/CC2500-Project)<br>
<br>
<br>
<b>Сборка прибора:</b><br>
<br>
Для сборки прибора необходимы следующие компоненты:<br>
1. Контроллер ESP-32. <br>
Я использую модуль ESP32-WROOM:<br>
https://www.aliexpress.com/item/ESP-32S-ESP-WROOM-32-ESP32-ESP-32-Bluetooth-and-WIFI-Dual-Core-CPU-with-Low/32826634234.html?spm=a2g0s.9042311.0.0.ZQhor8
<br>
2. Радиомодуль на базе чипа Texas Instruments CC2500 с усилителем слабого сигнала.<br>
https://www.aliexpress.com/item/CC2500-PA-LNA-Romote-Wireless-Module-CC2500-SI4432-For-RC-Quadcopter-Helicopter-Multirotor-Model-Parts/32740098548.html?spm=a2g0s.9042311.0.0.ZQhor8
<br>
3. Модуль контроля заряда батареи.<br>
4. Аккумулятор 3.7 В.<br>
5. Стаблизатор, для понижения напряжения до 3.3В:
https://www.aliexpress.com/item/Free-Shipping-Hot-Sale-Smart-Electronics-Integrated-Circuit-AP7361-15FGE-7-IC-REG-LDO-1-5V/32584383940.html?spm=a2g0s.9042311.0.0.ZQhor8
<br>
6. Светодиоды, резисторы и конденсаторы в соответствии с печатной платой.
<b>Установка поодержки ESP-32 в Arduino Studio:</b><br>
<br>
https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/windows.md
<br>
<b>Загрузка прошивки в контроллер</b><br>
Для загрузки прошивки в контроллер необходимо подключить к плате адаптер USB-TTL используя контакты для программирования<br>
Чтобы активировать режим прошивки контроллера необходимо замкнуть контакт IO0 контроллера на землю и перегрузить контроллер кратковременным замыканием контакта EN на землю.<br>
Если не удается загрузит прошивку на скорости 921600 - попробуйте понизить скорость до 115200.<br>
