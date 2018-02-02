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
6. Светодиоды, резисторы и конденсаторы в соответствии с печатной платой.<br>
<br>
<b>Установка поддержки ESP-32 в Arduino Studio:</b><br>
<br>
https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/windows.md<br>
<br>
<b>Настройка библиотеки ESP32</b><br>
<br>
1. Файл ..\hardware\espressif\esp32\boards.txt<br>
В описании платы ESP32 Dev Module меняем частоту процессора на 80МГц<br>
esp32.name=ESP32 Dev Module<br>
....<br>
esp32.build.f_cpu=240000000L<br>
изменяем на:<br>
esp32.name=ESP32 Dev Module<br>
....<br>
esp32.build.f_cpu=80000000L<br>
<br>
2. Файл ..\hardware\espressif\esp32\platform.txt<br>
Добавляем возможность обрабатывать исключительные ситуации:<br>
# These can be overridden in platform.local.txt<br>
compiler.c.extra_flags=-<br>
compiler.c.elf.extra_flags=<br>
compiler.S.extra_flags=<br>
compiler.cpp.extra_flags=<br>
изменяем на:<br>
# These can be overridden in platform.local.txt<br>
compiler.c.extra_flags=-fexceptions<br>
compiler.c.elf.extra_flags=<br>
compiler.S.extra_flags=<br>
compiler.cpp.extra_flags=-fexceptions<br>
<br>
3. Файл ..\hardware\espressif\esp32\tools\sdk\sdkconfig<br>
Настриваем параметры для работы контроллера на частоте 80МГц:<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_80=<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_160=<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_240=y<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=240<br>
...<br>
CONFIG_FREERTOS_HZ=1000<br>
изменяем на:<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_80=y<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_160=<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_240=<br>
CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=80<br>
...<br>
CONFIG_FREERTOS_HZ=333<br>
<br>
4. Файл ..\hardware\espressif\esp32\tools\sdk\include\config\sdkconfig.h<br>
Настриваем параметры для работы контроллера на частоте 80МГц:<br>
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 240<br>
...<br>
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_240 1<br>
...<br>
#define CONFIG_FREERTOS_HZ 1000<br>
изменяем на:<br>
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 80<br>
...<br>
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_80 1<br>
...<br>
#define CONFIG_FREERTOS_HZ 333<br>
<br>
<b>Загрузка прошивки в контроллер</b><br>
<br>
Для загрузки прошивки в контроллер необходимо подключить к плате адаптер USB-TTL используя контакты для программирования<br>
Чтобы активировать режим прошивки контроллера необходимо замкнуть контакт IO0 контроллера на землю и перегрузить контроллер кратковременным замыканием контакта EN на землю.<br>
Если не удается загрузит прошивку на скорости 921600 - попробуйте понизить скорость до 115200.<br>
После удачной загрузки прошивки убрать перемычку с контакта IO0 на землю и перегрузить контроллер.<br>
<br>
<br>
<b>Настройка прибора:</b><br>
<br>
Первые 2 минуты после включения прибор находится в режиме настройки.<br>
В этом режиме прибор запускает WiFi точку доступа c именем "Parakeet" и без пароля.<br>
Для конфигурации прибора необходимо подключиться к этой точке доступа, запустить интернет браузер и в адресной строке ввести адрес http://192.168.70.1<br>
В открывшейся форме параметров можно настроить следующие параметры:<br>
<ul style="list-style-type:disc">
  <li>Код трансмиттера Dexcom</li>
  <li>Цифровой код для работы с облачным сервисом</li>
  <li>Адрес облачного сервиса</li>
  <li>Имя точки доступа WiFi для передачи данных в интернет. Если вы не планируете использовать WiFi оставьте это поле пустым.</li>
  <li>Пароль для точки доступа WiFi</li>
  <li>Формат обмена по BlueTooth (без BlueTooth, xDrip, xBridge)</li>
  <li>Признак работы прибора с мобильной сетью</li>
  <li>APN вышего мобильного оператора</li>
</ul>
После заполнения всех необходимых полей нажмите кнопку "Save".<br>
В случае использования прибора в режиме xBridge код трансмиттера Dexcom можно установить с помощью программы xDrip+ на вашем мобильном телефоне.<br>
В случае использования прибора с поддержкой мобильной сети, некоторые настройки можно изменять с помощью СМС на номер симкарты, установленной в прибор.<br>
Список обрабатываемых СМС:<br>
<ul style="list-style-type:disc">
  <li>TRANSMIT <Код трансмиттера Dexcom> - установить код трансмиттера Dexcom</li>
  <li>PWD <цифровой код> - установить цифровой код</li>
  <li>HTTP <адрес облачного сервиса> - установить адрес облачного сервиса</li>
  <li>APN <APN вашего мобильного оператора> - установить APN мобильного оператора</li>
  <li>SETTINGS - узнать текущие настройки. Текущие настройки будут присланы несколькими СМС-сообщениями</li>
  <li>REBOOT - перегрузить прибор</li>
</ul>
<br>
<b>Привязка прибора к программе xDrip+ по протоколу BlueTooth:</b><br>
<br>
Алгоритм работы прибора разработан таким образом, что BlueTooth включается только после того как прибор поймает сигнал от трансмиттера.<br>
Для привязки прибора к программе xDrip+ по протоколу BlueTooth необходимо выполнить следующие действия:<br>
1. Включить прибор.<br>
2. При необходимости настроить параметры прибора<br>
3. Дождаться сигнала с трансмиттера. Во время ожидания сигнала с трансмиттера прибор коротко мигает желтым светодиодом.<br>
4. После пойманного сигнала, прибор отправит данные в интернет (если такой режим работы настроен) и будет ждать 45 секунд соединения с программой xDrip+
по протоколу BlueTooth. При этом будет непрерывно гореть желтый светодиод.<br>
5. В течении 45 секунд в программе xDrip+  необходимо произвести BlueTooth Scan и выбрать прибор из списка найденных.<br>
