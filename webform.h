
const char* edit_form = "<html>\
<title>\
Parakeet-8266 Settings\
</title>\
<body> \
<form action='save' method='POST'>\
<table  border='0' cellpadding='2' cellspacing='2' style='border-collapse: collapse' width='100%%'>\
<tr>\
  <td align=center>\
    Parakeet-8266 Settings\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    Transmitter ID:\
  </td>\
  <td align=left>\
   <input type='text' name='DexcomID' maxlength='5' size=5 value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    Password Code:\
  </td>\
  <td>\
    <input type='text' name='PasswordCode' maxlength='5' size='5'  value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    Webservice URL:\
  </td>\
  <td>\
    <input type='text' name='WebService' maxlength='55' size='55' value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    WiFi SSID:\
  </td>\
  <td>\
    <input type='text' name='WiFiSSID' maxlength='15' size='15' value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    WiFi Password:\
  </td>\
  <td>\
    <input type='text' name='WiFiPwd' maxlength='15' size='15' value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    BlueTooth Format:\
  </td>\
  <td>\
    <input type='radio' name='BtFormat' value='0' %s>None\
    <input type='radio' name='BtFormat' value='1' %s>xDrip\
    <input type='radio' name='BtFormat' value='2' %s>xBridge\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    Use Moble network:\
  </td>\
  <td>\
    <input type='checkbox' name='UseGSM' value='YES'%s>\
  </td>\
</tr>\
<tr>\
  <td width=30%%>\
    Moble network APN:\
  </td>\
  <td>\
    <input type='text' name='APN' maxlength='30' size='30' value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td align=center>\
    <input type='submit' value='Save' />\
  </td>\
</tr>\
</table>\
</form>\
</body>\
</html>";
