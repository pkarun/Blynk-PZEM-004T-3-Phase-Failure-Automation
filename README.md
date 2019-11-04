# Blynk-PZEM-004T-Phase-Failure-Automation
Blynk PZEM 004T 3 Phase failure motor automation.  

<h2>What it Does?</h2>
1) If voltage is lessthan minimum set value for all 3 Phase, then it turn OFF Relay 1</br> 
2) Monitors 3 PZEM 004T v3.0 device data with one ESP8266 device (multiple salve)</br> 
3) Auto Mode ON/OFF - With Auto Mode ON, it turn ON Relay 1 if it meets all 3 Phase voltage reaches minimum voltage value. If any those condition don't satisify, then it won't turn on</br> 
4) If voltage is lessthan minimum set value for all 3 Phase, then it turn OFF Relay 1 </br>
5) Show sum of voltage, current usage, active power, active energy, frequency and power factor</br>
6) You can update firmware using "HTTP Server OTA" method. In other words, through internet you can update firware without having physical access to device or without connecting to same network</br>
7) For other 3 Relays user can connect other devices to control On/Off through internet using blynk app </br>

<h2>Requirements</h2>
1) <a href ="http://s.click.aliexpress.com/e/ElytDjIu">3 PZEM-004T v3.0</a></br> 
2) <a href="http://s.click.aliexpress.com/e/nlefJ4PI">NodeMCU</a></br> 
3) <a href="http://s.click.aliexpress.com/e/eK05ynRS">4 Channel Relay Module</a></br> 
4) <a href="https://play.google.com/store/apps/details?id=cc.blynk">Blynk App</a></br> 

<h2>Installation</h2>
<ul>
<li>Open <code>secret.h</code> and change Bynk Auth code, Wifi settings, server settings and few other parameters as per your project requirement. </li>
<li>Open <code>settings.h</code> - Usually you don't need to change any values here, but if you need any customization feel free play with it.</li>
</ul>

<h2>Hardware Connection</h2>

<h3><b>PZEM-004T v3.0</b> to <b>NodeMCU</b></h3>

PZEM Device 1:</br> 

5v to vin</br> 
RX to D6 (has TX Pin)</br> 
TX to D5 (has RX Pin)</br> 
GND to GND</br> 

PZEM Device 2:</br> 

5v to vin</br> 
RX to D6 (has TX Pin)</br> 
TX to D5 (has TX Pin)</br> 
GND to GND</br> 

PZEM Device 3:</br> 

5v to vin</br> 
RX to D6 (has TX Pin)</br> 
TX to D5 (has TX Pin)</br> 
GND to GND</br> 

<h2>Software Setup</h2>

1) Download and install the Blynk Mobile App for iOS or Android.</br> 

2) Scan the QR code at the bottom of this page to clone the screenshot below, or create a new project yourself and manually arrange and setup the widgets.</br> 

3) Email yourself the Auth code.</br> 

4) Download this repo and copy the files in to your sketches directory. Open the sketch in Arduino IDE.</br> 

5) Go to the <code>settings.h</code> tab. This is where all the customisable settings are. You should be able to change almost everything from there before compiling.</br> 

6) Go to the <code>secret.h</code> tab. Here you change Bynk Auth code, Wifi settings, server settings and few other parameters as per your project requirement.

<h2>Screenshot</h2>
<img src="/images/1.png" alt="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 1" title="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 1" width="350" height="" style="max-width:100%;"></br>

<img src="/images/2.png" alt="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 2" title="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 2" width="350" height="" style="max-width:100%;"></br>

<img src="/images/3.png" alt="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 3" title="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 3" width="350" height="" style="max-width:100%;"></br>

<img src="/images/4.png" alt="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 4" title="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 4" width="350" height="" style="max-width:100%;"></br>

<img src="/images/5.png" alt="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 5" title="Blynk PZEM 004T 3 Phase Failure Automation Screenshot 5" width="350" height="" style="max-width:100%;"></br>

<h2>Scan QR Code on Blynk App</h2>

<img src="/images/blynk-qr-code.jpeg" alt="Blynk Project QR code" title="Blynk Project QR code" style="max-width:100%;"></a>
