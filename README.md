# LightTracker 1.1 433MHz

LightTracker 1.1 433MHz is one of the most affordable, smallest, lightest, powerful and open source LoRa APRS trackers available. It makes tracking pico balloons, weather balloons, model rockets, RC aircraft, and anything else that flies simple and easy.
It is able to report location, altitude, temperature and pressure to the internet via LoRa APRS gateways or direct to another LoRa radio module with a solar panel/super capacitors or just 3xAAA batteries.
Because LightTracker 1.1 433MHz is open source you can add your own custom sensors via I2C pins.

LightTracker 1.1 433MHz will ber available on https://shop.qrp-labs.com/aprs for order soon.

<img src="images/light_tracker_1_1-433-front_back.jpg" width="600">

**Important :** LightTracker 1.1 433 MHz may requires an amateur radio license to operate if 433 MHz is not ISM band in your country. So check regulations before using it.

![image](https://github.com/lightaprs/LightTracker-1.1-433/assets/48382675/5d6a126e-b77b-448b-a633-0a7e0fedc7f7)

## Basic Features

- **Software** : Open Source
- **Weight** : 5.21 grams
- **Dimensions** : 32 mm x 47 mm
- **IDE** : Arduino
- **Platform** : ARM Cortex-M0 (Arduino M0)
- **CPU** : ATSAMD21G18
- **Flash** : 256 kB
- **Ram** : 32 kB
- **Operating Frequency** : 48 Mhz
- **Operating Voltage** : 3.3 Volt
- **Input Voltage** : 2.7 (min) - 16 (max) Volt via usb or VBat pin
- **Sensor** : HP303B (pressure and temperature)
- **Power Consumption** : ~7 mA
- **LoRa Radio Module** : [EBYTE E22-400M22S](https://www.cdebyte.com/products/E22-400M22S) (SX1268)
- **LoRa Operating Frequency** : 410~493MHz (configurable by code)
- **LoRa Max Power** : 22dBm (configurable by code)
- **LoRa Power Consumption (TX)** : ~110 mA (22dBm)
- **GPS** : Ublox MAX-M8Q (GPS-GLONASS)
- **GPS Antenna Gain** : 4.3 dBi
- **Extended Pins** : I2C, 2x Analog, 1x DAC

<img src="images/light_tracker_1_1-433-weight.jpg" width="600">

## Configuration

To programme LightTracker, all you need is a micro usb (B type) cable, a few installations and configurations.

### 1.Install Arduino IDE

Download and install [Arduino IDE](https://www.arduino.cc/en/Main/Software). If you have already installed Arduino, please check for updates. Its version should be v1.8.13 or newer.

### 2.Configure Board

- Open the **Tools > Board > Boards Manager...** menu item as follows:

![image](https://user-images.githubusercontent.com/48382675/135890740-df30ddd3-ee2b-42b7-bc90-b30240cf5ee3.png)

- Type "Arduino SAMD" in the search bar until you see the **Arduino SAMD Boards (32-Bits Arm Cortex-M0+)** entry and click on it.

![image](https://user-images.githubusercontent.com/48382675/135891280-ad4eb226-dc00-4ff9-8332-a57fa986d16f.png)

- Click **Install** .
- After installation is complete, close the **Boards Manager** window.
- Open the **Tools > Board** menu item and select **Arduino SAMD Boards (32-Bits Arm Cortex-M0+) -> Arduino M0** from the the list as follows:

![image](https://user-images.githubusercontent.com/48382675/135892579-8fb214f0-07ad-485d-9aba-d51d7acf9a16.png)

### 3.Copy Libraries & Compile Source Code

You are almost ready to programme LightTracker 1.1 433MHz :)

- First download the repository to your computer using the green "[Code -> Download ZIP](https://github.com/lightaprs/LightTracker-1.1-433/archive/refs/heads/main.zip)" button and extract it.
- You will see more than one Arduino project optimized for different use cases. For example if you are planning to use LightTracker 1.1 433MHz for a pico balloon project, then use "[lora-aprs-pico-balloon-tracker](lora-aprs-pico-balloon-tracker)" folder or if you want to track your assets, vehicles, etc. then use "[lora-aprs-asset-tracker](lora-aprs-asset-tracker)" folder.
- You will also notice some folders in the "libraries" folder. You have to copy these folders (libraries) into your Arduino libraries folder on your computer. Path to your Arduino libraries:

  **Windows** : This PC\Documents\Arduino\libraries\
 
  **Mac** : /Users/\<username\>/Documents/Arduino/libraries/

- Copy all of them into your Arduino libraries folder as follows:

![copy-to-library](https://github.com/lightaprs/LightTracker-1.1-433/assets/48382675/540e9290-7f88-4602-8500-f8d4e08963ad)

- Then open the relevant sketch file (*.ino) with Arduino IDE and change your settings as described in Wiki pages and save it.
- Click **Verify**

### 4.Upload

- First attach an antenna to your tracker as if described in [Antenna Guide] LoRa radio module may be damaged if operated without attaching an antenna, since power has nowhere to go.
- Connect LightTracker to your computer with a micro USB cable.
- IYou should see a COM port under **Tools->Port** menu item. Select that port.

![image](https://user-images.githubusercontent.com/48382675/135892815-b129bd92-1d88-41e3-a943-dd61bd19f3e9.png)

- Click **Upload**
- Your tracker is ready to launch :)

## Support

If you have any questions or need support, please contact support@lightaprs.com

## Wiki

### General

* **[F.A.Q.]**
* **[Antenna Guide]**
* **[Tips & Tricks for Pico Balloons]**
