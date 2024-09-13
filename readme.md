# RKade Wheel

Modified fork of vsulako/AFFBWheel, adapted for Happ/Midway original belt-driven arcade wheel and motor for use with Windows-based multi-cabs (Cruisin Exotica and NFS cabs in my case).
So far, this works with some older and newer PC Games with FFB, Teknoparrot, mame, demul, M2, SuperModel3, Flycast and other emulators supported by FFBArcadePlugin (https://github.com/Boomslangnz/FFBArcadePlugin/releases).

## Wiring Schematic

![RKadeWheel](https://github.com/user-attachments/assets/42a19ffd-b6f9-4e99-b6d0-026121c9d7a7)

## Instructions for the firmware

1. Download the project from github as a zip, and unzip the archive to any folder.
2. Download [Arduino IDE](https://www.arduino.cc/en/software). Version 2.3.2 (current latest) has been tested.
3. Download additional libraries:
   - [digitalWriteFast](https://github.com/NicksonYap/digitalWriteFast)
   - [avdweb_AnalogReadFast](https://github.com/avandalen/avdweb_AnalogReadFast)
4. Install the downloaded libraries:
   - 4.1. Download the library as *zip*.
   - 4.2. Open *Arduino IDE*, then click **Sketch > Include Library > Add .ZIP Library....** and select *zip* with downloaded library. 
5. Open the folder where you unpacked the project archive, go to the RKadeWheel folder and open the RKadeWheel file in Arduino IDE.
6. Make the necessary changes to the config.h file for customization.
7. Connect the *Arduino* board to your PC.
8. Select your board type *Arduino* **Tools > Board** (Leonardo, ProMicro, Micro, etc.) 
9. Select the port on which *Arduino* is detected
10. Click the upload button: 
11. Wait for the firmware process to finish.
