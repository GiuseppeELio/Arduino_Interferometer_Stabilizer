# Arduino Interferometer Stabilizer
An open hardware/source and low-cost interferometer stabilizer equipped with a digital lock-in amplifier. The Arduino board used for this device is Arduino GIGA R1 WiFi. 

![](Python_GUI_PID_ON_OFF/icon.ico)

The proposed firmware control a specific arduino Giga R1 shield designed to operate as function of the input signal coming from an interferometer where a piezo-electric mirror is direcly controlled by the Piezo DAC output (from the same board). Moreover, the Python GUI code can also be compiled to be a Standalone applications made for Win and Mac Os. To do that, install PyInstaller and then in the Python IDE/ terminal exectute:

! pyinstaller --windowed --name InterferometerStabilizer --icon=icon.icns InterferometerStabilizer_GUI_Py.py

It will create a .exe application if you are using Windows or an .app file if compiled by using Mac Os. 
 
Using one of the Matlab, Python or Standalone GUI you will obtaine a user friendly software like the one reported in the Figure below 
![](GUI_working_example/GUI.png)

There you can set all the useful parameters, Activate or Deactive the PID, collect your data, save them, print and display in the embedded plotter or analyzed them as showin in the next figure
![](GUI_working_example/GUI_analysis.png)


About the electronic and the board manufacturing, the schematics, the Gerber file, the BoM and the Pick and Place files are avaialble in the relative directories.  
