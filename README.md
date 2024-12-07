# Real-time Control Software for a Drone

This is an ESP32 flying drone project.

### Build And Flash

Must initialize an idf env or something.

Then, build (it's time to build) and flash. (instead of port put the actual usb port for esp32):

```zsh
idf.py build
ls /dev/cu.*
idf.py -p port flash
```

### Playing With Sensor Data

Currently I'm running the monitor command and piping the output to a text file:

```zsh
idf.py monitor | tee imu_data.txt
```

### Read and Plot

I have a python script to open that log file, extract values and produce plots.

See under `py_analyzer/`. Right now it's shitty but it'll be nice. IT WORKS!

### TO DO:

- Make separate tasks for IMU and Barometer, need different frequencies.
- Clean everything up
