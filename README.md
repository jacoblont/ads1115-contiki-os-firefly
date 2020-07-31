# ads1115-contiki-os-firefly
ADS1115 library for [Contiki-OS](https://github.com/contiki-os/contiki) on the Zolertia Firefly board.

I created a library to use the Adafruit ADS1115 16-bit ADC in combination with the Zolertia Firefly node running Contiki-OS.
There are some ADS1115 specific things in the code, but I am sure that also the 12-bit ADS1015 can be used without having to make too much modifications.

### Contiki is a submodule of the repository ###
Please use the following command two times, within the contiki folder to get the contiki files after cloning the repository

`git submodule update --init`

The command `git submodule update --init --recursive` should also work.

### How to compile? ###
First go to the application using `cd /code/apps/test-ads1115`

Next, run `make test-ads1115.upload && make login` and you should soon see two sensor values appearing in your terminal when the compilation process is done.

### Issues with the serialdump executable ###
Solution: recompile serialdump within the contiki/tools/sky folder and rename the executable to serialdump-linux (first remove the old version).

### Disclaimer ###
Please note that I made the code for my specific purpose. Everything works fine and everything is there to use an ADS1115 on the Firefly, but you may need to modify some things to make it work for your project.
Please take a look at the datasheet of the ADC and make sure that you use the right (current sensing) resistors for your application to prevent blowing up your ADC. The resisitors on the I2C bus should not have to be changed.

I take no responsibility for any damages caused by using the code and give no warranties in respect of using this code.

### Hardware connections ###
The ADS1115 and the Firefly communicate over I2C, so a I2C bus needs to be created. I created the following scheme to connect the two devices:

![I2C connections between the ADC and the Firefly](/readme_image/i2c_connections.png "I2C connections image")

Since the Firefly runs on 3.3V, I chose to make the I2C bus 3.3V, which is also why the ADS1115 is powered using 3.3V.
Again, the current sensing resistors (R3 and R4 in the image) are chosen to meet the signals I use in my project. You may need to change these for your project to make sure you do not blow up the ADC and also to make sure that you can use the full 16 bit resolution (if you want, of course).

### Based on ###
Some parts of the code are based on or inspired by the [Adafruit ADS1x15 library](https://github.com/adafruit/Adafruit_ADS1X15).
