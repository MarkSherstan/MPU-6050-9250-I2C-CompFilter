Setup as described [here](https://tutorials-raspberrypi.com/measuring-rotation-and-acceleration-raspberry-pi/).

```
sudo raspi-config
```
Enable SPI and I2C and reboot.

```
sudo nano /etc/modules
```

Make sure `i2c-bcm2708` and `i2c-dev` are both in the file. Reboot again.


```
sudo apt-get install i2c-tools python-smbus
```


```
sudo i2cdetect -y 1
```

Should yield:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --  
```

sudo i2cget -y 1 0x68 0x75
