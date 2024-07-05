# Esp-Usb-Latency-Test

Code for performing end-to-end latency test for inputs. Can be configured in two ways:
1. ADC Measurement for end to end with a phone: actuate the button, then use a
   photo-diode/photo-transistor to measure when the screen changes and computes
   the time it took.
2. Hosted measurement for end to end with the ESP: actuate the button, then
   measure the time it takes to receive the updated input report.

See also [esp-latency-test](https://github.com/finger563/esp-latency-test)

This repository also contains a couple python analysis tools:
* [`analysis.py`](./analysis.py) can be used to plot a histogram of latency
  values that are measured from the system.
* [`multi-analysis.py`](./multi-analysis.py) can be used to analyze multiple
  controller analysis files simultaneously, plotting them in a single box-plot
  of latency vs battery life according to a meta-config file provided.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Esp-Usb-Latency-Test](#esp-usb-latency-test)
    - [Development](#development)
    - [Cloning](#cloning)
    - [Build and Flash](#build-and-flash)
    - [Output](#output)

<!-- markdown-toc end -->

## Hardware Needed

1. ESP32S3 dev board - ideally one with USB connectors for both a UART and the
   native USB so that you can gather log data more easily. I recommend the
   [esp32-s3-usb-otg
   devkit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/user_guide.html)
   ([mouser
   link](https://www.mouser.com/ProductDetail/Espressif-Systems/ESP32-S3-USB-OTG?qs=TCDPyi3sCW2REilQUpYpuw%3D%3D)).
   You will need to make sure it can provide VBUS power to the device under
   test. NOTE: for the `ESP32-S3-USB-OTG` devkit to provide VBUS power, you must
   plug it into a host, it will pass through the host power to the device.
   Otherwise you will need to solder on a battery to the devkit and enable the
   battery power using the on-board switch.
   * If you decide to use something like a ESP32-S3-DevKitC-1, then you'll need
     to modify the board (remove / bypass D7) so that the UART USB / VCC-5V can
     power the USB device port.
2. Dupont wires to connect to button on controller (patch into button and gnd
   signal).
   
Test Setup:
![image](https://github.com/finger563/esp-usb-latency-test/assets/213467/ea2a5b83-1ef8-4884-be31-db12847c7a41)

For measurement method (1/ADC) above, you'll also need:
3. Photo-diode for measuring the brightness / light of the screen. I used
   [Amazon 3mm flat head PhotoDiode](https://www.amazon.com/dp/B07VNSX74J).
4. Resistor (1k-10k) from photodiode output to ground.

If you're planning to run method (1/ADC) above, you'll likely need to run the
embedded code once with `CONFIG_DEBUG_PLOT_ALL` enabled (via menuconfig), so
that you can see the ADC values for the screen on/off state based on the screen
/ app / sensor you select and how you've mounted them. I use duct tape to
"mount" the sensor to my phone screen :sweat_smile:. Then you can configure the
appropriate upper/lower thresholds accordingly to take data.

Some controllers, such as 
* `8BitDo Pro 2` (note: it should be set to `D` compatibility setting)
* `Backbone One` (USB Receptacle)
* `Playstation Dualsense (model CFI-SCT1W)`
* `Xbox Elite Wireless Controller 2 (model 1797)` (note: currently doesn't work
  because it shows up as multiple usb devices)
* `Xbox Wireless Controller (model 1708)` (note: currently doesn't work because
  it shows up as multiple usb devices)
* `Nintendo Switch Pro Controller` (note: it appears we need to send some data
  to it, otherwise we just get a single report) see [additional
  information](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/USB-HID-Notes.md#80-04)

⚠️ Right now xbox controllers (elite 2 model 1797 and xbox model 1708) report `No
HID device at USB port 1`. I believe this is because they show up as multiple
devices. ⚠️

I believe the warning above is related to these issues:
* https://github.com/espressif/esp-idf/issues/12667
* https://github.com/espressif/esp-idf/issues/12554

## Use

It's recommended to use the
[uart_serial_plotter](https://github.com/esp-cpp/uart_serial_plotter) after
flashing to monitor and plot the latency values in real time. If you do this,
you can then also save the resultant output to a text file.

This text file can be loaded and parsed by the [`analysis.py`](./analysis.py)
script and the [`multi-analysis.py`](./multi-analysis.py) script.

### Real-time Plotting

You can use the
[uart_serial_plotter](https://github.com/esp-cpp/uart_serial_plotter) to plot
the latency values in real time.

``` sh
# follow setup / use instructions in esp-cpp/uart_serial_plotter repo
➜  uart_serial_plotter git:(master) $ source env/bin/activate
(env) ➜  uart_serial_plotter git:(master) $ python src/main.py
```

It will automatically find and open the serial port with the esp32 attached. If
there are multiple, you can use the `Serial` menu. to select another port.

If you want to save the recorded data to a file, you can use `File > Export UART
Data` command to save the data to a `txt` file.

### Analysis

#### Setup

These setup steps only need to be run the first time you set up the python
environment.

``` sh
# create the environment
➜  esp-usb-latency-test git:(main) $ python3 -m venv env

# activate the environment
➜  esp-usb-latency-test git:(main) $ source env/bin/activate

# install the dependencies (matplotlib, numpy)
(env) ➜  esp-usb-latency-test git:(main) $ pip install -r requirements.txt
```

#### Running

Any time you have a text file of csv data (such as what comes from the esp32
code), you can run the python script on it to generate a histogram.

``` sh
# This will run an interactive plot with matplotlib
(env) ➜  esp-usb-latency-test git:(main) $ python ./analysis.py tests/2024-05-30.txt

# This will simply save the output to the provided png file (destination folder must exist if provided)
(env) ➜  esp-usb-latency-test git:(main) $ python ./analysis.py tests/2024-05-30.txt --output output/2024-05-30.png

# you can also specify your own title
(env) ➜  esp-usb-latency-test git:(main) $ python ./analysis.py tests/2024-05-30-15ms-wake.txt --output output/2024-05-30-15ms-wake.png --title "Latency Histogram"
```

You can also run the `multi-analysis.py` script to analyze multiple controller
latency files at once. This script will generate a box plot of the latency
values for each controller, and a scatter plot of latency vs battery life.

``` sh
# this will simply load in the files listed in the meta-config (csv) file and plot them all
(env) ➜  esp-usb-latency-test git:(main) $ python ./multi-analysis.py hosted.csv

# you can optionally provide a title for the plot
(env) ➜  esp-usb-latency-test git:(main) $ python ./multi-analysis.py hosted.csv --title "Latency vs Battery Life"
```

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules git@github.com:finger563/esp-usb-latency-test
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build 

## Output:

Test Setup:
![image](https://github.com/finger563/esp-usb-latency-test/assets/213467/ea2a5b83-1ef8-4884-be31-db12847c7a41)

Xbox-Alike:
![CleanShot 2024-07-05 at 14 31 19](https://github.com/finger563/esp-usb-latency-test/assets/213467/d8f3aef3-4ed2-4c83-b50d-a073672c6dff)

PS5 DualSense:
![CleanShot 2024-07-05 at 14 33 03](https://github.com/finger563/esp-usb-latency-test/assets/213467/26dea419-55cd-478c-8f5d-a147761a1d53)
