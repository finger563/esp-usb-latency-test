# Esp-Usb-Latency-Test

Code for performing latency test over USB for gamepad inputs. It performs hosted
measurement for end to end with the ESP which involves: actuate the button, then
measure the time it takes to receive the updated input report.

See also [esp-latency-test](https://github.com/finger563/esp-latency-test) for
end-to-end (with phone) latency testing using a photodiode or for doing BT / BLE
hosted latency testing.

This repository also contains a couple python analysis tools:
* [`analysis.py`](./analysis.py) can be used to plot a histogram of latency
  values that are measured from the system.
* [`multi-analysis.py`](./multi-analysis.py) can be used to analyze multiple
  controller analysis files simultaneously, plotting them in a single box-plot
  of latency vs battery life according to a meta-config file provided.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Esp-Usb-Latency-Test](#esp-usb-latency-test)
    - [Hardware Needed](#hardware-needed)
    - [Use](#use)
        - [Real-time Plotting](#real-time-plotting)
        - [Analysis](#analysis)
            - [Setup](#setup)
            - [Running](#running)
    - [Cloning](#cloning)
    - [Build and Flash](#build-and-flash)
    - [Output:](#output)

<!-- markdown-toc end -->

## Hardware Needed

1. ESP32S3 dev board - I recommend [esp32-s3-usb-otg
   devkit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/user_guide.html)
   ([mouser
   link](https://www.mouser.com/ProductDetail/Espressif-Systems/ESP32-S3-USB-OTG?qs=TCDPyi3sCW2REilQUpYpuw%3D%3D)).
   If you choose to go with a different dev-board, you will need to make sure it
   can provide VBUS power to the device under test. NOTE: for the
   `ESP32-S3-USB-OTG` devkit to provide VBUS power, you must plug it into a
   host, it will pass through the host power to the device. Otherwise you will
   need to solder on a battery to the devkit and enable the battery power using
   the on-board switch.
   * If you decide to use something like a ESP32-S3-DevKitC-1, then you'll need
     to modify the board (remove / bypass D7) so that the UART USB / VCC-5V can
     power the USB device port.
   * :warning: You'll need to modify the code if you use any board other than
     the one listed above! :warning:
2. Dupont wires to connect to button on controller (patch into button and gnd
   signal).
   
You'll need to wire up to the IO45 and GND pads in the free IO pads section of
the USB-OTG dev board:

![CleanShot 2024-07-08 at 15 51 07](https://github.com/finger563/esp-usb-latency-test/assets/213467/8317f67c-5a1d-46f9-bc5e-cd94c75f05be)
   
Test Setup:
![CleanShot 2024-07-08 at 15 53 44](https://github.com/finger563/esp-usb-latency-test/assets/213467/8c9906b2-3682-4577-829c-a9c382599a02)

Some controllers, such as 
* `8BitDo Pro 2` (note: it should be set to `D` compatibility setting)
* `Backbone One` (USB Receptacle)
* `Playstation Dualsense (model CFI-SCT1W)`
* `Nintendo Switch Pro Controller`
* `Xbox Elite Wireless Controller 2 (model 1797)` (note: currently doesn't work
  because it shows up as multiple usb devices)
* `Xbox Wireless Controller (model 1708)` (note: currently doesn't work because
  it shows up as multiple usb devices)

⚠️ Right now xbox controllers (elite 2 model 1797 and xbox model 1708) report `No
HID device at USB port 1`. See
https://github.com/finger563/esp-usb-latency-test/issues/4 for more information.

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

Note: In this screenshot I have not wired up the USB-OTG devkit to the
controllers, I'm simply plugging and unplugging the controllers via usb and then
pressing buttons manually.

![CleanShot 2024-07-08 at 10 55
42](https://github.com/finger563/esp-usb-latency-test/assets/213467/554bf27f-e216-47a6-b160-bd11d0ef842a)
