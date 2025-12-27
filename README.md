# Dreamwave - Wireless Dreamcast Controller

![Prototype](/images/prototype.jpg)

Dreamwave turns any OEM Dreamcast controller into a wireless Bluetooth gamepad.

Built using a Raspberry Pi Pico 2 W, it can be used both on a Dreamcast console (with additional hardware) or on a PC.

*The project is currently a proof-of-concept, integrating minimally with [Pico2Maple](https://github.com/cluoma/pico2maple-fw) for use on Dreamcast, and on PC with the help of Steam Input.*

# What Works

* **PC**: controller input only
* **Dreamcast** (with [Pico2Maple](https://github.com/cluoma/pico2maple-fw)): controller input and VMU LCD display in slot 1

# How It Works

Dreamwave uses the RP2350's PIO to emulate the Dreamcast's Maple bus. It queries the controller for its current input state and sends this over Bluetooth HID to a host device.

For the VMU display, Dreamwave accepts requests to open a custom Bluetooth Classic L2CAP channel for data exchange between the controller and host.

# Pairing

On boot, Dreamwave will always try and reconnect to the last-connected host. If this fails, it will fallback into pairing mode after about a minute.

# Hardware

All one needs hardware-wise is a Pico 2 W, a battery, and some charging/power circuitry. Then it is just a matter of finding a way to attach everything to the Dreamcast controller.

I wanted a big battery and did not want to modify the controller too much. So, I settled on the following components stuffed into a small case that lives in the second controller slot:

* Raspberry Pi Pico 2 W
* 1500mAh LiPo battery
* Pimoroni LiPo Shim
* 3.3 to 5v boost converter
* Female JST 2.0mm connector

![Components](/images/components.png)
![Prototype Case](/images/prototype_case.png)

## Wiring

| Dreamcast Controller Pins (Left to Right) |                        |
|-------------------------------------------|------------------------|
| 1                                         | RP2350 Pin 21 (GPIO 16) |
| 2                                         | 5v                     |
| 3                                         | GND                    |
| 4                                         | GND                    |
| 5                                         | RP2350 Pin 22 (GPIO 17) |

Everything gets wired to the female JST header which can plug directly into the Dreamcast controller's PCB when wired the following way:

![Wiring](/images/wiring.png)


# Libraries

* [pico-sdk](https://github.com/raspberrypi/pico-sdk) - [BSD 3-Clause](https://github.com/raspberrypi/pico-sdk/blob/master/LICENSE.TXT)
* [btstack](https://github.com/bluekitchen/btstack) - [non-commercial](https://github.com/bluekitchen/btstack/blob/master/LICENSE), [pico commercial exception](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP)
* [cyw43-driver](https://github.com/georgerobotics/cyw43-driver) - [License](https://github.com/georgerobotics/cyw43-driver/blob/main/LICENSE.RP)
