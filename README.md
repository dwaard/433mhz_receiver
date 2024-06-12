# 433mhz_receiver

Arduino project that receives 433mhz data from wireless temperature and humidity sensors and sends it to ThingSpeak.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

- A [ThingSpeak](https://thingspeak.com/) account
- The [ThingSpeak Communication Library for Arduino, ESP8266 and ESP32](https://github.com/mathworks/thingspeak-arduino)

### Installing

A step by step series of examples that tell you how to get a development env running

1. Create a `secrets.h` file (copy from the example)
1. Fill out your wifi credentials
1. Create a channel on ThingSpeak
1. Enable the correct amount of fields. Check the code for the name of each field
1. Add the channel number and write api key to the `secrets.h` file

## Authors

* **Daan de Waard** - *Initial work* - [dwaard](https://github.com/dwaard)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Cornelius'article on [One Transistor](https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html). I used this a lot for basic understanding of the signals and the code.
