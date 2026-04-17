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

1. Create a `secrets.h` file in the src folder (copy from the example)
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


```
PASSKEY=71AE3B89C1BD5392E7D5057C250D2868&
stationtype=EasyWeatherPro_V5.2.6&
runtime=765869&
heap=22516&
dateutc=2026-04-10+10:58:09&
tempinf=69.6&
humidityin=40&
baromrelin=29.684&
baromabsin=29.232&
tempf=43.2&
humidity=46&
winddir=156&
windspeedmph=0.22&
windgustmph=1.12&
maxdailygust=6.93&
solarradiation=243.13&
uv=2&
rainratein=0.000&
eventrainin=0.000&
hourlyrainin=0.000&
dailyrainin=0.000&
weeklyrainin=0.280&
monthlyrainin=0.350&
yearlyrainin=1.382&
totalrainin=36.311&
vpd=0.151&
wh65batt=0&
freq=868M&
model=WS2900_V2.02.02&
interval=60
```