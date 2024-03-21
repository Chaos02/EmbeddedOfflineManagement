# EmbeddedOfflineManagement

This Program is meant for running in an "embedded" System. 

What I mean by embedded is that the entire assembly essentially mimics a very expandable laptop. The Solution contains:
 - 1 x86 computer with an SMBus interface accessed by a management GUI application
 - 1+ SMBus capable BMS along with batteries
 - 1 micro controller that runs at all times for monitoring the entire embedded system state (cooling + battery temperatures, charging status, etc.)

This Repo contains the program of said micro controller and was developed to run on an Arduino Leonardo.

The project never got finished as it was ultimately scrapped but this repo contains about 60% of the total programming of the project.
It's meant to use:
 - The [Cooperative Multitasking library](https://bitbucket.org/amotzek/cooperative-multitasking/src/master/) from Andreas Motzek
 - The [Arduino Interpolation library](https://github.com/luisllamasbinaburo/Arduino-Interpolation) from Luis Llamas
 - My [SMBus + SmartBattery Arduino "library"](https://github.com/Chaos02/Arduino-Wire-SMBus-API)
 - My [MiAPI C# interface](https://github.com/Chaos02/MiAPI-CSharp-Translator) as the "online" (x86) Management interface.

Make sure to copy the contents of src/lib to your IDE's Arduino library folder.

I have detailed Connection and System Diagrams aswell as writte documentation but "publishing" them here would require more work than I want to put into this at the moment.
If you are interested you can contact me via GitHub, I'm sure I can share most of it.

## State of Program

 - It already works as a temperature monitor and fan controller.
 - SmartBattery Monitoring is WIP but about 90% done. The alert callback function is not ready yet.
 - Communication between the main computer and this manager is not fully fleshed out.
   There's a naming-system for property-reporting via a Serial connection to the controller but the goal was to use a better suitable method like SMBus.


### Good To Know (ramble-Notepad during project)

Resistance "in front" of VinPin: U/RI -> 15V/(R*0,04A) -> 375 Ohm at 0,8W ... meh...
Voltage divider for USB PD (EPR) to 5,5Vmax R values: 10700Ohm, 1300Ohm (goes up to 50,75V)

29.0 Electrical Characteristics
    Voltage on any Pin except RESET and VBUS
    with respect to Ground (8)                       -0.5V to VCC +0.5V

    DC Current per I/O Pin                                       40.0mA
    DC Current V CC and GND Pins                                200.0mA

24.6 Temperature Sensor
    The ATmega16U4/ATmega32U4 includes an on-chip temperature sensor, whose the value can be read through
    the A/D Converter.
    The temperature measurement is based on an on-chip temperature sensor that is coupled to a single ended
    ADC input. MUX[5..0] bits in ADMUX register enables the temperature sensor. The internal 2.56V voltage
    reference must also be selected for the ADC voltage reference source in he temperature sensor measurement.
    When the temperature sensor is enabled, the ADC converter can be used in single conversion mode to
    measure the voltage over the temperature sensor.
    The temperature sensor and its internal driver are enabled when ADMUX value selects the temperature sensor
    as ADC input. The propagation delay of this driver is approximately 2μS. Therefore two successive conversions
    are required. The correct temperature measurement will be the second one.
    One can also reduce this timing to one conversion by setting the ADMUX during the previous conversion.
    Indeed the ADMUX can be programmed to select the temperature sensor just after the beginning of the
    previous conversion start event and then the driver will be enabled 2μS before sampling and hold phase of
    temperature sensor measurement.

#### SMBus adresses

| Slave Address | Description                                              | Specification                                                                                                                              |
| ------------- | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 0001 000      | SMBus Host                                               | System Management Bus Specification, version 1.1 December 1998                                                                             |
| 0001 001      | Smart Battery Charger                                    | Smart Battery Charger Specification, version 1.1 December 1998                                                                             |
| 0001 010      | Smart Battery Selector<br />Smart Battery System Manager | Smart Battery Selector Specification, version 1.1 December 1998<br /> Smart Battery System Manager Specification, version 1.0B August 1999 |
| 0001 011      | Smart Battery                                            | Smart Battery Data Specification, version 1.1 December 1998                                                                                |
| 0001 100      | SMBus Alert Response                                     | System Management Bus Specification, version 1.1 December 1998                                                                             |
| 0101 000      | ACCESS.bus host                                          |                                                                                                                                            |
| 0110 111      | ACCESS.bus default address                               |                                                                                                                                            |
| 1001 0XX      | Unrestricted addresses                                   | System Management Bus Specification, version 1.1 December 1998                                                                             |
| 1100 001      | SMBus device default address                             |    System Management Bus Specification, version 1.1 December 1998                                                                                                                                        |

