# ESP32 Low Power Impulse Logger
This project aims to use the ESP32 PCNT peripheral to count impulses even when the processor is asleep.

The original idea was to create a low power data logger for electricity meters, which writes the data to an SD card.
In this way, consumption can simply be recorded for a transitional period without any additional infrastructure (ESP can be batter powered for some months, and without a database) in order to be able to better predict the effects of a PV system, a flexible electricity tariff and/or a heat pump for a specific household.

However currently, I doesn't need the low-power aspect and the SD card support, but MQTT and ModBus.

The count is read out regularly and converted into an average power (watts) since the last readout.
The power value is then published via MQTT and/or provided in a ModBus-Server in the Holding Register 1000 as int16.

This Project can also be used for other Meters that provide some pulses per energy unit, like Gas or Water Meters.

# Hardware
A Sensor is needed that convert the meterpulses into a digital signal, that the ESP can read.
It is highly recommended to use a lowpass filter in order to debounce the digital signal as the PCNT input can be very sensitiv.

# Config
First, the include/config.h.tpl needs to be copied to include/config.h where you can configure your setup.
In order to calculate the power, the impulses per kWh has to be defined for the specific meter.