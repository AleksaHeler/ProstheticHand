# Proof of concept for the prosthetic hand project

Here is the example of using the **ECG (electrocardiogram) sensor** for detecting activity of the muscle and sending filtered data to serial monitor. We're using wrong kind of sensor here because there wasn't any EMG (electromyogram) sensor available in our country, but soon we will test some EMG sensor once it arrives.

The **ECG_sensor_filtering.ino** Arduino sketch reads analog input voltage from the ECG sensor and filters it trough high pass filter to filter out the noise, then it makes the signal digital (converts from a threshold to binary value depending on if it is above th or below it), then a low pass filter which can be considered a [debounce](https://en.wikipedia.org/wiki/Switch#Contact_bounce), then again thresholding to make sure it is always 0 or 1. It works fairly well although it is far from perfect. This gives us hope that by using the right kind of sensor we will actually get clean data.
