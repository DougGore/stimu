# ST IMU Python Library for Raspberry Pi

IMUs are very useful but there can be quite a bit of work and understanding required to make them return usable results.

This library tries to reduce the pain of getting started to something fairly trivial after I spent a long time trying to make the ST LSM6DS3 work as I would expect.

**Note:** Only the LSM6DS3 is supported by this library at present. ST use a very similar approach and even matching registers on other chips so you may find this code adaptable.

## Getting started

This code hasn't been fully adapted into a library yet so for now the recommendation is to put the single `lsm6ds3.py` file directly into your project and import it like this:

~~~ python
from lsm6ds3 import IMU
~~~

You can execute the the file directly from Python and see a built in example of a gyroscope based compass:

~~~ bash
python3 lsm6ds3.py
~~~

## Usage

Create an instance of the IMU class:

~~~ python
imu = IMU()
~~~

Once your IMU instance is initialised it is recommended you run a gyroscope calibration.

~~~ python
imu.calibrate_gyro()
~~~

This sequence runs for 5 seconds to measure the gyroscope's drift, try to avoid any movement of your hardware during this time. Once this is complete the bias is automatically applied to readings on the gyroscope's Z axis. This helps greatly with tasks like tracking headings.

### Gyroscope compass

Here is an example of implementing a compass using just the gyroscope.

~~~ python
# Sleep period is the time to sleep between reads, tries to match the frequency of the gyroscope's configured sample rate
sleep_period = imu.gyro_sleep_period()

a = time.time()
heading = 0

while True:
    gyro_z = imu.gyroZ()

    # Time between gyro samples
    loop_period = time.time() - a
    a = time.time()

    # Add the gyroscope's degrees/sec scaled by sample time to the heading value
    heading += gyro_z * loop_period

    print("Gyro Z = {:.2f}, heading = {:.2f}".format(gyro_z, heading))

    sleep(sleep_period)
~~~

The above code takes regular gyroscope readings and adds them to a `heading` variable. This provides a basic compass relative to your starting position. This is useful for tracking a robot's rotation for example.