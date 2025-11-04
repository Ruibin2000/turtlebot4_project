Commend to run in terminal, demo, publish the angles

```
source /opt/ros/jazzy/setup.bash
source ~/tb4_ws/install/setup.bash
python3 ~/test/my_reader.py
```


```
source ~/tb4_ws/install/setup.bash
python3 ~/test/my_reader_dual.py                # default 10 Hz
# or customize:
python3 ~/test/my_reader_dual.py --rate_hz 5 --aoa_topic /aoa_angle --snr_topic /snr_db
```