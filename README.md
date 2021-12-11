# Humanoid-walking   
   
#### Dynamixel SDK Install   
   
```bash   
git clone git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git   
cd python/   
sudo python3 setup.py install   
```   
   
#### Dynamixel SDK Instruction   
   
* Example `DynamixelSDK/python/tests/protocol1_0/read_write.py` for **AX-12A**

```python   
# The setting of AX-12
ADDR_MX_TORQUE_ENABLE     = 24              # line 52
ADDR_MX_GOAL_POSITION     = 30              # line 53
ADDR_MX_PRESENT_POSITION  = 36              # line 54

# Enter the value you set
DXL_ID                    = 38              # line 60
BAUDRATE                  = 1000000         # line 61
DEVICENAME                = '/dev/ttyUSB0'  # line 62

# Move between these values
DXL_MINIMUM_POSITION_VALUE = 358            # line 67
DXL_MAXIMUM_POSITION_VALUE = 440            # line 68
```   
