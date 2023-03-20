# Motor Down Detection using ESC Telemetry

This code uses Telemetry from an Alpha ESC to detect potential motor failures from several different information. 

## 0) Requirements:
    This code uses the (PRDC_TMAESC)[https://github.com/PR-DC/PRDC_TMAESC] library. Please download it in your Arduino IDE (tested with v1.8.16).

## 1) Code

This is a two-part code:

### 1.1) **motorESCAlpha.ino** - should be used inside a microcontroller with the number of Serial ports equal to your drone's number of motors. It will be responsible for sending all the information above to the companion computer, for each motor:
    - motorIdx - Motor index
    - isMotorUpdated - Boolean that works like a heartbeat. If we don't receive information after a period of time, communication failed
    - timeArm_ms            - Time since ARM.
    - throttleReceivePercent- Desired throttle
    - throttleOutPercent    - Actual throttle
    - RPM                   - Motor RPM
    - voltage               - Voltage at the ESC
    - current               - Total current 
    - currentPhase          - Phase current
    - tempMOSFET            - Temperature at MOSFET
    - tempCap               - Capacitor temperature

    - Example output: 0, 1, 1500, 20.00, 19.99, 1200, 24.60, 1.000, 0.666, 45.00, 45.00
    - It means that motor in index 0 is updated (1), 1500ms has passed since ARM, throttle being sent is at 20.00%, actualy throttle performed is 19.99%, motor is spinning at 1200 RPM, and voltage at ESC is 24.6V. Total current is 1.0 A, phase current is 666mA, MOSFET and capacitor temperatures are at 45 degrees Celsius.

### 1.2) **motorESCAlpha.py** - should be used inside a companion computer, such as Raspberry Pi or NVIDIA Jetson. Will output an array of elements that can be either "normal" or "down", if a motor is considered not to be in perfect conditions. For the purpose of the tests made, only RPM was used, but all other data, specially motor current, could be better used to determine if a motor was down or not. 
    - Code also outputs a python dictionary with arrays of all the above data, parsed to JSON. Note how motor numbers begin with 1, for better readability, instead of 0, which is used in the code. Example output for motors only (sent all the time):

    {"motor": "1", "status": "normal"}
    {"motor": "2", "status": "normal"}
    {"motor": "3", "status": "normal"}
    {"motor": "4", "status": "normal"}
    {"motor": "5", "status": "normal"}
    {"motor": "6", "status": "down"}

    - Means Motor 6 is considered to be down, by at lest one if the criteria used in isMotorOK()
    - Example complete output (sent every self.REQUEST_DATA_EVERY_N_SECONDS seconds). Note that data is also parsed to JSON:
    - {
        "motorStatusList"       :   ["normal","normal","normal","normal","normal","down"]
        "throttleInPercentList" :   [20.00,20.00,20.00,20.00,20.00,20.00],
        "throttleOutPercentList":   [19.99,19.99,19.99,19.99,19.99,19.99],
        "motorRPMList"          :   [1200,1200,1200,1200,1200,1200],
        "voltageList"           :   [24.6,24.6,24.6,24.6,24.6,24.6],
        "totalCurrentList"      :   [1.000,1.000,1.000,1.000,1.000,1.000],
        "phaseCurrentList"      :   [0.666,0.666,0.666,0.666,0.666,0.666],
        "mosfetTempList"        :   [45.00,45.00,45.00,45.00,45.00,45.00],
    }
