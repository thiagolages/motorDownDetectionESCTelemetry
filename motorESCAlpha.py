# Interpret ESC data reading from Teensy, or from a similar microcontroller, running the motorESCAlpha.ino code
# Should be used by a companion computer connected to Teensy via USB Cable, and Teensy connected to T-Motor Alpha ESC via White Cable (Serial) !
# This code will send an array of motor status. Each element can be either "normal" or "down", depending on some conditions.
# Author: Thiago Lages (github.com/thiagolages)

# This code is capable of analyzing:

# isMotorUpdatedList    - whether we have received that from a specific motor or not (boolean)
# throttleInPercentList - desired throttle (%)
# throttleOutPercentList- actual throttle (%)
# motorRPMList          - RPM from each motor 
# voltageList           - voltage at each Alpha ESC (V)
# totalCurrentList      - total current (A)
# phaseCurrentList      - phase current (A)
# mosfetTempList        - temperature of Alpha ESC's MOSFET (degree Celsius)
# capacitorTempList     - capacitor temperature inside Alpha ESC (degree Celsius)

# It's possible to define MIN and MAX values for each property, and detect if a motor has failed or not
# It's also possible to enable/disable each property check, inside the function isMotorOK()

# This code will also print data related to all the above properties once every self.REQUEST_DATA_EVERY_N_SECONDS seconds

# The data is a dictionary, as follows, and can be easily parsed to JSON:

# self.dataToAirCraft = { 
#     "motorStatusList"       :   self.motorStatusList,       # "normal" or "down"
#     "throttleInPercentList" :   self.throttleInPercentList, # throttle being sent to motors
#     "throttleOutPercentList":   self.throttleOutPercentList,# throttle actually being performed
#     "voltageList"           :   self.voltageList,           # voltage on each ESC
#     "phaseCurrentList"      :   self.phaseCurrentList,      # phase current on each motor
#     "motorRPMList"          :   self.motorRPMList,          # motor RPM
#     "totalCurrentList"      :   self.totalCurrentList,      # total current on each motor
#     "mosfetTempList"        :   self.mosfetTempList         # temperature of each ESC MOSFET
# }

# Author: Thiago Lages (github.com/thiagolages)

from numpy import inf
import serial, json, logging, time
from logging.handlers import RotatingFileHandler

# configure logging (https://stackoverflow.com/questions/57622296/python-logging-only-to-console-not-to-file)
maxBytesLogFile            = 10*1024*1024                          # max file size in bytes. Set to 10 MB
backupCount                = 2                                     # max number of backup log files to keep

# get default logger
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# remove all default handlers
for handler in logger.handlers:
    logger.removeHandler(handler)

# create new handler
handle = RotatingFileHandler('log/motor.log', mode='a', maxBytes=maxBytesLogFile, backupCount=backupCount, encoding=None, delay=0)

# create formatter
logFormatter = logging.Formatter("%(asctime)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s")
handle.setFormatter(logFormatter)

# now add new handler to logger
logger.addHandler(handle)

class Motor:

    # class constructor
    def __init__(self):
        # motor feedback config
        self.frequencySendFullTelemetry = 2.0                                   # frequency at which we will send full telemetry to CCS

        # serial port configs
        self.ser                        = None                                  # 'serial' object
        self.serialPort                 = '/dev/ttyACM0'                        # serial port
        self.serialBaudRate             = 1000000                               # serial baud rate
        self.serialTimeout              = 2                                     # timeout in sec to wait for data from teensy

        # useful variables
        self.dataFromTeensy             = None                                  # data received from teensy
        self.dataToAirCraft             = {}                                    # dict containing motor data that will be sent to aircraft via Python Shell
        self.totalNumMotors             = 6                                     # total number of motors of the aircraft
        self.bytesToSendToTeensy        = str.encode('0'*self.totalNumMotors)   # data to send to teensy so it sends back motor info IN ORDER, which is important. it needs to be 'totalNumMotors' zeros
        self.serialFailCount            = 0                                     # counter on serial failures 
        self.maxTimeNoResponseSerial    = 5.0                                   # max number of fails accepted when trying to connect via serial
        self.numOfDataReceived          = 10+1                                  # this should be equal to the number of data received (for each motor) PLUS 1 (which is the motorNumbe itself)
        # Obs.: numOfDataReceived = numMotor + ((i+1), timeArm_ms[i], throttleReceivePercent[i], throttleOutPercent[i], RPM[i], voltage[i], current[i], currentPhase[i], tempMOSFET[i], tempCap[i])

        #######################################################################################################################################
        ### Data that will be received from Teensy. If the number of data sent by Teensy changes, please change 'self.numOfDataReceived' !! ###
        #######################################################################################################################################
        self.timeList                   = [None] *self.totalNumMotors # 1
        self.isMotorUpdatedList         = [False]*self.totalNumMotors # 2
        self.throttleInPercentList      = [None] *self.totalNumMotors # 3
        self.throttleOutPercentList     = [None] *self.totalNumMotors # 4
        self.motorRPMList               = [None] *self.totalNumMotors # 5
        self.voltageList                = [None] *self.totalNumMotors # 6
        self.totalCurrentList           = [None] *self.totalNumMotors # 7
        self.phaseCurrentList           = [None] *self.totalNumMotors # 8
        self.mosfetTempList             = [None] *self.totalNumMotors # 9
        self.capacitorTempList          = [None] *self.totalNumMotors # 10
                                                                      # +1, which is motor number
        #######################################################################################################################################
        
        self.motorStatusList            = [None]*self.totalNumMotors    # will be inferred based on previous data

        # variables to check for failures and to send/receive data
        self.lastTimeSerialOK           = time.time()
        self.lasTimeHeartbeat           = time.time()                           # last time we received something from Teensy
        self.maxTimeNoHeartbeat         = self.maxTimeNoResponseSerial          # max acceptable time with no response at all from Teensy
        
        self.lastTimeSendFullTelemetry  = 0.0                                   # last time we've sent telemetry data to CCS
        self.maxTimeBetweenTelemetry    = 1.0/self.frequencySendFullTelemetry   # time interval (sec) between sending full motor telemetry to CCS

        self.lastTimeMotorUpdate        = [time.time()]*self.totalNumMotors     # last time we've received any kind of info from each motor
        self.maxTimeNoUpdate            = self.maxTimeNoHeartbeat               # max time with no response from a specific motor

        # The values shown below were measured on 1 DLV-1 motor on 08/09/2022
        # The values represent MAX and MIN values for ONE motor, not for all motors
        # RPM
        self.MAX_RPM_THRESHOLD          =  10000 # when motors are fast
        self.MIN_RPM_THRESHOLD          =   350 # when motors are slow
        # Phase Current
        self.MAX_PHASE_CURR_THRESHOLD   =     9 # Amps
        self.MIN_PHASE_CURR_THRESHOLD   =  0.18 # Amps
        # Total Current
        self.MAX_TOTAL_CURR_THRESHOLD   =    18 # Amps
        self.MIN_TOTAL_CURR_THRESHOLD   =   1.4 # Amps
        # Voltage
        self.MAX_VOLTAGE_THRESHOLD      =  25.2 # Volts (Alpha ESC website)
        self.MIN_VOLTAGE_THRESHOLD      =    18 # Volts (Alpha ESC website)
        # Temperature (MOSFET from ESC)
        self.MAX_TEMP_MOSFET_THRESHOLD  =    75 # degrees Celsius
        self.MIN_TEMP_MOSFET_THRESHOLD  =    20 # degrees Celsius
        
        self.CHECK_MOTORS_FREQUENCY_HZ      = 10 # frequency at which we will check the motors
        self.REQUEST_DATA_EVERY_N_SECONDS   = 1.0/self.CHECK_MOTORS_FREQUENCY_HZ    # seconds
    
    def getDataCallback(self):
        try:
            if (not self.readDataFromTeensy())  : return # if we had any problems with teensy               , return, else keep going
            self.isMotorsUpdateOK()
            if (not self.updateData())          : return # if we had any problems updating data from ESCs   , return, else keep going

            self.prepareAndSendDataToAirCraft()

        except Exception as e: 
            if ((time.time() - self.lasTimeHeartbeat) > self.maxTimeNoHeartbeat):
                ## NO COMMUNICATION WITH TEENSY !!
                self.sendCommunicationFailed()
                logger.debug("No heartbeat from Teensy for {} seconds !!!".format(self.maxTimeNoHeartbeat))
            
            logger.debug(e)

    def readDataFromTeensy(self):
        
        self.ser.reset_input_buffer()                           # clear buffer before reading to avoid any old data received previously
        self.ser.write(str.encode(self.bytesToSendToTeensy))    # write 'totalNumMotors' zeroes into teensy serial so it outputs at least info from one motor

        self.dataFromTeensy             = self.ser.readline()   # read data from Teensy
        
        # decode bytes to string and remove newline ('\n') and carriage return ('\r') characters 
        self.dataStringList             = self.dataFromTeensy.decode("utf-8").replace('\r','').replace('\n','').split(",") 
        
        if (len(self.dataStringList) != self.numOfDataReceived):
            logger.debug("Number of data received is not equal to {} ! It's probably a Status Code from Alpha ESC's. Data received = {}".format(self.numOfDataReceived, self.dataFromTeensy))
            return False
        else:
            self.lasTimeHeartbeat           = time.time()           # indicate we have received something from Teensy (so communication is OK)
            return True

    def updateData(self):
        # using map() to convert all strings to float, and then converting them to a list()
        motorNum,isMotorUpdated,timeMotor,throttleInPercent,throttleOutPercent,motorRPM,voltage,totalCurrent,phaseCurrent,mosfetTemp,capacitorTemp = list(map(float, self.dataStringList))
        
        motorNum = int(motorNum)    # index from 0 to self.totalNumMotors-1
        if (motorNum >= 0 and motorNum < self.totalNumMotors):
            self.timeList               [motorNum] = timeMotor
            self.isMotorUpdatedList     [motorNum] = bool(int(isMotorUpdated))
            self.throttleInPercentList  [motorNum] = throttleInPercent
            self.throttleOutPercentList [motorNum] = throttleOutPercent
            self.motorRPMList           [motorNum] = motorRPM
            self.voltageList            [motorNum] = voltage
            self.totalCurrentList       [motorNum] = totalCurrent
            self.phaseCurrentList       [motorNum] = phaseCurrent
            self.mosfetTempList         [motorNum] = mosfetTemp
            self.capacitorTempList      [motorNum] = capacitorTemp              
            
            # This HAS to come after all the above attribuitions, since isMotorOK checks for current, voltage, RPM, etc.
            self.motorStatusList        [motorNum] = "normal" if self.isMotorOK(motorNum) else "down"

            # update time after we've received and updated all values
            if (self.isMotorUpdatedList[motorNum] == True):
                self.lastTimeMotorUpdate     [motorNum] = time.time()

            return True
        else:
            return False

    def isMotorsUpdateOK(self):
        # If any motor hasn't been updated for some time, we need to send a warning
        for motorNum, isMotorUpdated in enumerate(self.isMotorUpdatedList):
            if (isMotorUpdated == False and time.time() - self.lastTimeMotorUpdate[motorNum] >= self.maxTimeNoUpdate):
                self.sendCommunicationFailed()
                return False
        
        # if everything is OK, return True (meaning no data is repeated)
        return True    

    def isMotorOK(self, motorNum):
        if (motorNum < 0 or motorNum >= self.totalNumMotors):
            logger.debug("Motor Num invalid ! Received {0}, and it should be from 0 to {1} !".format(motorNum, self.totalNumMotors-1))
            return False

        answer = True # motor status is OK unless something changes it to False
        if (not self.isMotorRPMOK           (motorNum)): answer = False # check motor RPM limits and updates motors last RPM and check for repetitions
        #if (not self.isMotorVoltageOK      (motorNum)): answer = False # checks motor voltage limits
        #if (not self.isTotalMotorCurrentOK (motorNum)): answer = False # checks motor total current limits
        #if (not self.isPhaseMotorCurrentOK (motorNum)): answer = False # checks motor phase current limits
        #if (not self.isMotorTemperatureOK  (motorNum)): answer = False # checks MOSFET temperature limits

        return answer
    
    def isMotorRPMOK(self, motorNum):
        if (self.isMotorUpdatedList[motorNum]):
            motorRPM        = self.motorRPMList[motorNum] 
            return self.isWithinMaxMinLimits(motorRPM, self.MIN_RPM_THRESHOLD, self.MAX_RPM_THRESHOLD)
        else: # ignore if we don't have updated data
            return True

    def isMotorVoltageOK(self, motorNum):
        if (self.isMotorUpdatedList[motorNum]):
            voltage = self.voltageList[motorNum]
            return self.isWithinMaxMinLimits(voltage, self.MIN_VOLTAGE_THRESHOLD, self.MAX_VOLTAGE_THRESHOLD)
        else: # ignore if we don't have updated data
            return True

    def isTotalMotorCurrentOK(self, motorNum):
        if (self.isMotorUpdatedList[motorNum]):
            totalCurrent = self.totalCurrentList[motorNum]
            return self.isWithinMaxMinLimits(totalCurrent, self.MIN_TOTAL_CURR_THRESHOLD, self.MAX_TOTAL_CURR_THRESHOLD)
        else: # ignore if we don't have updated data
            return True

    def isPhaseMotorCurrentOK(self, motorNum):
        if (self.isMotorUpdatedList[motorNum]):
            phaseCurrent = self.phaseCurrentList[motorNum]
            return self.isWithinMaxMinLimits(phaseCurrent, self.MIN_PHASE_CURR_THRESHOLD, self.MAX_PHASE_CURR_THRESHOLD)
        else: # ignore if we don't have updated data
            return True

    def isMotorTemperatureOK(self, motorNum):
        if (self.isMotorUpdatedList[motorNum]):
            MOSFETtemp = self.mosfetTempList[motorNum]
            return self.isWithinMaxMinLimits(MOSFETtemp, self.MIN_TEMP_MOSFET_THRESHOLD, self.MAX_TEMP_MOSFET_THRESHOLD)
        else: # ignore if we don't have updated data
            return True

    def isWithinMaxMinLimits(self, value, min, max):
        if (value < min or value > max):
            return False
        else:
            return True

    def prepareAndSendDataToAirCraft(self):
        # send all data from all motors from time to time
        if ((time.time() - self.lastTimeSendFullTelemetry) >= self.maxTimeBetweenTelemetry):
            # create data dict that will be sent (as JSON)
            self.dataToAirCraft = { 
                    "motorStatusList"       :   self.motorStatusList,       # "normal" or "down"
                    "throttleInPercentList" :   self.throttleInPercentList, # throttle being sent to motors
                    "throttleOutPercentList":   self.throttleOutPercentList,# throttle actually being performed
                    "voltageList"           :   self.voltageList,           # voltage on each ESC
                    "phaseCurrentList"      :   self.phaseCurrentList,      # phase current on each motor
                    "motorRPMList"          :   self.motorRPMList,          # motor RPM
                    "totalCurrentList"      :   self.totalCurrentList,      # total current on each motor
                    "mosfetTempList"        :   self.mosfetTempList         # temperature of each ESC MOSFET
                }
        else: # only send status data
            self.dataToAirCraft = None
            # self.dataToAirCraft = {
            #         "motorStatusList"   :   self.motorStatusList,       # "normal" or "down"
            #     }

        # check if we have data from all motors loaded at least once
        if (None not in self.motorStatusList):
            self.sendToAircraft(self.dataToAirCraft)
        else:
            logger.debug("Not all motors are loaded yet")

    def sendToAircraft(self, dataToAirCraft):     
        if (dataToAirCraft == None): # only send motor status (compatible with yellow cable code)
            for i in range(self.totalNumMotors):
                msg = json.dumps({"motor": str(i+1), "status": self.motorStatusList[i]},separators=(",",":"))
                print(msg)
                logger.debug(msg)
            return # don't go further
        else:
            try:
                msg = json.dumps(dataToAirCraft,separators=(",",":"))
                print(msg)
                self.lastTimeSendFullTelemetry = time.time() # update when data was sent to Aircraft
                logger.debug(msg)
            except Exception as e:
                logger.debug(e)

    def printAllMotors(self):
        for motorNum in range(self.totalNumMotors):
            print("{}\t|{}\t|{}\t|{}\t|{}\t|{}\t|{}\t|{}\t|{}\t|{}".format(
                motorNum,
                self.timeList[motorNum],
                self.throttleInPercentList[motorNum],
                self.throttleOutPercentList[motorNum],
                self.motorRPMList[motorNum],
                self.voltageList[motorNum],
                self.totalCurrentList[motorNum],
                self.phaseCurrentList[motorNum],
                self.mosfetTempList[motorNum],
                self.capacitorTempList[motorNum]))
    
    def sendCommunicationFailed(self):
        self.sendToAircraft({"status": "error", "message": "Failed to communicate with Teensy."})

    def run(self):  # run get self.dataFromTeensy loop
        # connect to Serial before anything else
        while self.ser == None:
            try:
                self.ser = serial.Serial(self.serialPort, self.serialBaudRate)
                logger.debug("Connected to serial {}".format(self.serialPort))
                self.lastTimeSerialOK = time.time()
            except Exception as e:
                logger.debug(e)
                self.serialFailCount = self.serialFailCount+1
                if(time.time() - self.lastTimeSerialOK >= self.maxTimeNoResponseSerial):
                    self.sendCommunicationFailed()
                    logger.debug("Couldn't connect to serial !")
                    time.sleep(0.5)

        # if everything is OK
        while True:
            lastTimeGetCallback = time.time()

            self.getDataCallback()

            timePassed = time.time() - lastTimeGetCallback
            time.sleep(self.REQUEST_DATA_EVERY_N_SECONDS - timePassed) # to make the frequency of data acquisition more accurate

if __name__ == "__main__":  
    try:
        logger.debug("################## Monitoring started ###################")
        motor = Motor()
        motor.run()
    except Exception as e:
        if ((time.time() - motor.lasTimeHeartbeat) > motor.maxTimeNoHeartbeat):
            ## NO COMMUNICATION WITH TEENSY !!
            motor.sendCommunicationFailed()
            logger.debug("No heartbeat from Teensy for {} seconds !!!".format(motor.maxTimeNoHeartbeat))
        logger.debug(e)
