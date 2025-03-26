#!/usr/bin/python

def he_control_loop(dummy, state,timeState):
    from time import sleep
    from datetime import datetime, timedelta
    import RPi.GPIO as GPIO
    import config as conf

    GPIO.setwarnings(False)
    GPIO.cleanup()  # Add this line to ensure clean state
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(conf.he_pin, GPIO.OUT)
    GPIO.output(conf.he_pin, 0)
    heating = False

    try:
        while True:
            pidstate['awake'] = timer.timer(timeState)

            avgpid = state['avgpid']
            
            if not state['awake'] or state['circuitBreaker']:
                state['heating'] = False
                GPIO.output(conf.he_pin, 0)
                sleep(1)
            else:
                if avgpid >= 100:
                    state['heating'] = True
                    GPIO.output(conf.he_pin, 1)
                    sleep(1)
                elif avgpid > 0 and avgpid < 100:
                    state['heating'] = True
                    GPIO.output(conf.he_pin, 1)
                    sleep(avgpid/100.)
                    GPIO.output(conf.he_pin, 0)
                    sleep(1-(avgpid/100.))
                    state['heating'] = False
                else:
                    GPIO.output(conf.he_pin, 0)
                    state['heating'] = False
                    sleep(1)

    finally:
        GPIO.output(conf.he_pin, 0)
        GPIO.cleanup()


def pid_loop(dummy, state):
    from time import sleep, time
    from math import isnan
    import adafruit_max31855
    import PID as PID
    import config as conf
    from datetime import datetime
    import RPi.GPIO as GPIO
    from digitalio import DigitalInOut
    import board

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    def c_to_f(c):
        return c * 9.0 / 5.0 + 32.0

    # Initialize SPI and sensor
    spi = board.SPI()
    cs = DigitalInOut(board.D5)
    sensor = adafruit_max31855.MAX31855(spi, cs)
    
    # Initialize PID
    pid = PID.PID(conf.Pc, conf.Ic, conf.Dc)
    pid.SetPoint = state['settemp']
    pid.setSampleTime(conf.sample_time*5)

    # Initialize variables
    i = 0
    pidhist = [0.] * 10
    temphist = [0.] * 5
    lastsettemp = state['settemp']
    lasttime = time()

    try:
        while True:
            try:
                tempc = sensor.temperature
                
                # Update temperature history and average
                temphist[i % 5] = tempc
                avgtemp = sum(temphist)/len(temphist)
                
                # Update PID setpoint if changed
                if state['settemp'] != lastsettemp:
                    pid.SetPoint = state['settemp']
                    lastsettemp = state['settemp']
                
                # Update state counter to show thread is alive
                state['i'] = i
                
                # Update PID every 10 cycles
                if i % 10 == 0:
                    pid.update(avgtemp)
                    pidout = pid.output
                    if pidout > 100:
                        pidout = 100
                    elif pidout < 0:
                        pidout = 0
                    pidhist[i % 10] = pidout
                    state['avgpid'] = sum(pidhist)/len(pidhist)

                # Sleep and increment counter
                sleeptime = max(0, lasttime + conf.sample_time - time())
                sleep(sleeptime)
                i += 1
                lasttime = time()
                
            except Exception as e:
                print(f"PID loop error: {str(e)}")
                sleep(1)  # Add delay before retry
                continue
            
    except Exception as e:
        print(f"Fatal PID loop error: {str(e)}")
    finally:
        GPIO.cleanup()
        pid.clear()



if __name__ == '__main__':
    from multiprocessing import Process, Manager
    from time import sleep
    from urllib.request import urlopen
    import config as conf
    import timer
    from restServer import rest_server
    from datetime import datetime, timedelta
    import csv

    manager = Manager()
    pidstate = manager.dict()
    pidstate['snooze'] = conf.snooze
    pidstate['snoozeon'] = False
    pidstate['i'] = 0
    pidstate['settemp'] = conf.set_temp
    pidstate['avgpid'] = 0.

    timeState = manager.dict()    
    timeState['TimerOnMo'] = conf.TimerOnMo
    timeState['TimerOffMo'] = conf.TimerOffMo
    timeState['TimerOnTu'] = conf.TimerOnTu
    timeState['TimerOffTu'] = conf.TimerOffTu
    timeState['TimerOnWe'] = conf.TimerOnWe
    timeState['TimerOffWe'] = conf.TimerOffWe
    timeState['TimerOnTh'] = conf.TimerOnTh
    timeState['TimerOffTh'] = conf.TimerOffTh
    timeState['TimerOnFr'] = conf.TimerOnFr
    timeState['TimerOffFr'] = conf.TimerOffFr
    timeState['TimerOnSa'] = conf.TimerOnSa
    timeState['TimerOffSa'] = conf.TimerOffSa
    timeState['TimerOnSu'] = conf.TimerOnSu
    timeState['TimerOffSu'] = conf.TimerOffSu

    # timeState['overRide'] = conf.overRide

    pidstate['awake'] = timer.timer(timeState)

    p = Process(target=pid_loop, args=(1, pidstate))
    p.daemon = True
    p.start()

    h = Process(target=he_control_loop, args=(1, pidstate,timeState))
    h.daemon = True
    h.start()

    r = Process(target=rest_server, args=(1, pidstate,timeState))
    r.daemon = True
    r.start()

    # Start Watchdog loop
    piderr = 0
    weberr = 0
    weberrflag = 0
    urlhc = 'http://localhost:'+str(conf.port)+'/healthcheck'

    lasti = pidstate['i']
    sleep(1)

    while p.is_alive() and h.is_alive() and r.is_alive():
        curi = pidstate['i']
        if curi == lasti:
            piderr = piderr + 1
        else:
            piderr = 0

        lasti = curi

        if piderr > 9:
            print('ERROR IN PID THREAD, RESTARTING')
            with open("FailedPIDcsv.csv", "a+") as tempFile:
                fieldNames = ["time"]
                writer = csv.DictWriter(tempFile, fieldnames=fieldNames)
                writer.writerow({"time": datetime.now()})
            
            # Restart PID thread
            p.terminate()
            p.join()
            p = Process(target=pid_loop, args=(1, pidstate))
            p.daemon = True
            p.start()
            piderr = 0  # Reset error counter

        try:
            hc = urlopen(urlhc, timeout=10)
        except:
            weberrflag = 1
        else:
            if hc.getcode() != 200:
                weberrflag = 1

        if weberrflag != 0:
            weberr = weberr + 1

        if weberr > 9:
            print ('ERROR IN WEB SERVER THREAD, RESTARTING')
            with open("FailedWEBcsv.csv","a+") as tempFile:
                fieldNames = ["time"]
                writer = csv.DictWriter(tempFile,fieldnames=fieldNames)
                writer.writerow({"time": datetime.now()})
            # r.terminate()

        weberrflag = 0

        sleep(1)
