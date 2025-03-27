#!/usr/bin/python

def scheduler(dummy,state):
    import time
    import sys
    import schedule
    from datetime import datetime

    # Change buffering from 0 to 1 for line buffering
    sys.stdout = open("scheduler.log", "a", buffering=1)
    sys.stderr = open("scheduler.err.log", "a", buffering=1)

    print ("Starting scheduler thread ...")

    last_wake = 0
    last_sleep = 0
    last_sched_switch = 0

    while True:

        if last_wake != state['wake_time'] or last_sleep != state['sleep_time'] or last_sched_switch != state['sched_enabled']:
            schedule.clear()

            if state['sched_enabled'] == True:
                schedule.every().day.at(state['sleep_time']).do(gotosleep,1,state)
                schedule.every().day.at(state['wake_time']).do(wakeup,1,state)

                nowtm = float(datetime.now().hour) + float(datetime.now().minute)/60.
                sleeptm = state['sleep_time'].split(":")
                sleeptm = float(sleeptm[0]) + float(sleeptm[1])/60.
                waketm = state['wake_time'].split(":")
                waketm = float(waketm[0]) + float(waketm[1])/60.

                if waketm < sleeptm:
                    if nowtm >= waketm and nowtm < sleeptm:
                        wakeup(1,state)
                    else:
                        gotosleep(1,state)
                elif waketm > sleeptm:
                    if nowtm < waketm and nowtm >= sleeptm:
                        gotosleep(1,state)
                    else:
                        wakeup(1,state)

            else:
                wakeup(1,state)

        last_wake = state['wake_time']
        last_sleep = state['sleep_time']
        last_sched_switch = state['sched_enabled']

        schedule.run_pending()

        time.sleep(1)

def wakeup(dummy,state):
    state['is_awake'] = True

def gotosleep(dummy,state):
    state['is_awake'] = False

def he_control_loop(dummy,state):
    from time import sleep
    import RPi.GPIO as GPIO
    import config as conf
    import logging

    logger = logging.getLogger('silvia.gpio')
    
    try:
        logger.info("Initializing GPIO...")
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(conf.he_pin, GPIO.OUT)
        GPIO.output(conf.he_pin,0)
        logger.info(f"GPIO pin {conf.he_pin} initialized successfully")
        
        heating = False

        try:
            while True:
                avgpid = state['avgpid']

                if state['is_awake'] == False :
                    state['heating'] = False
                    GPIO.output(conf.he_pin,0)
                    sleep(1)
                else:
                    if avgpid >= 100 :
                        state['heating'] = True
                        GPIO.output(conf.he_pin,1)
                        sleep(1)
                    elif avgpid > 0 and avgpid < 100:
                        state['heating'] = True
                        GPIO.output(conf.he_pin,1)
                        sleep(avgpid/100.)
                        GPIO.output(conf.he_pin,0)
                        sleep(1-(avgpid/100.))
                        state['heating'] = False
                    else:
                        GPIO.output(conf.he_pin,0)
                        state['heating'] = False
                        sleep(1)

        finally:
            GPIO.output(conf.he_pin,0)
            GPIO.cleanup()
        
    except Exception as e:
        logger.error(f"GPIO Error: {str(e)}")
        raise
    finally:
        GPIO.output(conf.he_pin,0)
        GPIO.cleanup()

def pid_loop(dummy,state):
    import sys
    import logging
    from time import sleep, time
    from math import isnan
    
    logger = logging.getLogger('silvia.pid')
    
    try:
        logger.info("Initializing PID loop...")
        
        logger.info("Setting up logging files...")
        sys.stdout = open("pid.log", "a", buffering=1)
        sys.stderr = open("pid.err.log", "a", buffering=1)

        logger.info("Importing required modules...")
        try:
            import board
            import digitalio
            import adafruit_max31855
            import PID as PID
            import config as conf
        except Exception as e:
            logger.error(f"Failed to import required modules: {str(e)}")
            raise

        logger.info("Initializing SPI and sensor...")
        try:
            logger.info("Setting up SPI bus...")
            spi = board.SPI()
            
            logger.info("Setting up chip select on GPIO5...")
            cs = digitalio.DigitalInOut(board.D5)
            cs.direction = digitalio.Direction.OUTPUT
            
            logger.info("Initializing MAX31855...")
            sensor = adafruit_max31855.MAX31855(spi, cs)
            
            # Test sensor reading
            tempc = sensor.temperature
            logger.info(f"Initial temperature reading: {tempc}°C")
            if tempc is None or isnan(tempc):
                raise Exception("Could not read temperature - check wiring")
                
        except Exception as e:
            logger.error(f"Failed to initialize MAX31855 sensor: {str(e)}")
            raise

        logger.info("Setting up PID controller...")
        pid = PID.PID(conf.Pc, conf.Ic, conf.Dc)
        pid.SetPoint = state['settemp']
        pid.setSampleTime(conf.sample_time*5)

        nanct=0
        i=0
        j=0
        pidhist = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        avgpid = 0.
        temphist = [0.,0.,0.,0.,0.]
        avgtemp = 0.
        lastsettemp = state['settemp']
        lasttime = time()
        sleeptime = 0
        iscold = True
        iswarm = False
        lastcold = 0
        lastwarm = 0

        try:
            while True : # Loops 10x/second
                tempc = sensor.temperature  # Changed from readTempC() to temperature
                if isnan(tempc) :
                    nanct += 1
                    if nanct > 100000 :
                        sys.exit
                    continue
                else:
                    nanct = 0

                temphist[i%5] = tempc
                avgtemp = sum(temphist)/len(temphist)

                if avgtemp < 100 :
                    lastcold = i

                if avgtemp > 200 :
                    lastwarm = i

                if iscold and (i-lastcold)*conf.sample_time > 60*15 :
                    pid = PID.PID(conf.Pw,conf.Iw,conf.Dw)
                    pid.SetPoint = state['settemp']
                    pid.setSampleTime(conf.sample_time*5)
                    iscold = False

                if iswarm and (i-lastwarm)*conf.sample_time > 60*15 : 
                    pid = PID.PID(conf.Pc,conf.Ic,conf.Dc)
                    pid.SetPoint = state['settemp']
                    pid.setSampleTime(conf.sample_time*5)
                    iscold = True

                if state['settemp'] != lastsettemp :
                    pid.SetPoint = state['settemp']
                    lastsettemp = state['settemp']

                if i%10 == 0 :
                    pid.update(avgtemp)
                    pidout = pid.output
                    pidhist[int(i/10) % 10] = pidout  # Convert float division to integer
                    avgpid = sum(pidhist)/len(pidhist)

                state['i'] = i
                state['tempc'] = round(tempc,2)
                state['avgtemp'] = round(avgtemp,2)
                state['pidval'] = round(pidout,2)
                state['avgpid'] = round(avgpid,2)
                state['pterm'] = round(pid.PTerm,2)
                if iscold :
                    state['iterm'] = round(pid.ITerm * conf.Ic,2)
                    state['dterm'] = round(pid.DTerm * conf.Dc,2)
                else :
                    state['iterm'] = round(pid.ITerm * conf.Iw,2)
                    state['dterm'] = round(pid.DTerm * conf.Dw,2)
                state['iscold'] = iscold

                print (time(), state)

                sleeptime = lasttime+conf.sample_time-time()
                if sleeptime < 0 :
                    sleeptime = 0
                sleep(sleeptime)
                i += 1
                lasttime = time()

        finally:
            pid.clear

    except Exception as e:
        logger.error(f"Fatal error in PID loop: {str(e)}")
        raise
    finally:
        if 'pid' in locals():
            pid.clear()

def rest_server(dummy,state):
    from bottle import route, run, get, post, request, static_file, abort
    from subprocess import call
    from datetime import datetime
    import config as conf
    import os

    logger = logging.getLogger('silvia.server')

    basedir = os.path.dirname(os.path.abspath(__file__))
    wwwdir = os.path.join(basedir, 'www')
    
    logger.info(f"Serving files from: {wwwdir}")

    @route('/')
    def docroot():
        logger.info(f"Serving index.html from {wwwdir}")
        if not os.path.exists(os.path.join(wwwdir, 'index.html')):
            logger.error("index.html not found")
            abort(404, "index.html not found")
        return static_file('index.html', wwwdir)

    @route('/<filepath:path>')
    def servfile(filepath):
        return static_file(filepath,wwwdir)

    @route('/curtemp')
    def curtemp():
        return str(state['avgtemp'])

    @get('/settemp')
    def settemp():
        return str(state['settemp'])

    @post('/settemp')
    def post_settemp():
        try:
            settemp = float(request.forms.get('settemp'))
            if settemp >= 80 and settemp <= 110 :
                state['settemp'] = settemp
                return str(settemp)
            else:
                abort(400,'Set temp out of range 200-260.')
        except:
            abort(400,'Invalid number for set temp.')

    @get('/is_awake')
    def get_is_awake():
        return str(state['is_awake'])

    @post('/scheduler')
    def set_sched():
        sched = request.forms.get('scheduler')
        if sched == "True":
            state['sched_enabled'] = True
        elif sched == "False":
            state['sched_enabled'] = False
            state['is_awake'] = True
        else:
            abort(400,'Invalid scheduler setting. Expecting True or False')

    @post('/setwake')
    def set_wake():
        wake = request.forms.get('wake')
        try:
            datetime.strptime(wake,'%H:%M')
        except:
            abort(400,'Invalid time format.')
        state['wake_time'] = wake
        return str(wake)

    @post('/setsleep')
    def set_sleep():
        sleep = request.forms.get('sleep')
        try:
            datetime.strptime(sleep,'%H:%M')
        except:
            abort(400,'Invalid time format.')
        state['sleep_time'] = sleep
        return str(sleep)

    @get('/allstats')
    def allstats():
        return dict(state)

    @route('/restart')
    def restart():
        call(["reboot"])
        return '';

    @route('/shutdown')
    def shutdown():
        call(["shutdown","-h","now"])
        return '';

    @get('/healthcheck')
    def healthcheck():
        return 'OK'

    run(host='0.0.0.0', 
        port=conf.port,
        server='cheroot')  # Change server to 'cherrypy' instead of 'cheroot'

if __name__ == '__main__':
    from multiprocessing import Process, Manager
    from time import sleep
    from urllib.request import urlopen
    import config as conf
    import sys
    import logging

    # Setup logging
    logging.basicConfig(
        filename='silvia.log',
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger('silvia')

    # Initialize processes dictionary
    processes = {}

    try:
        # Initialize manager and state
        manager = Manager()
        pidstate = manager.dict()
        pidstate['is_awake'] = True
        pidstate['sched_enabled'] = conf.sched_enabled
        pidstate['sleep_time'] = conf.sleep_time
        pidstate['wake_time'] = conf.wake_time
        pidstate['i'] = 0
        pidstate['settemp'] = conf.set_temp
        pidstate['avgpid'] = 0.
        pidstate['heating'] = False
        pidstate['tempf'] = 0.0
        pidstate['avgtemp'] = 0.0

        # Start each process with error checking
        for name, target_func in [
            ('scheduler', scheduler),
            ('pid', pid_loop),
            ('heater', he_control_loop),
            ('server', rest_server)
        ]:
            try:
                logger.info(f"Starting {name} process...")
                proc = Process(target=target_func, args=(1, pidstate))
                proc.daemon = True
                proc.start()
                sleep(2)  # Give more time to initialize
                
                if not proc.is_alive():
                    logger.error(f"{name} process failed to start")
                    raise Exception(f"{name} process failed to start")
                
                processes[name] = proc
                logger.info(f"{name} process started successfully")
            
            except Exception as e:
                logger.error(f"Error starting {name} process: {str(e)}")
                raise

        # Store process references
        s = processes['scheduler']
        p = processes['pid']
        h = processes['heater']
        r = processes['server']

        # Watchdog loop
        print("Starting Watchdog...")
        piderr = 0
        weberr = 0
        weberrflag = 0
        urlhc = 'http://localhost:'+str(conf.port)+'/healthcheck'

        lasti = pidstate['i']
        sleep(2)  # Give processes time to start

        while True:
            if not all([p.is_alive(), h.is_alive(), r.is_alive(), s.is_alive()]):
                print("One or more processes died unexpectedly")
                raise Exception("Process died")

            curi = pidstate['i']
            if curi == lasti:
                piderr += 1
            else:
                piderr = 0

            lasti = curi

            # Handle PID errors
            if piderr > 9:
                print('ERROR IN PID THREAD, RESTARTING')
                p.terminate()
                p.join(timeout=1)
                p = Process(target=pid_loop, args=(1, pidstate))
                p.daemon = True
                p.start()
                piderr = 0

            # Check web server
            try:
                hc = urlopen(urlhc, timeout=2)
                if hc.getcode() != 200:
                    weberrflag = 1
            except:
                weberrflag = 1

            if weberrflag:
                weberr += 1
            else:
                weberr = 0

            # Handle web server errors
            if weberr > 9:
                print('ERROR IN WEB SERVER THREAD, RESTARTING')
                r.terminate()
                r.join(timeout=1)
                r = Process(target=rest_server, args=(1, pidstate))
                r.daemon = True
                r.start()
                weberr = 0

            weberrflag = 0
            sleep(1)

    except KeyboardInterrupt:
        logger.info("Shutting down gracefully...")
    except Exception as e:
        logger.error(f"Fatal error: {str(e)}")
    finally:
        # Cleanup all processes
        for name, process in processes.items():
            if process and process.is_alive():
                logger.info(f"Terminating {name} process...")
                process.terminate()
                process.join(timeout=1)
        sys.exit(1)