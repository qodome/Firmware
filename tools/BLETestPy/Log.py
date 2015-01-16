import time
from threading import Lock

screen_lock = Lock()

def log_file(fn, msg, timestamp = 0):
    output_f = open(fn, "a")
    if output_f == None:
        print "Failed to open " + fn + " for write!"
    else:
        if timestamp == 1:
            output_f.write(time.strftime('%Y/%m/%d %H:%M:%S',time.localtime(time.time())) + "\n")
        output_f.write(str(msg) + "\n")
        if timestamp == 1:
            output_f.write("\n\n")
        output_f.close()

def log(msg):
    log_file("ble.log", msg, 1)

def screen(msg):
    screen_lock.acquire()
    print msg
    screen_lock.release()

def task_log(fn, msg):
    log_file(fn, msg)
    screen(msg)

