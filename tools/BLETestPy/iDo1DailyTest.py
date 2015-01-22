import DongleFactory
import DongleMgr
import sys
import threading
import time
import Util
import Log

dm = None
task_done = 0
iDo_name = None

def read_callback():
    print "read_callback"

def notification_callback():
    print "notification_callback"

def work1():
    global dm
    global task_done
    global iDo_name
    log_f_name = iDo_name + ".log"

    Log.screen("Try to connect with " + iDo_name)
    ret, handle = dm.connect(DongleMgr.TARGET_NAME, iDo_name)
    if ret != DongleMgr.SUCCESS:
        Log.screen("Failed to connect to iDo")
        task_done = 1
        return


    #########################################################################
    # Check temperature API FF query
    #########################################################################
    Log.screen("Connected to " + iDo_name)
    Log.task_log(log_f_name, time.strftime('%Y/%m/%d %H:%M:%S',time.localtime(time.time())))

    time.sleep(1)
    ret, msg = dm.read_char(handle, "2a08", read_callback)
    if ret != DongleMgr.SUCCESS:
        Log.screen("Read 2a08 failed")
        task_done = 1
        return

    Log.task_log(log_f_name, "Board time now: " + msg)

    time.sleep(1)
    ret, l = dm.write_char(handle, "2a1e", "FF0000000000000000000000")
    if ret != DongleMgr.SUCCESS:
        Log.screen("Write 2a1e failed")
        task_done = 1
        return

    while 1:
        time.sleep(1)
        ret, msg = dm.read_char(handle, "2a1e", read_callback)
        if ret != DongleMgr.SUCCESS:
            Log.screen("Read 2a1e failed")
            task_done = 1
            return

        if Util.get_byte_from_ascii_str(msg, 1) != 255:
            break

    # Retrieve Total Count from Result
    total_fragments = Util.get_byte_from_ascii_str(msg, 1)
    Log.task_log(log_f_name, "Total fragments: " + str(total_fragments))
    ret, l = dm.write_char(handle, "2a1e", "FF0000000000000000000000")
    if ret != DongleMgr.SUCCESS:
        Log.screen("Write 2a1e failed")
        task_done = 1
        return

    Log.task_log(log_f_name, "FF query results:")
    for i in range(0, total_fragments * 2):
        time.sleep(1)
        ret, msg = dm.read_char(handle, "2a1e", read_callback)
        if ret != DongleMgr.SUCCESS:
            Log.screen("Read 2a1e failed")
            task_done = 1
            return
        Log.task_log(log_f_name, msg)
        if msg == "000000000000000000000000":
            Log.screen("Invalid FF query result!")
            task_done = 1
            return
        begin_end_flag = Util.get_byte_from_ascii_str(msg, 0)
        total = Util.get_byte_from_ascii_str(msg, 1)
        seq = Util.get_byte_from_ascii_str(msg, 2)
        if (i % 2 + 1) != begin_end_flag or total != total_fragments or seq != i / 2:
            Log.screen("Invalid FF query result!")
            task_done = 1
            return

    last_msg = msg
    time.sleep(1)
    ret, msg = dm.read_char(handle, "2a1e", read_callback)
    if ret != DongleMgr.SUCCESS:
        Log.screen("Read 2a1e failed")
        task_done = 1
        return
    if msg != last_msg:
        Log.task_log(log_f_name, "FF query result does not match!")
        Log.task_log(log_f_name, "msg: " + msg)
        task_done = 1
        return
   
    ##############################################################################################
    # Check power management log
    ##############################################################################################
    ret, l = dm.write_char(handle, "2a1e", "5f516f446f4d655f32303134")
    if ret != DongleMgr.SUCCESS:
        Log.screen("Write 2a1e failed")
        task_done = 1
        return

    for i in range(0, 100):
        time.sleep(1)
        ret, l = dm.write_char(handle, "6666", Util.int_to_ascii((3221225472 + 123 * 2048 + 36 + i * 20), 4))
        if ret != DongleMgr.SUCCESS:
            Log.screen("Write 2a1e failed")
            task_done = 1
            return

        time.sleep(1)
        ret, msg1 = dm.read_char(handle, "6666", read_callback)
        if ret != DongleMgr.SUCCESS:
            Log.screen("Read 6666 failed")
            task_done = 1
            return

        if Util.get_short_from_ascii_str(msg1, 4) == 65535:
            Log.screen("End of pwrmgmt records")
            break

        time.sleep(1)
        ret, msg2 = dm.read_char(handle, "6666", read_callback)
        if ret != DongleMgr.SUCCESS:
            Log.screen("Read 6666 failed")
            task_done = 1
            return

        Log.task_log(log_f_name, str(Util.get_short_from_ascii_str(msg1, 4)) + " " + str(Util.get_short_from_ascii_str(msg1, 6)) + " " + str(Util.get_int_from_ascii_str(msg1, 8)) + " " + str(Util.get_int_from_ascii_str(msg1, 12)) + " " + str(Util.get_int_from_ascii_str(msg1, 16)) + " " + str(Util.get_int_from_ascii_str(msg2, 4)))

    Log.task_log(log_f_name, "\n\n")

    ret, msg = dm.disconnect(handle)
    time.sleep(10)
    task_done = 1
    return

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "Usage: python " + sys.argv[0] + " \"<iDo's name>\""
        sys.exit(1)

    iDo_name = sys.argv[1]

    f = DongleFactory.DongleFactory()
    f.register("createDongleMgr", DongleMgr.DongleMgr)
    dm = f.createDongleMgr(1)
    if dm == None:
        print "Not able to get USB dongle"
        sys.exit(1)

    t1 = threading.Thread(target = work1)
    t1.daemon = True
    t1.start()

    while True:
        if task_done == 0:
            time.sleep(1)
        else:
            break

    dm.stop()
    sys.exit(0)

