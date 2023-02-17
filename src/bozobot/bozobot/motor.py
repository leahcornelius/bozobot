import serial
import time
import threading

# Result codes:
# -1: Tried to send command after kill command/ shutdown
# 0: Success
# 1: Invalid parameter count
# 2: Invalid parameter type/ could not parse
# 3: Invalid parameter value
# 4: Unknown command
# 5: Command not implemented
# 6: Command not allowed in current state
# 7: Internal error

class MotorDriverCommand():
    def __init__(self, command, *args_list):
        self.command = command
        self.args = args_list

    def __str__(self):
        return "MotorDriverCommand: {} {}".format(self.command, self.args)

class MotorDriver(threading.Thread):
    def __init__(self, port, baud, node, timeout=0.1):
        super().__init__()
        self.motor_driver = serial.Serial(port, baud, timeout=timeout)
        self.killed = threading.Event()
        self.connected = False

        # Used when clearing the command to prevent partial commands from being sent
        self.master_lock = threading.Lock()
        self.new_command = threading.Event()
        self.command = None
        self.command_time = None
        # self.after_expire = None # None: set motors to 0, 1: Brake for 1s, 2: Kill

        self.active_command = None
        self.active_timer = None
        self.active_command_expire = threading.Event()

        self.command_result_set = threading.Condition()
        self.command_result_code = None
        self.command_result_details = None
        self.parent_node = node

    @property
    def logger(self):
        return self.parent_node.get_logger()

    def run(self):
        if not self.wait_for_connection():
            self.logger.error("MotorDriver.WaitForConnectionFailed")
            self.kill(wait=False)
            return
        
        loop_var = True
        while loop_var:
            if (self.killed.is_set()):
                self.logger.warn("MotorDriver.KilledEventSet")
                loop_var = False
                break

            if (self.active_command is None):
                # Wait for a new command (BLOCK)
                if not self.new_command.wait(timeout=0.1):
                    continue

                self.logger.debug("MotorDriver.CommandEvent.WhileAtRest: {} for {} ms".format(
                    self.command, self.command_time))
                self._handle_new_command()
            else:
                # Check if we have received a new command
                if (self.new_command.is_set()):
                    # We have received a new command, check if it is the same as the active command
                    if (self.active_command == self.command):
                        # The new command is the same as the active command, so we should extend the timer
                        self.logger.debug("MotorDriver.CommandEvent.WhileActive.RepeatCurrentCommand: {} for {} ms".format(
                            self.command, self.command_time))
                        if (self.active_timer is not None):
                            self.active_timer.cancel()
                        if (self.command_time is not None):
                            self.active_timer = threading.Timer(
                                self.command_time, self._stop_active_command)
                            self.active_timer.start()
                        self.new_command.clear()

                    else:
                        self.logger.debug("MotorDriver.CommandEvent.WhileActive.OverwriteCurrentCommand: {} for {} ms (old: {}, {})".format(
                            self.command, self.command_time, self.active_command, self.active_timer))
                        self._handle_new_command()
                elif (self.active_command_expire.is_set()):
                    # The timer has expired, so we should stop the active command
                    self.logger.debug("MotorDriver.CommandFinishEvent.TimerExpireSet: {} for {} ms".format(
                        self.active_command, self.command_time))
                    self.active_command_expire.clear()
                    self._clear_command()
                    self._set_motors(0, 0)

    def _set_and_notify_result(self, code, details):
        with self.command_result_set:
            self.command_result_code = code
            self.command_result_details = details
            self.command_result_set.notify_all()

    def _handle_new_command(self):
        self.active_command = self.command
        if (self.active_command is None):
            self.new_command.clear()
            self.logger.error("MotorDriver.ActiveCommandNone")
            return

        if (self.active_command.command == "s"):
            # Stop
            self._set_motors(0, 0)
            self._clear_command()
            self._set_and_notify_result(0, "External stop")
            return

        elif (self.active_command.command == "m"):
            if (len(self.active_command.args) != 2):
                self._clear_command()
                self._set_and_notify_result(1, "Expected 2 parameters, got {}".format(
                    len(self.active_command.args) ))
                return

            try:
                left = int(self.active_command.args[0])
                right = int(self.active_command.args[1])
                if (left < -255 or left > 255):
                    self._clear_command()
                    self._set_and_notify_result(3, "Left motor value out of range: {}".format(
                        left))
                    return

                if (right < -255 or right > 255):
                    self._clear_command()
                    self._set_and_notify_result(3, "Right motor value out of range: {}".format(
                        right))
                    return

                self._set_motors(left, right)
            except ValueError as value_error:
                self._clear_command()
                self._set_and_notify_result(2, "Could not parse parameters (ValueError), error: {}".format(
                    value_error))
                return
        elif (self.active_command.command == "eb"):
            # Enable brake
            self._enable_brake()

        elif (self.active_command.command == "kill"):
            self._clear_command()
            self._set_and_notify_result(0, "Killed")
            self.kill(wait=False)
            return
        else:
            self._clear_command()
            self.logger.error("MotorDriver.UnknownCommand: {}".format(
                self.active_command.command))
            self._set_and_notify_result(4, "Unknown command: {}".format(
                self.active_command.command))
            return

        self.new_command.clear()
        if (self.command_time is not None):
            self.active_timer = threading.Timer(
                self.command_time, self._stop_active_command)
            self.active_timer.start()

    def _stop_active_command(self):
        self.active_command_expire.set()
        
    def _enable_brake(self):
        self.motor_driver.write(b"tb;\n")
        self.motor_driver.flush()

    def _clear_command(self):
        # This should only be called by the self.run() thread
        with self.master_lock:
            self.active_command = None
            self.active_timer = None
            self.command = None
            self.command_time = None
            self.new_command.clear()
            self.active_command_expire.clear()

    def wait_for_connection(self):
        if self.connected:
            return True
        self.logger.info("Waiting for motor board connection...")
        time.sleep(0.1)
        line = None
        while line is None or line == b"":
            if self.killed.is_set():
                break
            try:
                line = self.motor_driver.readline()
            except Exception as e:
                print(e)
                break
        else:
            self.logger.info("Motor board connected, ID line: {}".format(line.decode()))
            self.connected = True
            return True
        self.logger.info("MotorDriver: WaitForConnection failed or was aborted")
        return False

    def _set_motors(self, left, right):
        self.motor_driver.write(
            b"sm;" + str(left).encode() + b"," + str(right).encode() + b",\n")
        self.motor_driver.flush()

    def set_new_command(self, command, time):
        with self.master_lock:
            self.command = command
            self.command_time = time
            self.new_command.set()

    def kill(self, wait=True):
        self.killed.set()
        if wait:
            self.join()
        else:
            time.sleep(0.1)
        self.logger.info("MotorDriver.py: Kill() set kill flag")
        self.active_command = None
        if self.active_timer is not None:
            self.active_timer.cancel()
        self.active_timer = None
        self.command = None
        self.command_time = None
        self.new_command.clear()
        self.active_command_expire.clear()
        self.logger.debug("Terminating motor driver port")
        try:
            self.motor_driver.write(b"kill;\n")
            self.motor_driver.flush()
            self.motor_driver.close()
        except Exception as e:
            self.logger.error("Exception while closing motor driver port: {} ignoring".format(e))

        self.logger.info("Motor Driver killed")