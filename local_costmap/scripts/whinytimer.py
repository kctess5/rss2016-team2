import rospy
import time
import threading

class WhinyTimer(object):
    """A timer with a maximum call rate.
    
    The timer prints a warning when the callback is too slow.
    
    Callback is called in a separate thread.
    So care must be taken so that callback is thread safe.
    Callback is called at a rate at most 1 / duration.
    This timer does not provide an event.

    Args:
        duration: Period of the timer. A ROS duration.
        callback: The function to call.
    """
    def __init__(self, duration, callback):
        # Store duration in seconds as a float.
        # A ROS duration is taken to be more similar to rospy.Timer.
        # Python time is used internally because it is monotonically increasing.
        self.duration = duration.to_sec()
        self.callback = callback
        self._spawn_thread()

    def _spawn_thread(self):
        """Start a thread which loops."""
        thread = threading.Thread(target=self._thread_main)
        thread.daemon = True
        self.thread = thread
        self.thread.start()

    def _thread_main(self):
        """The main loop of the thread."""
        # Time of the last issued overrun warning.
        last_warned_time = 0.
        # Seconds between each overrun warning.
        warning_suppress_duration = 2.

        while True:
            # Time of last execution start.
            last_exec_time = time.time()
            self.callback()
            last_complete_time = time.time()

            # Time to sleep until next call.
            sleep_time = self.duration - (time.time() - last_exec_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Sleep time is negative.
                # We are running behind schedule.
                # Warn, but not too often.
                if time.time() > last_warned_time + warning_suppress_duration:
                    print (("Warning: Timer overflow by {} seconds.\n"
                            "         Callback took {} seconds.\n"
                            "         Warning suppressed for {} seconds.")
                            .format(-sleep_time, last_complete_time - last_exec_time,
                                    warning_suppress_duration))
                    last_warned_time = time.time()


class EveryN(object):
    """Counter to facilitate doing something every n rounds.

    Args:
        n: step returns true after n calls.
    """
    def __init__(self, n):
        self._start = n
        self._counter = self._start
        self.reset()

    def reset(self, n=None):
        """Reset the counter"""
        if n != None:
            self._start = n
        self._counter = self._start

    def step(self, auto_reset=False):
        """Step the timer.
        Args:
            auto_reset: Whether to reset after the timer hits.
        Returns: Whether the timer has hit. (bool)
        """
        self._counter -= 1
        if auto_reset and self.peak():
            self.reset()
            return True
        else:
            return self.peak()

    def peak(self):
        return self._counter < 0
