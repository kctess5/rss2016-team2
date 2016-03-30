import rospy
import threading
import tf

class TFPub(object):
    """Publishes a transform at a fixed rate.
    The default rate is 15 Hz.
    The same transform is continuously published even if set is not called.
    This is to avoid a tf expiring because the producer is slow.
    """
    def __init__(self, rate=15.0):
        self._lock = threading.RLock()
        self._pub_tf = tf.TransformBroadcaster()
        self._latest = None
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._on_timer)

    def Set(self, translation, rotation, child, parent):
        """Set the current transform.
        Publishes immediately and sets the value for future publishes.
        Takes args similar to sendTransform but not time.
        Args:
            translation: Translation in XYZ (3-array).
            rotation: Rotation as a quaternion (4-array).
            child: Child tf frame (string).
            parent: Parent tf frame (string).
        Note that time is filled by this class.
        """
        with self._lock:
            self._latest = (translation, rotation, child, parent)
        self._send()

    def Stop(self):
        """Stop publishing forever.
        Some more messages might be published after Stop returns.
        Re-enabling is not supported."""
        with self._lock:
            self._timer.shutdown()

    def _on_timer(self, event):
        self._send()

    def _send(self):
        with self._lock:
            if self._latest == None:
                # Don't bother until something has been set.
                return

            # Read the data safely, then release the lock.
            (translation, rotation, child, parent) = self._latest 

        self._pub_tf.sendTransform(
            translation=translation,
            rotation=rotation,
            time=rospy.Time.now(),
            child=child,
            parent=parent)
