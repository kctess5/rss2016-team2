"""
Are we race car.....


Or are we dancer?
"""

import platform
import subprocess

def is_racecar():
    """Whether this is running on the racecar."""
    hostname = platform.node()
    return ("racecar-" in hostname) and (hostname != "racecar-vm")

def is_vm():
    """Whether this is running on a 141 vm.
    Please note this is not equivalent to (not is_racecar()).
    If you want (not is_racecar()), then use that.
    """
    hostname = platform.node()
    return (hostname == "racecar-vm")

def is_corys_vm():
    """Whether this is running on cory's VM."""
    i3 = _run_quietly(["pgrep", "i3bar"]) == 0
    milesfile = _run_quietly(["stat", "/home/racecar/.ismiles"]) == 0
    return bool(i3 and not milesfile)

def is_miless_vm():
    """Whether this is running on miles's VM."""
    i3 = _run_quietly(["pgrep", "i3bar"]) == 0
    milesfile = _run_quietly(["stat", "/home/racecar/.ismiles"]) == 0
    return bool(i3 and milesfile)

def _run_quietly(shell_args):
    """Run a shell command silently and get its exit code."""
    p = subprocess.Popen(shell_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()
    return p.returncode

if __name__ == "__main__":
    print "is_racecar", is_racecar()
    print "is_vm", is_vm()
    print "is_corys_vm", is_corys_vm()
    print "is_miless_vm", is_miless_vm()
