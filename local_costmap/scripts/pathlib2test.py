from profilehooks import profile
# from pathlib import arc_step as arc_step_fast
from pathlib2 import arc_step_fast

@profile
def testfunc():
    for _ in xrange(100000):
        arc_step_fast(.05, 3., 1., 1., .1)

if __name__ == "__main__":
    testfunc()
