'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Trautmann Jeremias, Wittenbeck Valerij

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from functools import wraps
from time import time
import csv
import __builtin__
import rospy

def timed(f):
    """
    measure execution time
    """
    @wraps(f)
    def wrapper(*args, **kwds):
        start = time()
        result = f(*args, **kwds)
        elapsed = time() - start
        print "%s took %.3f secs to finish" % (args[0].__class__.__name__, elapsed)
        return result
    return wrapper


def key_press(f):
    """
    Wait for any key press
    """
    def wrapper(*args, **kwds):
        result = f(*args, **kwds)
        if hasattr(__builtin__, "evaluation"):
            if __builtin__.evaluation:
                raw_input("Press key to continue")
        else: 
            raw_input("Press key to continue")
        return result
    return wrapper


def log(f):
    """
    Write State, Time and Outcome into csv log
    """
    def wrapper(*args, **kwds):
        result = f(*args, **kwds)

        if hasattr(__builtin__, 'log_dir'):
            # cheating hard for getting the log_dir
            with open(__builtin__.log_dir +  '/log.csv', 'a') as log:
                logwriter = csv.writer(log, delimiter=' ', 
                                       quotechar='|', quoting=csv.QUOTE_MINIMAL)

                logwriter.writerow([time(), args[0].__class__.__name__, result])
        return result
    return wrapper
