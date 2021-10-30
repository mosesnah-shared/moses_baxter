import time
import sys

from my_constants import Constants as C

class Logger(object):
    def __init__(self, record_data = False):
        self.record_data = record_data
        self.terminal    = sys.stdout

        if record_data == True:  
            time_now = time.strftime( '%Y_%m_%d-%H_%M' )                 # Redirecting the stdout to file
            self.log = open( C.SAVE_DIR + 'baxter_'  + time_now + '.txt' , 'w')
        else:
            self.log = None
   
    def write(self, message ):
        self.terminal.write( message )

        if self.log is not None:
            self.log.write( message )  

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass    