

import sys, select, termios, tty
import time, datetime

# Arrow-UP (A) -> FORWARD
# Arrow-DOWN (B)  -> BACKWARD 
# Arrow-RIGHT (C) -> ROTAION CLOCK_WISE
# Arrow-LEFT (D) -> ROTAION ANTI CLOCK_WISE
# s -> STOP 

# coupled buttons A,B for forward/backward movement, C,D for clockwise/anticlockwise rotation, s for stop

key_binding_dict = {'A':False, 'B':False, 'C':False, 'D':False, 's':False}
key_counter_dict = {'A':[False, 0], 'B':[False, 0], 'C':[False, 0], 'D':[False, 0], 's':False}
keypress_counters = {'linear':0, 'angular':0}

class readKeyBoard() :
    def __init__(self) -> None:
        # Settings to read keys
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
   
    def getKeyboardActivities(self):

        while True:

            key = self.getKey()
        
            if(key == 'x'): 
                print("Exiting!")
                break
            
            if(key == 'A'):
                keypress_counters['linear'] += 1
            if(key == 'B'):
                 keypress_counters['linear'] -= 1

            if(key == 'C'):
                 keypress_counters['angular'] -= 1
            if(key == 'D'):
                  keypress_counters['angular'] += 1

            if(key == 's'):
                print("STOPING the bot!")
                keypress_counters['linear'] = 0
                keypress_counters['angular'] = 0   


              