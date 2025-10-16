import random
import math

#determines random wind conditions, also limits the wind to be +- 6 m/s
def next_wind(curr_w):
    change = random.uniform(-1.0, 1.0)
    new = curr_w + change
    if (new > 6):
        new = 6
    elif (new < -6):
        new = -6
    return new