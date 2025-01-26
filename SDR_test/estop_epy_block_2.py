import numpy as np
import time
import multiprocessing
from gnuradio import gr

# Shared memory variable (boolean type)
shared_variable = multiprocessing.Value('b', False)

class blk(gr.sync_block):
    """Toggle Variable Block with Shared Memory"""

    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='Toggle Variable Block with Shared Memory',
            in_sig=[np.float32],
            out_sig=None
        )
        self.last_toggle_time = 0
        self.cooldown = 2

    def work(self, input_items, output_items):
        current_time = time.time()
        for value in input_items[0]:
            if value == 1.0 and (current_time - self.last_toggle_time) >= self.cooldown:
                with shared_variable.get_lock():
                    shared_variable.value = not shared_variable.value
                    print(f"Toggled Variable: {shared_variable.value}")
                self.last_toggle_time = current_time

        return len(input_items[0])

