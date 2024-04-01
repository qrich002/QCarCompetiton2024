# main.py
import time
from threading import Thread, Event

from pal.utilities.scope import MultiScope

from Helpers import *
from parameters_config import Config
from ControlLoop import ControlLoop
from initialize_multi_scopes import initialize_multi_scopes

# Prepare the configuration object
config = Config()

# Initialize the kill event for thread management
KILL_THREAD = Event()

def main():
    # Start the control loop in a separate thread
    controlThread = Thread(target=lambda: ControlLoop(config, KILL_THREAD))
    controlThread.start()

    try:
        while controlThread.is_alive():
            MultiScope.refreshAll()
            time.sleep(0.01)    
    except KeyboardInterrupt:
        KILL_THREAD.set()  # Signal to stop the control loop
    finally:
        KILL_THREAD.set()  # Ensure the flag is set if any exception occurs

    # Clean up and ensure thread is joined
    controlThread.join()
    input('Experiment complete. Press Enter to exit...')

if __name__ == '__main__':
    main()