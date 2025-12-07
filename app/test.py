from subprocess import run
from multiprocessing import Process
def start_flight():
    return run(["python3", r"../flight.py"])


process_running_flight = Process(target=start_flight)
process_running_flight.start()
