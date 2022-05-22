import logging

"""
Logging levels
# DEBUG: Detailed information, typically of interest only when diagnosing problems.

# INFO: Confirmation that things are working as expected.

# WARNING: An indication that something unexpected happened, or indicative of some
    problem in the near future. The software is still working as expected

# ERROR: Due to a more serious problem, the software has not been able to perform such function

# CRITICAL: A serious error, indicating that the program itself may be unable to continue running.
"""

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s:%(levelname)s:%(message)s')

file_handler = logging.FileHandler('logging_example.log')
file_handler.setFormatter(formatter)

log.addHandler(file_handler)


def test():
    log.debug("According to all laws of aviation, bees should not fly")
    log.debug("Set your fleeb to level plumbus")
    
    log.warning("Prepare to die")
    log.error("No more pumpernickel for you")
    log.critical("I've lost all my toesie wosies!")


test()