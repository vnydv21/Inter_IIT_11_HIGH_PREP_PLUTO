import logging

log_format = logging.Formatter('%(asctime)s | %(levelname)s \t %(message)s', datefmt='%H:%M:%S')

def GetLogger(name, log_file, level=logging.DEBUG):
    """Returns a logger object to set file"""

    handler = logging.FileHandler(log_file,'w')
    handler.setFormatter(log_format)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger
