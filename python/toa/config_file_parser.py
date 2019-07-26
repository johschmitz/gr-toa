import configparser
import sys

def parse_config_file(file):
    config = configparser.ConfigParser()
    if 1 != len(config.read(file)):
        sys.exit("Error: Couldn't read config file.")
    cfg_evaled = dict()
    for section in config:
        cfg_evaled[section] = dict()
        for key in config[section]:
            cfg_evaled[section][key] = eval(config[section][key])
    return cfg_evaled