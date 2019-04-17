import configparser

def parse_config_file(file):
    config = configparser.ConfigParser()
    config.read(file)
    cfg_evaled = dict()
    for section in config:
        cfg_evaled[section] = dict()
        for key in config[section]:
            cfg_evaled[section][key] = eval(config[section][key])
    return cfg_evaled