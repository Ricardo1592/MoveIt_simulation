import configparser
import os

class MyConfigParser:
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.config.read(os.path.join(os.path.dirname(__file__), '..\\..\\..', 'config.ini'))

    def get(self, section, key):
        return self.config.get(section, key)

    def get_int(self, section, key):
        return self.config.getint(section, key)

    def get_boolean(self, section, key):
        return self.config.getboolean(section, key)
