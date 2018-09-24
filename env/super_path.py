from __future__ import print_function

from env.path import Path

import sys
import os

sys.path.append(os.path.abspath('..//'))


class SuperPath(object):

    DELIMITER = "%"

    def __init__(self):
        self.paths = []

    def to_string(self):
        super_string = ""
        for path in self.paths:
            super_string += path.to_string() + self.DELIMITER
        return super_string

    def decode(self, super_string):
        path_strings = super_string.split(self.DELIMITER)[:-1]
        for path_string in path_strings:
            path = Path()
            path.decode(path_string)
            self.paths.append(path)


if __name__ == "__main__":
    a = "-270,81.456179797,84.4023108412,92.9002253912,108.515229898," \
        "%-270,2.21925134227,98.9097083301,-4.45338860541,117.379590018,%"
    sp = SuperPath()
    sp.decode(a)
    print(sp.paths)
