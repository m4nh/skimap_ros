import numpy as np
import os


class Bench(object):
    NAME_SEPARATOR = '_'

    def __init__(self, filename, name_composition='0_str', columns=[], start_column=0):
        self.filename = filename
        self.raw_data = np.genfromtxt(filename, dtype=object, delimiter=' ')
        self.data = np.array(self.raw_data[:, start_column:], dtype=float)

        col_counter = 0
        self.columns = {}
        for c in columns.split(":"):
            self.columns[c] = col_counter
            col_counter += 1

        raw_data_row = self.raw_data[0, :]
        self.name = ""
        if name_composition == None or len(name_composition) == 0:
            self.name = os.path.basename(filename).split(".")[0]
        else:
            name_chunks = name_composition.split(":")
            for i in range(0, len(name_chunks)):
                chunk = name_chunks[i]
                index = int(chunk.split("_")[0])
                type = str(chunk.split("_")[1])
                self.name += self._buildNameChunk(raw_data_row, index, type)
                if i != len(name_chunks) - 1:
                    self.name += Bench.NAME_SEPARATOR

    def getDataByName(self, column_name):
        index = self.columns[column_name]
        return self.data[:, index]

    def _buildNameChunk(self, raw_data, index, type):
        print "BUILD CHUNK", index, type, raw_data[index]
        if type == 'float':
            return "{:.2f}".format(float(raw_data[index]))
        if type == 'int':
            return "{:d}".format(int(float(raw_data[index])))
        if type == 'str':
            return "{}".format(str(raw_data[index]))
