# Misc. helper funcs
import os

## Makes a new folder and adds a number if the folder already exists in the dir
def make_folder(file_path):
    if not os.path.exists(file_path):
        os.makedirs(file_path)
        
def write_to_file(data, file):
    with open(file, 'a') as fd:
        fd.write(data)
