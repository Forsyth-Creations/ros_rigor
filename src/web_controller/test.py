import os
from glob import glob

# os.path.join("lib", package_name, "backend", "app")

base = "lib"

path = os.path.join("web_controller", "frontend", ".next")

# Recursively list all directories

directories = [x[0] for x in os.walk(path)] 

directories_and_files = []

for directory in directories:
    files = glob(os.path.join(directory, "*"))
    
    # Adjust the directory path to be relative to the lib
    directory = os.path.join(base, directory)
    directories_and_files.append((directory, files))
    
print(directories_and_files)

