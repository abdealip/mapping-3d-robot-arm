import os

def rmdir_recursive(dir):
    for name in os.listdir(dir):
        path = os.path.join(dir, name)
        if os.path.isfile(path):
            os.remove(path)
        else:
            rmdir_recursive(path)
    os.rmdir(dir)

def clear_and_mkdir_with_confirmation(dir):
    if os.path.isdir(dir):
        res = input(f"Warning: Directory {dir} exists. Overwrite? [y/n] ")
        while res not in ["y", "n"]:
            res = input("Please type y to overwrite, n to cancel: ")
        if res == "y":
            rmdir_recursive(dir)
        else:
            exit(0)
    os.mkdir(dir)

def make_filename(fname: str, extension: str) -> str:
    fname = fname.replace("/", "_")
    fname = fname.replace(" ", "_")
    fname = fname.replace(".", "_")
    fname += "." + extension
    return fname
