import os
import argparse
import subprocess
import sys
import pathlib

def lsp(path_list):
    """
    lsp: List full path
    """
    
    cwd = os.getcwd()
    
    full_paths = []
    
    for path in path_list:
        if os.path.isabs(path):
            full_path = os.path.expanduser(path)
        else:
            full_path = os.path.abspath(f"{cwd}/{path}")
        print(full_path)
        full_paths.append(full_path)
    
    return full_paths

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('args', type=str, nargs='*')

    shortcut_select_group = parser.add_mutually_exclusive_group()
    shortcuts = [
	'lsp',
    ]
    for sc in shortcuts:
        shortcut_select_group.add_argument(f'--{sc}', action='store_true')
    
    args = parser.parse_args()
    
    for sc in shortcuts:
        if getattr(args, sc):
            globals()[sc](args.args)
