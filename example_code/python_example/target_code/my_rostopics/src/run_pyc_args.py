#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Code description.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import importlib.util
import sys
import os

def run_pyc_file(pyc_file, args):
    # save original sys.argv
    original_sys_argv = sys.argv
    try:
        # 将 sys.argv 替换为传递给 .pyc 文件的参数
        sys.argv = [pyc_file] + args
        spec = importlib.util.spec_from_file_location("__main__", pyc_file)
        module = importlib.util.module_from_spec(spec)
        sys.modules["__main__"] = module
        spec.loader.exec_module(module)
    finally:
        # restore original sys.argv
        sys.argv = original_sys_argv

if __name__ == "__main__":
    # modify .pyc file path accordingly to your needs
    pyc_path = os.path.join(os.path.dirname(__file__), 'publisher_node.cpython-37.pyc')
    # pass arguments from the command line
    run_pyc_file(pyc_path, sys.argv[1:])
