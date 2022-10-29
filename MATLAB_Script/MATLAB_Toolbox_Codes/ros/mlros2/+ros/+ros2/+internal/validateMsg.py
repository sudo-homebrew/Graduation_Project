#!/usr/bin/env python3

# Modifications Copyright 2019-2021 The MathWorks, Inc.
#
# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import sys

import rosidl_parser
from rosidl_adapter import parser

CPPKEYWORDS = [
    'And',    'and_eq',   'asm',     'auto',
    'bitand', 'bitor',    'bool',    'break',
    'case',   'catch',    'char',    'class',
    'compl',  'const',    'const_cast', 'continue',
    'default','delete',   'do',      'double',
    'dynamic_cast','else','enum',    'explicit',
    'export', 'extern',   'false',   'float',
    'for',    'friend',   'goto',    'if',
    'inline', 'int',      'long',    'mutable',
    'namespace','new',    'not',     'not_eq',
    'operator','or',      'or_eq',   'private',
    'protected','public', 'register','reinterpret_cast',
    'return',  'short',   'signed',  'sizeof',
    'static',  'static_cast','struct','switch',
    'template','this',    'throw',   'true',
    'try',     'typedef', 'typeid',  'typename',
    'union',   'unsigned','using',   'virtual',
    'void',    'volatile','wchar_t', 'while', 
    'xor',     'xor_eq' ]

def is_valid_field_name_extended(name):
    try:
        m = parser.VALID_FIELD_NAME_PATTERN.match(name)
    except TypeError:
        raise InvalidResourceName(name)
    return m is not None and m.group(0) == name and name not in CPPKEYWORDS

def main(argv=sys.argv[1:]):
    argparser = argparse.ArgumentParser(
        description='Parse all recursively found .msg files.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(
        'paths',
        nargs='+',
        help='The base paths to search for .msg files')
    args = argparser.parse_args(argv)

    files = get_files(args.paths)
    idlparser = parser
    idlparser.is_valid_field_name = is_valid_field_name_extended
    for filename in files:
        pkg_name = os.path.basename(os.path.dirname(os.path.dirname(filename)))
        try:
            if filename.endswith('.msg'):
                idlparser.parse_message_file(pkg_name, filename)
            elif filename.endswith('.srv'):
                idlparser.parse_service_file(pkg_name, filename)
            #print(pkg_name, filename)
        except Exception as e:
            print('Message Package: ', pkg_name, ', File: ', filename)
            sys.exit(e)

    return 0


def get_files(paths):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension from "msg" folders
                for filename in sorted(filenames):
                    if filename.endswith('.msg') and os.path.basename(dirpath) == 'msg':
                        files.append(os.path.join(dirpath, filename))
                    elif filename.endswith('.srv') and os.path.basename(dirpath) == 'srv':
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return files


if __name__ == '__main__':
    sys.exit(main())