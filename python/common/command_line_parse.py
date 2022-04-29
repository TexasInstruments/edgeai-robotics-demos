#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Command line parse utility

This scripts provides a helper function for parsing command line arguments,
to be used across demos.

Only the following functions are exported from this module:
    get_cmdline_args
"""

import sys
import argparse
import logging

# Exported methods
__all__ = ['get_cmdline_args']

class Parser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

logMap = {0:logging.DEBUG,
          1:logging.INFO,
          2:logging.WARNING,
          3:logging.ERROR}

def get_cmdline_args(sysv_args):
    '''
    Helper function to parse command line arguments

    Args:
        sysv_args: system command line argument array

    Returns:
        A dictionary containing parsed command line arguments.

    '''
    help_str = "Run : " + sysv_args[0] + " -h for help]"
    parser = Parser(usage = help_str,
                    formatter_class=argparse.RawTextHelpFormatter)

    help_str_config = "Path to demo config file\n" + \
                      "    ex: " + sysv_args[0] + " app_config.yaml"
    parser.add_argument("config", help = help_str_config)

    help_str_log = "Log level: 0:DEBUG | 1:INFO | 2:WARNING | 3:ERROR"
    parser.add_argument("-l", "--log_level",
                        help = help_str_log,
                        type=int,
                        choices=[0,1,2,3],
                        default = 3)

    args = parser.parse_args()

    args.log_level = logMap[args.log_level]
    return args

