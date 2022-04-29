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

"""Common logging utility

This script provides helper functions for creating logger objects.

"""

import logging

def create_logger(name, level=logging.ERROR, fileName=None, maxFileSize=-1, maxFileCnt=2):
    """
    Helper function to create a logger object. This method supports setting up the logger:
    - To optionally use a single file or multiple files
    - If multiple files, then the ability to set maximum size of each file

    A negative value value for 'maxFileSize' would force the logging to a single file with
    a maximum size of 1 MB (Mega Byte).

    Args:
        name (str): Name of the logger object
        level (int): Log level. Any loglevel

    Returns:
        A fully constructed logger object
    """
    # Create a logger object
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # create formatter
    formatter = logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s')

    # Create a console logging handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    if fileName != None:
        if maxFileSize < 0:
            # Limit size to 1 MB
            maxFileSize = 1024 * 1024
            maxFileCnt  = 1

        # Create a file handler with max size
        file_handler = logging.RotatingFileHandler(fileName,
                                                   maxBytes=maxFileSize,
                                                   backupCount=maxFileCnt)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger
