#!/bin/bash

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

# Update chrony.conf
FILENAME=/etc/chrony/chrony.conf

STRING="server $PC_IP_ADDR iburst"

if [ $(sed -n '/^'"$STRING"'$/p' $FILENAME | wc -l) -eq 0 ]; then
    sed -i '17,20d' $FILENAME
    sed -i "17i server $PC_IP_ADDR iburst" $FILENAME

    if [ "$(uname -m)" == "x86_64" ]; then
        sed -i '33i # For local server setting' $FILENAME
        sed -i '34i local stratum 8' $FILENAME
        sed -i '35i manual' $FILENAME
        sed -i "36i allow $J7_IP_ADDR" $FILENAME
        sed -i $'36 a \n' $FILENAME
    fi
fi

# Run chrony
systemctl enable chrony
service chrony start

if [ "$(uname -m)" != "x86_64" ]; then
    chronyc tracking
fi
