#!/usr/bin/env python

'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Trautmann Jeremias, Wittenbeck Valerij

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from asr_world_model.msg import CompletePattern

def evaluateCompletePatterns(completePatterns):
    brotFound = False
    getraenkeFound = False
    muesliFound = False
    gedeckFound = False

    for completePattern in completePatterns:
        if (completePattern.patternName == "Brot_Gedeck_2"
                #and completePattern.confidence >= 0.9
                or completePattern.patternName == "Brot_Regal"):
            brotFound = True
        elif (completePattern.patternName == "Getraenke_Gedeck"
                or completePattern.patternName == "Getraenke_Regal"):
            getraenkeFound = True
        elif (completePattern.patternName == "Muesli_Gedeck"
                or completePattern.patternName == "Muesli_Regal"):
            muesliFound = True
        elif (completePattern.patternName == "Gedeck_Ende"
                or completePattern.patternName == "gedeck"):
            gedeckFound = True

    return (brotFound and getraenkeFound and muesliFound and gedeckFound)
