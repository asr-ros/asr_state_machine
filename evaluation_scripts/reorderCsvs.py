'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Trautmann Jeremias, Wittenbeck Valerij

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import os
import os.path

order = ["Zustand", "state", "ObjectSearchInit", "DirectSearchInit", "SearchInit", "SceneRecognition", "PosePrediction", "CropBoxGeneration", "NBVSetPointCloud", "GetGoalCameraPose", "NextBestView", "GenerateRandomPose", "MoveBase", "FakeMoveBase", "MovePTU", "PTUPoseCorrection", "FrustumViz", "VisualizeWaypoints", "ObjectDetection", "NextBestViewUpdate", "CropboxStateRecording", "GridInitStateRecording", "CheckSearchFinished", "Total", "MovedDistance"]

toRemove = ["ObjectSearchInit", "DirectSearchInit", "SearchInit", "FrustumViz", "VisualizeWaypoints", "CheckSearchFinished", "Total"]

def isNumber(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def roundAllValues(content):
  content = [x.strip() for x in content]
  newContent = []
  
  for line in content:
    countEach = 0
    isFirst = True
    newLine = str()
    for each in line.split(','):
      if isFirst:
        isFirst = False
      else:
        newLine += ","
      if isNumber(each):
        newLine += "{0:.2f}".format(float(each))
        if line.startswith("MovedDistance") and countEach >= 2:
          newLine += " m"
      else:
        newLine += each
      countEach += 1
    newLine += "\n"
    newContent.append(newLine)
  return newContent

def isToRemove(line):
  for removeLine in toRemove:
    if line.startswith(removeLine):
      return True
  return False

def calculateTotal(content):
  totalSum = 0.0

  for line in content:
    if line.startswith("Zustand") or line.startswith("state") or line.startswith("MovedDistance") or line.startswith("Total"):
      continue
    
    eachs = line.split(',')
    totalSum += float(eachs[2])

  return "Total,-," + "{0:.2f}".format(totalSum / 60.0) + " min,-\n"

def reorderFile(file):
  with open(file, 'r') as f:
    content = f.readlines()

  content = roundAllValues(content)
  content = [line for line in content if not isToRemove(line)]
  content.append(calculateTotal(content))
      
  reorderedContent = []
  for newOrder in order:
    for line in content:
      if line.startswith(newOrder):
        reorderedContent.append(line)
        break
  
  for line in content:
    isFound = False
    for reordereLine in reorderedContent:
      if line == reordereLine:
        isFound = True
        break
    if isFound == False:
      print ("Unknown, will be append to end: %s" % line)
      reorderedContent.append(line)
  
  with open(file, 'w') as f:
    for line in reorderedContent:
      f.write("%s" % line)

def reorderCsvs(path):
  for root, dirs, files in os.walk(path):
    for name in files:
      if ".csv" in name and "mean" in name:
        reorderFile(os.path.join(root, name))

if __name__ == '__main__':
  print "This tool will reorder the states of the generated logs by analysis.py, according to the order in the script. Furthermore, some states will be deleted, a state for total will be added and all values will be rounded to two digits."
  print "Enter the path to the top log folder"
  path = str(raw_input("Enter path: "))
  reorderCsvs(path)

