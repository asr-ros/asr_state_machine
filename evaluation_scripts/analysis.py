'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Trautmann Jeremias, Wittenbeck Valerij

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import pandas as pd
import os
import os.path
import fnmatch
import math


def generate_analysis(path, filename):
  """ Generate state analysis in path

  Folder has to contain log.csv file
  """

  # replace first line with column titles
  log_path = os.path.join(path, 'log.csv')

  if not os.path.isfile(log_path):
    print(path + " does not contain log.csv")
    print("Skipping")
    return

  lines = []
  with open(log_path, 'r') as fin:
    lines = fin.readlines()

  if len(lines) == 0:
    # empty log file
    return

  lines[0] = "timestamp state outcome\n"
  
  with open(log_path, 'w') as fout:
    for line in lines:
      fout.write(line)

  df = pd.read_csv(log_path, sep=' ')


  # add delta column
  df['delta'] = (df['timestamp']-df['timestamp'].shift()).fillna(0)

  # new DataFrame with count column
  new_df = pd.DataFrame({'count': df.groupby('state')['delta'].count()})

  # add sum column
  new_df['sum'] = df.groupby('state')['delta'].sum()

  # add mean column
  new_df['mean'] = df.groupby('state')['delta'].mean()
  
  new_df.to_csv(os.path.join(path, filename))

  # this is not how you should do it but it gets the job done
  # it fixes the state column 
  new_df = pd.read_csv(os.path.join(path, filename))
  
  generate_moved_distance(path, new_df)
  # Update with MovedDistance; has to be writen a second time, because of state column fix
  new_df.to_csv(os.path.join(path, filename), index=False)
  print "experiment_mean for: " + str(path) + '\n' + str(new_df)


  runtime = df['timestamp'][df.last_valid_index()] - \
      df['timestamp'][df.first_valid_index()]

  print "Runtime: " +  str(runtime) + '\n'
  ret = {'runtime': runtime, 'dataframe': new_df}
  return ret

def generate_moved_distance(path, df):
  log_path = None
  
  for file in os.listdir(path):
    if fnmatch.fnmatch(file, 'state_machine*.log'):
      if log_path is not None:
        print("Multiple logs for state_machine found. This will be ignored: " + str(file))
      else:
        log_path = os.path.join(path, file)

  if log_path is None:
    print(str(path) + " does not contain state_machine*.log")
    print("Skipping")
    return

  lines = []
  with open(log_path, 'r') as fin:
    lines = fin.readlines()

  if len(lines) == 0:
    # empty log file
    return


  searchForInit = False
  searchForNext = False
  
  my_sum = 0.0
  count = 0.0
  
  oldX = 0.0
  oldY = 0.0
  currentX = 0.0
  currentY = 0.0
  
  for line in lines:
    # Gets the initial robot pose
    # This was in the log on 14.11.2016
    if "Initial robot state" in line:
      searchForInit = True
    elif searchForInit and "x: " in line:
      oldX = float(line.replace("x: ", ""))
    elif searchForInit and "y: " in line:
      oldY = float(line.replace("y: ", ""))
      searchForInit = False
    
    # Gets the poses in each move_base step
    # This was in the log on 14.11.2016
    elif "This is the actual robot pose after navigation: position:" in line:
      searchForNext = True
    elif searchForNext and "  x: " in line:
      currentX = float(line.replace("  x: ", ""))
    elif searchForNext and "  y: " in line:
      currentY = float(line.replace("  y: ", ""))
      # Add distance from current to old pose
      my_sum += getDistance(oldX, oldY, currentX, currentY)
      count += 1
      oldX = currentX
      oldY = currentY
      searchForNext = False

  if count == 0:
    # incomplete log file
	return

  mean = my_sum/count
  # Add new row to DataFrame
  df.loc[len(df)] = ["MovedDistance (in m)", count, my_sum, mean]
   
   
def getDistance(x1, y1, x2, y2):
  return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
    

def analyze_subfolders(path):
  """ Generates analysis in all subfolders with log.csv file """
  runtime_index = {}
  runtime_data = {}
  df_concat = {}
  
  # It's import to go bootomup, so that we can create the csv files in higher levels out of the basis
  for root, dirs, files in os.walk(path, topdown=False):
    abspathOfCurrentDir = os.path.abspath(root)
    if os.path.isfile(os.path.join(root, "log.csv")):
      ret = generate_analysis(root, os.path.basename(root) + ".csv")
      if ret is None:
        print "ret is none"
        continue
      runtime_index[abspathOfCurrentDir] = [root]
      runtime_data[abspathOfCurrentDir] = [ret['runtime']]
      df_concat[abspathOfCurrentDir] = [ret['dataframe']]
    # If no log.csv present it is most likely a dir containing subdirs, which already have csv files
    else:
      subDirPaths = get_immediate_subdirectories(abspathOfCurrentDir)
      appendDataAndCreateCsvForCurrentDepth(root, subDirPaths, runtime_index, runtime_data, df_concat)


def get_immediate_subdirectories(a_dir):
  return [os.path.abspath(os.path.join(a_dir, name)) for name in os.listdir(a_dir)
          if os.path.isdir(os.path.join(a_dir, name))]

def appendDataAndCreateCsvForCurrentDepth(currentDir, subDirPaths, runtime_index, runtime_data, df_concat): 
  all_runtime_index = []
  all_runtime_data = []
  all_df_concat = []
  
  # Make a new list containing all basis anylisis for the currentDir
  for subDirPath in subDirPaths:
    if subDirPath in runtime_index:
      all_runtime_index.extend(runtime_index[subDirPath])
      all_runtime_data.extend(runtime_data[subDirPath])
      all_df_concat.extend(df_concat[subDirPath])
    else:
      print "No csv in subDir: " + str(subDirPath)
  if len(all_runtime_index) == 0:
    print "There are no csv in the subdirs for the following dir: " + str(currentDir)
    return

  abspathOfCurrentDir = os.path.abspath(currentDir)
  runtime_index[abspathOfCurrentDir] = all_runtime_index
  runtime_data[abspathOfCurrentDir] = all_runtime_data
  df_concat[abspathOfCurrentDir] = all_df_concat

  createConcatCsv(runtime_index[abspathOfCurrentDir], runtime_data[abspathOfCurrentDir], df_concat[abspathOfCurrentDir], currentDir)

def createConcatCsv(runtime_indexes, runtime_datas, df_concates, path):  
  df_concat = pd.DataFrame()
  for df in df_concates:
    df_concat = pd.concat((df_concat, df))
  
  experiment_mean = pd.DataFrame()
  experiment_mean['count'] = df_concat.groupby('state')['count'].mean()
  experiment_mean['sum'] =  df_concat.groupby('state')['sum'].mean()
  experiment_mean['mean'] = df_concat.groupby('state')['mean'].mean()
  
  filePrefix = os.path.basename(path)
  print "experiment_mean for: " + str(path) + '\n' + str(experiment_mean)
  experiment_mean.to_csv(os.path.join(path, filePrefix + '_mean.csv'))
  
  pd.Series(runtime_datas, index=runtime_indexes).to_csv(os.path.join(path, filePrefix + '_runtimes.csv'))
  print "Runtime (mean): " +  str(reduce(lambda x, y: x + y, runtime_datas) / len(runtime_datas)) + '\n'


if __name__ == '__main__':
  print "This tool will generate analysis information from the log file(s). You can enter the path to the log file in the next step."
  print "You can create a hierarchy of log_folders. For each level there will be a runtimes.csv and experiment_mean.csv generated."
  path = str(raw_input("Enter path: "))
  analyze_subfolders(path)

