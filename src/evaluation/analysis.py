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
import csv
import sys


def generate_analysis(path, filename="state_analysis.csv"):
    """ Generate state analysis in path

    Folder has to contain log.csv file
    """

    # replace first line with column titles
    log_path = path + '/log.csv'
    print(log_path)

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

    # add mean column
    new_df['mean'] = df.groupby('state')['delta'].mean()

    # add sum column
    new_df['sum'] = df.groupby('state')['delta'].sum()
    
    new_df.to_csv(path + '/' + filename)

    # this is not how you should do it but it gets the job done
    # it fixes the state column 
    new_df = pd.read_csv(path + '/' + filename)


    runtime = df['timestamp'][df.last_valid_index()] - \
        df['timestamp'][df.first_valid_index()]

    print("Runtime " + str(runtime))
    ret = {'runtime': runtime, 'dataframe': new_df}
    return ret


def analyze_subfolders(path):
    """ Generates analysis in all subfolders with log.csv file """
    runtime_data = []
    runtime_index = []
    df_concat = pd.DataFrame()
    
    for (_, dirs, _) in os.walk(path):
        for dir in dirs:
            ret = generate_analysis(path + "/" + dir, dir + ".csv")
            if ret is None:
                print "ret is none"
                continue
            runtime = ret['runtime']
            runtime_index.append(dir)
            runtime_data.append(runtime)
            df_concat = pd.concat((df_concat, ret['dataframe']))

    pd.Series(runtime_data, index=runtime_index).to_csv('runtimes.csv')
    
    experiment_mean = pd.DataFrame()
    experiment_mean['count'] = df_concat.groupby('state')['count'].mean()
    experiment_mean['sum'] =  df_concat.groupby('state')['sum'].mean()
    experiment_mean['mean'] = df_concat.groupby('state')['mean'].mean()
    print experiment_mean

    experiment_mean.to_csv('experiment_mean.csv')


if __name__ == '__main__':
    path = str(raw_input("Enter path or nothing for pwd: "))
    analyze_subfolders(path)

