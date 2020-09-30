import glob
import pandas as pd
import os
import statsmodels.api as sm

log_dir = (os.path.join(os.path.expanduser('~'), '.scrimmage', 'experiments', 'my_first_parameter_varying'))
files = glob.glob(os.path.join(log_dir, '*_job_*', 'cpa.csv'))

agg = pd.DataFrame()
for file in files:
    run_num = int((os.path.basename(os.path.dirname(file))).split('_')[-1])
    frame = pd.read_csv(file)
    frame['run'] = run_num
    agg = pd.concat([agg, frame], copy=False)
agg = agg.reset_index(drop=True)

params_agg = pd.read_csv(os.path.join(log_dir, 'batch_params.csv'), index_col='run')

data = agg.join(params_agg, on='run')

entity1 = data[::2]

entity1.to_csv(os.path.join(log_dir, 'entity_1_data.csv'), index_label='index')
