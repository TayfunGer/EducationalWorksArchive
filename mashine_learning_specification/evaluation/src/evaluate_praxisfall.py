import matplotlib.pyplot as plt
import pandas as pd
from ml_spec_lib.lib import ML_SPEC as ml
import random
import os

UP_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
EVALUATION_FILE = UP_DIR + "/data/auswertung_praxisfall.csv"

df = pd.read_csv(EVALUATION_FILE)

# drop all data with NULL
df.dropna(inplace=True)

# Compute labels with treshhold for best accuracy
df = ml.compute_labels_only_from_predictions(df,  0.558, label="prediction_1")


def classify_and_count(row, count_dict):
    if row['collision'] == 0 and row["prediction_1"] == 0:
        count_dict['00'] += 1
        return '00'
    elif row['collision'] == 0 and row["prediction_1"] == 1:
        count_dict['01'] += 1
        return '01'
    elif row['collision'] == 1 and row["prediction_1"] == 0:
        count_dict['10'] += 1
        return '10'
    else:
        count_dict['11'] += 1
        return '11'


count_dict = {'00': 0, '01': 0, '10': 0, '11': 0}

# divide results into classes and count
df['class'] = df.apply(classify_and_count, args=(count_dict,), axis=1)

colors = ['orange', 'forestgreen', 'forestgreen', 'red']

# Create Histogramm and set x and y label
fig, ax = plt.subplots()
plt.bar(count_dict.keys(), count_dict.values(), width=0.5,
        color=colors)
plt.xlabel('Klasse', fontsize=14)
plt.ylabel('Anzahl', fontsize=14)

# Write the numbers on the bars of the histogram
for i, v in enumerate(count_dict.values()):
    plt.text(i, v, str(v), color='black', ha='center', va='bottom')

# Fit the diagram and plot it
ax.set_ylim(bottom=0.5)
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
plt.yticks(fontsize=12)
plt.show()
