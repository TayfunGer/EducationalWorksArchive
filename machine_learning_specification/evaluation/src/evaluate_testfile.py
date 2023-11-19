import matplotlib.pyplot as plt
import pandas as pd
from ml_spec_lib.lib import ML_SPEC as ml
import os

UP_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

TEST_FILE = UP_DIR + "/data/lane_change_test_data.csv"
FILE_NAME = UP_DIR + "/data/auswertung_model_a_ego_6_a_other_6.csv"
FILE_NAME_1 = UP_DIR + "/data/auswertung_model_a_ego_4_5_a_other_4_5.csv"
FILE_NAME_2 = UP_DIR + "/data/auswertung_model_a_ego_6_a_other_3.csv"
FILE_NAME_3 = UP_DIR + "/data/auswertung_model_a_ego_7_a_other_5.csv"


dataframes = []
# Load files
dataframe = pd.read_csv(FILE_NAME)
dataframe_1 = pd.read_csv(FILE_NAME_1)
dataframe_2 = pd.read_csv(FILE_NAME_2)
dataframe_3 = pd.read_csv(FILE_NAME_3)
test_dataframe = pd.read_csv(TEST_FILE)

dataframes.append(dataframe)
dataframes.append(dataframe_1)
dataframes.append(dataframe_2)
dataframes.append(dataframe_3)

# Compute labels and metrics for each file
for dataframe in dataframes:
    df = ml.compute_labels_from_predictions(dataframe, 0.5)
    precision, recall, f1, accuracy = ml.compute_metrics(df)
    cm = ml.confusion_matrix(df)
    print("--------------------------")
    print("precision:", end="")
    print(precision)
    print("1 - precision:", end="")
    print(1.0 - precision)
    print("recall:", end="")
    print(recall)
    print("f1_score:", end="")
    print(f1)
    print("auc:", end="")
    print(accuracy)
    print("--------------------------")
# Plot confusion matrix
plt.show()
