import matplotlib.pyplot as plt
import pandas as pd
from ml_spec_lib.lib import ML_SPEC as ml
import os

UP_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

TEST_FILE = UP_DIR + "/data/lane_change_test_data.csv"
FILE_NAME = UP_DIR + "/data/auswertung_model_a_ego_6_a_other_6.csv"

# Load files
dataframe = pd.read_csv(FILE_NAME)
test_dataframe = pd.read_csv(TEST_FILE)


# Define treshholdrange (0.200 - 0.800)
treshholds = [i/1000 for i in range(200, 800)]

accuracys = []
precisions = []
f1_scores = []
recalls = []

# Iterate over all thresholds calculate the labels
# and the metrics for the respective thresholds
for treshhold in treshholds:
    df = ml.compute_labels_from_predictions(dataframe, treshhold)
    precision, recall, f1, accuracy = ml.compute_metrics(df)
    accuracys.append(accuracy)
    precisions.append(precision)
    f1_scores.append(f1)
    recalls.append(recall)

# Find treshhold for max. precision
df = ml.compute_labels_from_predictions(
    dataframe, treshholds[precisions.index(max(precisions))])
precision, recall, f1, accuracy = ml.compute_metrics(df)

# Print metrics for max. Precison and create confusion matrix
ml.confusion_matrix(df)
print(treshholds[precisions.index(max(precisions))])
print("--------------------------")
print("precision: ", end="")
print(precision)
print("1 - precision: ", end="")
print(1.0 - precision)
print("recall: ", end="")
print(recall)
print("f1_score: ", end="")
print(f1)
print("accuracy: ", end="")
print(accuracy)
print("--------------------------")
indices = df[(df["y_true"] == 0) &
             (df["y_pred"] == 1)].index

# Find treshhold for max. accuracy
df = ml.compute_labels_from_predictions(
    dataframe, treshholds[accuracys.index(max(accuracys))])
precision, recall, f1, accuracy = ml.compute_metrics(df)

# Print metrics for max. accuracy and create confusion matrix
ml.confusion_matrix(df)
print(treshholds[accuracys.index(max(accuracys))])
print("--------------------------")
print("precision: ", end="")
print(precision)
print("1 - precision: ", end="")
print(1.0 - precision)
print("recall: ", end="")
print(recall)
print("f1_score: ", end="")
print(f1)
print("accuracy: ", end="")
print(accuracy)
print("--------------------------")
result = test_dataframe.loc[indices]

# Check for possible collision if a is 6.5 m/s²
new_labels = ml.check_collision_new_label(result, 6.5, 6.5, 1.0)
print(new_labels.sum())

# Plot confusion matrix
plt.show()
