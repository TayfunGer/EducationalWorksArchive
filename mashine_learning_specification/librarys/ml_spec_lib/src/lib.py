import pandas as pd
import tensorflow as tf
from tensorflow import keras
import math
#import pandas_ml as pdml
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay, roc_curve, RocCurveDisplay, auc, precision_score, recall_score, f1_score, precision_recall_curve, PrecisionRecallDisplay, accuracy_score


class ML_SPEC:
    def __init__(self):
        pass

    def data_to_uniform_distribution(dataframe: pd.DataFrame, label_name: str = "label") -> pd.DataFrame:
        '''
        creates an uniform distribution in a dataset

        Params:
            dataframe (pandas.DataFrame): dataframe to be distributed uniformly
            label_name (str): name of the column for the labels
        Return:
            dataframe (pandas.DataFrame): dataframe that is uniformly distributed
        '''
        grouped_df = dataframe.groupby(label_name)
        return grouped_df.apply(lambda x: x.sample(
            grouped_df.size().min(), replace=True)).reset_index(drop=True)

    def dataframe_to_dataset(dataframe: pd.DataFrame, label_name: str = "label") -> tf.data.Dataset:
        '''
        converts a pandas dataframe to a tensorflow dataset

        Params:
            dataframe (pandas.DataFrame): dataframe that should be converted to a tf dataset
            label_name (str): name of the column for the labels
        Return:
            dataset (tf.data.Dataset): the converted data set
        '''
        dataframe = dataframe.copy()
        dataframe[label_name] = dataframe[label_name].astype(int)
        labels = dataframe.pop(label_name)
        dataset = tf.data.Dataset.from_tensor_slices((dict(dataframe), labels))
        dataset = dataset.shuffle(buffer_size=len(dataframe))
        return dataset

    def split_data_into_validation_training_test(dataframe: pd.DataFrame, frac_validation: int = 0.2, frac_test: int = 0.2, random_state: int = 42):
        '''
        split the pandas dataframe into 3 pandas dataframes validation_dataframe, test_dataframe, training_dataframe

        Params:
            dataframe (pandas.DataFrame): dataframe that should be splited
            frac_validation(int): fraction of which the validation frame consists
            frac_test(int): fraction of which the test frame consists
            random_state(int): value for the random generator for consistent data sets
        Return:
            Tupel[validation_dataframe, test_dataframe, training_dataframe]: tuple containing the split data frames
        '''
        validation_dataframe = dataframe.sample(
            frac=frac_validation, random_state=random_state)
        dataframe = dataframe.drop(validation_dataframe.index)
        test_dataframe = dataframe.sample(
            frac=frac_test, random_state=random_state)
        training_dataframe = dataframe.drop(test_dataframe.index)
        return validation_dataframe, test_dataframe, training_dataframe

    def encode_numerical_feature(feature, name: str, dataset):
        '''
        Normalizes the input data around the range 0 with a standard deviation of 1

        Params: 
            feature : The keras tensor that represents the feature
            name(str): Feature name
            dataset: Tensorflow dataset which contains the features
        Return:
            encoded_feature: The normalized feature
        '''
        normalizer = keras.layers.Normalization()
        feature_ds = dataset.map(lambda x, y: x[name])
        feature_ds = feature_ds.map(lambda x: tf.expand_dims(x, -1))

        # Learn the statistics of the data
        normalizer.adapt(feature_ds)

        # Normalize the input feature
        encoded_feature = normalizer(feature)
        return encoded_feature

    def calculate_new_label(v_ego: float, v_behind: float, d_behind: float, d_front: float, a_ego: float, a_behind: float, reactiontime: float):
        '''
        calculates the new value based on the braking deceleration, speed and distance 

        Params: 
            v_ego(float) : ego velocity
            v_behind(float): vehicle behind velocity
            d_behind(float): distance to the rear vehicle
            d_front(float): Distance to the vehicle in front
            a_ego(float): Braking delay ego
            a_behind(float): Braking delay other
            reactiontime(float): Response time to start breaking 
        Return:
            encoded_feature: The normalized feature
        '''
        breaking_distance_ego = ((v_ego / 3.6)**2) / (2*a_ego)
        breaking_distance_behind = ((v_behind/3.6)**2) / \
            (2*a_behind) + (v_behind / 3.6 * reactiontime)
        if breaking_distance_ego <= d_front and breaking_distance_behind <= d_behind:
            return True
        else:
            return False

    def apply_new_label_on_dataframe(dataframe: pd.DataFrame, a_ego: float, a_behind: float, reactiontime: float):
        '''
        Adds the new labels to the dataframe

        Params: 
            dataframe (pandas.DataFrame): dataframe where the labels should be added
            a_ego(float): Braking delay ego
            a_behind(float): Braking delay other
            reactiontime(float): Response time to start breaking
        Return:
            dataframe (pandas.DataFrame): dataframe where the labels were added
            columname (str): name of the added column
        '''
        label = dataframe.apply(lambda row: ML_SPEC.calculate_new_label(
            row["v_ego"], row["v_behind"], row["d_behind"], row["d_front"], a_ego, a_behind, reactiontime), axis=1)
        columname = "label_" + str(a_behind) + "_" + str(reactiontime)
        dataframe.insert(loc=len(dataframe.columns),
                         column=columname, value=label)
        return dataframe, columname

    def calculate_standard_deviation_from_uniform(a1: float, b1: float, a2: float, b2: float, T: float):
        '''
        calculates the standard deviation of the distance

        Params: 
            a1(float): lower limit speed vehicle_1
            b1(float): upper limit speed vehicle_1
            a2(float): lower limit speed vehicle_2
            b2(float): upper limit speed vehicle_2
            T(float): Scenario period
        Return:
            standard_devitation(float): standard deviation of the distance between 2 vehicles
            variance(float): variance of the distance between 2 vehicles
        '''
        var1 = (b1/3.6-a1/3.6)**2 / 12.0
        var2 = (b2/3.6-a2/3.6)**2 / 12.0
        standard_devitation = T * math.sqrt(var1 + var2)
        variance = standard_devitation**2
        return standard_devitation, variance

    def predict_test_date(dataframe: pd.DataFrame, model, label_name: str = "label"):
        '''
        The model performs a prediction based on all data in the dataframe

        Params: 
            dataframe (pandas.DataFrame): dataframe that serves as input
            model: The model which should perform the prediction
            label (str): The label which should serve as input from the dataframe
        Return:
            dataframe (pandas.DataFrame): Dataframe which contains the predicted and the true labels
        '''
        y_true = []
        y_pred = []
        for row in dataframe.iterrows():
            y_true.append(row[1][label_name])
            sample = {
                "v_front": row[1]["v_front"],
                "v_ego": row[1]["v_ego"],
                "v_behind": row[1]["v_behind"],
                "d_front": row[1]["d_front"],
                "d_behind": row[1]["d_behind"],
            }
            input_dict = {name: tf.convert_to_tensor(
                [value]) for name, value in sample.items()}
            prediction = model.predict(input_dict)
            y_pred.append(prediction[0][0])

        df = pd.DataFrame(list(zip(y_true, y_pred)),
                          columns=["y_true", "y_pred"])
        return df

    def compute_metrics(dataframe: pd.DataFrame):
        '''
        Compute metrics from dataframe

        Params: 
            dataframe (pandas.DataFrame): Dataframe with which the metrics are to be calculated
        Return:
            returns (float, float, float, float): The computet metrics precission, recall, f1 , accuracy 
        '''
        precision = precision_score(dataframe["y_true"], dataframe["y_pred"])
        recall = recall_score(dataframe["y_true"], dataframe["y_pred"])
        f1 = f1_score(dataframe["y_true"], dataframe["y_pred"])
        accuracy = accuracy_score(dataframe["y_true"], dataframe["y_pred"])
        return precision, recall, f1, accuracy

    def confusion_matrix(dataframe: pd.DataFrame):
        '''
        Creates a confusion matrix from dataframe

        Params: 
            dataframe (pandas.DataFrame): Dataframe from which the confusion matrix is to be created
        Return:
            cm_display(ConfusionMatrixDisplay): Visualization from the confusion matrix
        '''
        plt.rc('image', cmap="tab20c")
        plt.rc('font', size=18)
        cm = confusion_matrix(dataframe["y_true"], dataframe["y_pred"])
        cm_display = ConfusionMatrixDisplay(cm).plot()
        return cm_display

    def compute_labels_from_predictions(dataframe: pd.DataFrame, treshhold: float = 0.5):
        '''
        Calculates the labels from a dataframe using the given threshold

        Params: 
            dataframe (pandas.DataFrame): Dataframe from which the labels are to be calculated
        Return:
            dataframe (pandas.DataFrame): Dataframe wich contains the true and the predict labels
        '''
        df = dataframe.copy()
        df["y_true"] = df["y_true"].astype(int)
        df["y_pred"] = df["y_pred"].apply(
            lambda y: 1 if y > treshhold else 0)
        return df

    def compute_labels_only_from_predictions(dataframe: pd.DataFrame, treshhold: float = 0.5, label: str = "prediction_1"):
        '''
        Calculates the labels from a dataframe using the given threshold and a singelton label

        Params: 
            dataframe (pandas.DataFrame): Dataframe from which the labels are to be calculated
        Return:
            dataframe (pandas.DataFrame): Dataframe wich contains the predict labels
        '''
        df = dataframe.copy()
        df[label] = df[label].apply(
            lambda y: 1 if y > treshhold else 0)
        return df

    def check_collision_new_label(dataframe: pd.DataFrame, a_ego: float, a_behind: float, reactiontime: float):
        '''
        Test if a collision would occur with a new label

        Params: 
            dataframe (pandas.DataFrame): Dataframe from which is checked the collision
            a_ego (float): New braking deceleration ego
            a_other (float): New braking deceleration other
            reactiontime(float): Reaction time of a human driver
        Return:
            label (Series[bool]): Series which contains the new labels
        '''
        label = dataframe.apply(lambda row: ML_SPEC.calculate_new_label(
            row["v_ego"], row["v_behind"], row["d_behind"], row["d_front"], a_ego, a_behind, reactiontime), axis=1)
        return(label)
