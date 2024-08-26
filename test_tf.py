
import tensorflow as tf
from dlclive import DLCLive, Processor

print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
