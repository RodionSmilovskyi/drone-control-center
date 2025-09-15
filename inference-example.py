import numpy as np
import tflite_runtime.interpreter as tflite

interpreter = tflite.Interpreter(model_path="master-model.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


input_data = np.array([0.5, 0.45], dtype=np.float32)
interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()


output_data = interpreter.get_tensor(output_details[0]['index'])
print(output_data)
print("ready")