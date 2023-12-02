from flask import Flask, render_template, request
import pickle
import numpy as np

model = pickle.load(open('model-2.pkl','rb'))
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/predict',methods=['POST'])
def predict_zone():
    Sensor_1 = float(request.form.get('Sensor 1'))
    Sensor_2 = float(request.form.get('Sensor 2'))
    Sensor_3 = float(request.form.get('Sensor 3'))
    Sensor_4 = float(request.form.get('Sensor 4'))

    # prediction
    result = model.predict(np.array([Sensor_1,Sensor_2,Sensor_3,Sensor_4 ]).reshape(1, 4))

    if result[0] == 1:
        result = 'Zone 1'
    elif result[0] == 2:
        result = 'Zone 2'
    elif result[0] == 3:
        result = 'Zone 3'
    else:
        result = 'Zone 4'

    return render_template('index.html', result=result)

if __name__ == '__main__':
    print(__name__)
    app.run(host='0.0.0.0', port=8082)