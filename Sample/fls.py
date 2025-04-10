#!/usr/bin/python3

from flask import Flask, render_template, request
import time
import subprocess as sb

app = Flask(__name__)

@app.route('/',methods=['GET'])
def root():
    return render_template('root.html')

@app.route('/takeoff',methods=['GET'])
def takeoff():
    # sb.run(["./ser",""])
    sb.Popen("./ser")
    return render_template('takeoff.html')

if __name__=='__main__':
    print("init !!")
    app.run(host='0.0.0.0', port=50000, debug=False)
