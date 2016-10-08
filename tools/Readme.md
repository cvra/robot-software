# Tools help

## PID plots

You can plot a stream of PID data received from the robot via simpleRPC in messagepack format using
```bash
bokeh server plot_pid.py
```

The application assumes the data to come as an array of integers: [setpoint, measured].

In you web browser, you can watch the plot on `localhost:5006`.
You can also set the name of the topic containing the streamed data as well as the plot buffer via the text input widgets on that page.


## Requirements

You can install requirements using pip
```bash
pip install -r requirements.txt
```
