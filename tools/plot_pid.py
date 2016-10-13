from cvra_rpc import message

from functools import partial
from threading import Thread
from socketserver import UDPServer
import time

from bokeh.models import ColumnDataSource
from bokeh.models.widgets import TextInput, Select
from bokeh.plotting import curdoc, figure
from bokeh.io import show
from bokeh.layouts import row

from tornado import gen


MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)

# this must only be modified from a Bokeh session allback
start = time.time()
pid_setpoint = ColumnDataSource(data=dict(x=[0], y=[0]))
pid_measured = ColumnDataSource(data=dict(x=[0], y=[0]))

# This is important! Save curdoc() to make sure all threads
# see then same document.
doc = curdoc()

# Topic selector
options = []
topics_received = []
topic_select = Select(title="Topic:", value="topic", options=options)

# Number of points to keep in plot buffer
plot_buffer = TextInput(value="400", title="Plot buffer:")

@gen.coroutine
def update(now, setpoint, measured):
    pid_setpoint.stream(dict(x=[now], y=[setpoint]), int(plot_buffer.value))
    pid_measured.stream(dict(x=[now], y=[measured]), int(plot_buffer.value))

def msg_cb(todo, msg, args):
    global topics_received
    topics_received.append(msg)

    if (msg == topic_select.value):
        now = time.time() - start
        setpoint = args[0]
        measured = args[1]

        # but update the document from callback
        doc.add_next_tick_callback(partial(update, now=now, setpoint=setpoint, measured=measured))

RequestHandler = message.create_request_handler({}, msg_cb)
msg_server = UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_sub_thd = Thread(target=msg_server.serve_forever)
msg_sub_thd.daemon = True
msg_sub_thd.start()

p = figure(plot_width=1500, plot_height=800)
l_setpoint = p.line(x='x', y='y', color="red", source=pid_setpoint, legend='Setpoint')
l_measured = p.line(x='x', y='y', color="blue", source=pid_measured, legend='Measured')

doc.add_root(row(children=[topic_select, plot_buffer]))
doc.add_root(p)

def update_topics_list():
    global topics_received
    topic_select.options = list(set(topics_received))
    topics_received = []

doc.add_periodic_callback(update_topics_list, 1000)
