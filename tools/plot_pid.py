from cvra_rpc import message

from functools import partial
from threading import Thread
from socketserver import UDPServer
import time

from bokeh.models import ColumnDataSource
from bokeh.plotting import curdoc, figure

from tornado import gen


MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)
STREAM_ROLLOVER = 400 # number of points to keep in plot buffer

# this must only be modified from a Bokeh session allback
start = time.time()
pid_consign = ColumnDataSource(data=dict(x=[0], y=[0]))
pid_measured = ColumnDataSource(data=dict(x=[0], y=[0]))

# This is important! Save curdoc() to make sure all threads
# see then same document.
doc = curdoc()

@gen.coroutine
def update(now, consign, measured):
    pid_consign.stream(dict(x=[now], y=[consign]), STREAM_ROLLOVER)
    pid_measured.stream(dict(x=[now], y=[measured]), STREAM_ROLLOVER)

def msg_cb(todo, msg, args):
    if (msg == 'distance_pid'):
        now = time.time() - start
        print('[{}] receiving: {} {}'.format(now, msg, args))
        consign = args[0]
        measured = args[1]

        # but update the document from callback
        doc.add_next_tick_callback(partial(update, now=now, consign=consign, measured=measured))

RequestHandler = message.create_request_handler({}, msg_cb)
msg_server = UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_sub_thd = Thread(target=msg_server.serve_forever)
msg_sub_thd.daemon = True
msg_sub_thd.start()

p = figure(plot_width=1500, plot_height=800)
l_consign = p.line(x='x', y='y', color="red", source=pid_consign)
l_measured = p.line(x='x', y='y', color="blue", source=pid_measured)

doc.add_root(p)
