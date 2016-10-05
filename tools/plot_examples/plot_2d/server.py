from cvra_rpc import message

from functools import partial
from threading import Thread
from socketserver import UDPServer
import time

from bokeh.models import ColumnDataSource
from bokeh.plotting import curdoc, figure

from tornado import gen


MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20000)

# this must only be modified from a Bokeh session allback
source = ColumnDataSource(data=dict(x=[0], y=[0]))

# This is important! Save curdoc() to make sure all threads
# see then same document.
doc = curdoc()

@gen.coroutine
def update(x, y):
    source.stream(dict(x=[x], y=[y]))

def msg_cb(todo, msg, args):
    print('receiving:', msg, args)
    x = args['x']
    y = args['y']

    # but update the document from callback
    doc.add_next_tick_callback(partial(update, x=x, y=y))

RequestHandler = message.create_request_handler({}, msg_cb)
msg_server = UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_sub_thd = Thread(target=msg_server.serve_forever)
msg_sub_thd.daemon = True
msg_sub_thd.start()

p = figure(x_range=[0, 1], y_range=[0,1])
l = p.circle(x='x', y='y', source=source)

doc.add_root(p)
