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
start = time.time()
source = ColumnDataSource(data=dict(x=[0], y=[start]))

# This is important! Save curdoc() to make sure all threads
# see then same document.
doc = curdoc()

@gen.coroutine
def update(var):
    source.stream(dict(x=[time.time() - start], y=[var]))

def msg_cb(todo, msg, args):
    print('receiving:', msg, args)
    x = args['x']

    # but update the document from callback
    doc.add_next_tick_callback(partial(update, var=x))

RequestHandler = message.create_request_handler({}, msg_cb)
msg_server = UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_sub_thd = Thread(target=msg_server.serve_forever)
msg_sub_thd.daemon = True
msg_sub_thd.start()

p = figure(y_range=[-1, 1])
l = p.circle(x='x', y='y', source=source)

doc.add_root(p)
