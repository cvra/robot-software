import gdb
from collections import namedtuple

THREAD_STATES = [
    "READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM", "WTMTX",
    "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT", "SNDMSGQ", "SNDMSG",
    "WTMSG", "FINAL"
]

Thread = namedtuple('Thread', ('r13', 'stklimit', 'name', 'state', 'priority'))


def get_thread(thread):
    void_p = gdb.lookup_type('void').pointer()

    # Grab stack pointer
    r13 = thread['p_ctx']['r13'].cast(void_p)
    stklimit = thread['p_stklimit']
    name = thread['p_name'].string()
    prio = thread['p_prio']
    state = THREAD_STATES[int(thread['p_state'])]

    if not name:
        name = "<no name>"

    return Thread(
        r13=r13, stklimit=stklimit, name=name, state=state, priority=prio)


def follow_links(thread):
    newer = thread.dereference()['p_newer']
    older = thread.dereference()['p_older']

    return older, newer


def chibios_get_threads():
    """
    Iterates over the thread list of ChibiOS, yielding Thread structure each
    time.
    """
    # Walk the thread registry
    rlist_p = gdb.parse_and_eval('&ch.rlist')

    rlist_as_thread = rlist_p.cast(gdb.lookup_type('thread_t').pointer())

    older, newer = follow_links(rlist_as_thread)

    while newer != rlist_as_thread:
        yield get_thread(newer.dereference())

        previous = newer
        older, newer = follow_links(rlist_as_thread)

        if older != previous:
            raise gdb.GdbError('Rlist pointer invalid--corrupt list?')


class ThreadsCommand(gdb.Command):
    thread_format = "'{name:20s}'"

    def __init__(self):
        super(ThreadsCommand, self).__init__("chthreads", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        print(
            "Name                \tStack pointer\tStack limit\tState\tPriority")
        for t in chibios_get_threads():
            print("{:20s}\t0x{:08x}\t0x{:08x}\t{}\t{}".format(
                t.name,
                long(t.r13), long(t.stklimit), t.state, long(t.priority)))


ThreadsCommand()
