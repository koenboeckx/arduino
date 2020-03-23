from multiprocessing.connection import Client

address = ('localhost', 6000)
conn = Client(address, authkey=b'secret')
conn.send(['a', 2.5, None, int, sum])
conn.send('close')
conn.close()
