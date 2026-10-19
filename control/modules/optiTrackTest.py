import streaming

opTrack = streaming.optiTracker()

cnt = 0
while(cnt<10):
    opTrack.readSocket()
    cnt += 1

opTrack.optiSave()
opTrack.closeSocket()