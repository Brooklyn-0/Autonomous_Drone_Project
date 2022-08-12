import time
import os

num = 0
path = os.getcwd()


while True:
	os.system('sudo fswebcam -r 1980x1080 --save %s/webcampics/img%s' % (path, str(num)))
	num+=1
	time.sleep(1)
	if num == 10:
		break
