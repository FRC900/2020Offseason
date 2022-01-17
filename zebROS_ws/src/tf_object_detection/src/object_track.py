import numpy as np 
import cv2
from tracker import Tracker
import time
import imageio
images = []
import random
import re
from io import StringIO
import sys
import io
def createimage(w,h):
	size = (w, h, 1)
	img = np.ones((w,h,3),np.uint8)*255
	return img

def analysis(centers, tracker):

		print(tracker)

		old_stdout = sys.stdout
		new_stdout = io.StringIO()
		sys.stdout = new_stdout
		print(tracker)
		output = new_stdout.getvalue()
		sys.stdout = old_stdout

		diffrence = output
		file1 = open("Logs.txt", "a")  # append mode
		file1.write(output)
		file1.close()
		# diffrence = diffrence[-57:-32]
		diffrence = diffrence[diffrence.find('Prediction:[[') + 13:diffrence.find(']] Track')]
		print(diffrence)
		trackID = output[:diffrence.find(' SkippedFrames')]
		print(trackID)

		diffrence = diffrence.split()
		try:
			x = int(float(diffrence[0]))
			y = int(float(diffrence[1]))
			print(x, y)
			print(str(trackID) + "Diffrence between actual and expected=" + str(abs(x - i[0]) + abs(y - i[1])))

		except Exception as e:
			print(e)




def main():
	data = np.array(np.load('Detections.npy'))[0:10,0:150,0:150]
	#print(data[0].shape)
	#print(type(data))
	#stillarray = np.zeros((150,2))
	#stillarray[:, 0] = 255
	#stillarray[:, 1] = 255
	#data = np.concatenate((data,[stillarray]),axis= 0)
#
	#stillarray = np.zeros((150,2))
	#stillarray[:, 0] = 400
	#stillarray[:, 1] = 400
	#data = np.concatenate((data,[stillarray]),axis= 0)
#
	#stillarray[:, 0] = 111
	#stillarray[:, 1] = 111
	#data = np.concatenate((data,[stillarray]),axis= 0)

	#for i in range(data.shape[0] - 1):

	#data = np.delete(data, 0, 0)
	#data = np.delete(data, 1, 0)
	data = np.delete(data, 2, 0)
	data = np.delete(data, 3, 0)
	data = np.delete(data, 4, 0)
	data = np.delete(data, 4, 0)
	#data = np.delete(data, 3, 0)
	#data = np.delete(data, 2, 0)
	#data = np.delete(data, 1, 0)

	invalidNumber = 9999

	stillarray = np.arange(300).reshape(150,2)
	stillarray[stillarray > 151] = invalidNumber
	print(stillarray)
	data = np.concatenate((data, [stillarray]), axis=0)
	print(data[0])

	stillarray = np.arange(300).reshape(150,2)
	#stillarray[stillarray > 151] = 0
	print(stillarray)
	stillarray[:, 1] += 30
	data = np.concatenate((data, [stillarray]), axis=0)
#
	stillarray = np.arange(300).reshape(150,2)
	#stillarray[stillarray > 151] = 0
	print(stillarray)
	stillarray[:, 1] -= 30
	data = np.concatenate((data, [stillarray]), axis=0)
#
	#print(data.shape)
	#print(data[1])
	#print("\n\n\n\n")
#
	#print(data[2])




	tracker = Tracker(150, 30, 5)
	skip_frame_count = 0
	track_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127)]

	for i in range(data.shape[1]):

		centers = data[:,i,:]
		print(type(centers))
		print("\n\n")
		if i >= 75:
			print(centers)
			time.sleep(2)
		for idx, cord in enumerate(centers):
			if cord[0] == 9999 or cord[1] == 9999:
				print("Delted a center with " + str(invalidNumber))
				time.sleep(2)
				centers = np.delete(centers[0], idx)
				print(centers)



		#for ii in range(3):
		#if random.random() > .8:
		#	centers = np.delete(centers, random.randint(0, centers.shape[0]-1), 0)
		#	print("deleting data?")
		#	print(centers.shape)


		frame = createimage(512,512)

		if (len(centers) > 0):
			print(tracker)
			tracker.update(centers)


			#print(tracker)

			#analysis(centers, tracker)


			#print(len(tracker.tracks))
			time.sleep(.1)
			for j in range(len(tracker.tracks)):
				try:
					if (len(tracker.tracks[j].trace) > 1):
						x = int(tracker.tracks[j].trace[-1][0,0])
						y = int(tracker.tracks[j].trace[-1][0,1])
						tl = (x-10,y-10)
						br = (x+10,y+10)
						cv2.rectangle(frame,tl,br,track_colors[j],1)
						cv2.putText(frame,str(tracker.tracks[j].trackId), (x-10,y-20),0, 0.5, track_colors[j],2)
						cv2.putText(frame, str(i), (10, 10), 0, 0.5, track_colors[j], 2)
						for k in range(len(tracker.tracks[j].trace)):
							x = int(tracker.tracks[j].trace[k][0,0])
							y = int(tracker.tracks[j].trace[k][0,1])
							cv2.circle(frame,(x,y), 3, track_colors[j],-1)
						cv2.circle(frame,(x,y), 6, track_colors[j],-1)
					cv2.circle(frame,(int(data[j,i,0]),int(data[j,i,1])), 6, (0,0,0),-1)
				except Exception as e:
					print(e)
			cv2.imshow('image',frame)
			cv2.imwrite("image"+str(i)+".jpg", frame)
			images.append(imageio.imread("image"+str(i)+".jpg"))
			#time.sleep(0.1)
			if i >= 75:
				time.sleep(.5)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				cv2.destroyAllWindows()
				break

	imageio.mimsave('Multi-Object-Tracking.gif', images, duration=0.08)
			
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

if __name__ == '__main__':
	main()
