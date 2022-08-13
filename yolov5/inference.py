import torch
import glob


model = torch.hub.load('ultralytics/yolov5', 'custom', path='runs/train/model1best.pt')  # add your own path
#model = torch.hub.load('path/to/yolov5', 'custom', path='path/to/best.pt', source='local')  # local repo

im = '../datasets/model_train1/test/images/IMG_8075_JPG.rf.d353e22c614d3bf6fc26209e52f08c57.jpg'  # add your image path
result = model(im)
print(result.pandas().xyxy[0])
result.show()

#for image in glob.glob('../datasets/model_train1/test/images/*.jpg'):
#	results = model(image)
#	results.pandas()
