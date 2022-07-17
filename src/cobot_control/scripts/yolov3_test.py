import numpy as np
import time
import cv2
import sys


INPUT_FILE='14.jpg'
OUTPUT_FILE='predicted.jpg'
LABELS_FILE='classes.names'
CONFIG_FILE='yolov3-custom.cfg'
WEIGHTS_FILE='yolov3-custom_last.weights'
CONFIDENCE_THRESHOLD=0.3

#is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

cap = cv2.VideoCapture(-1)

class DNN :
    def __init__(self) :
        self.H, self.W = 0, 0
        self.result = []
        self.output = []

    def give_output( self , frame, choice = 0) :

        self.predict( frame )
        print("self.result : ",self.result)

        frame = self.result[-1]

        #print("Frame : ", frame )

        xcenter, ycenter = frame.shape[1]/2, frame.shape[0]/2

        cv2.line( frame , (int(xcenter) , 0) , (int(xcenter) , int(2*ycenter)) , (255,0,0) , 5)
        cv2.line( frame , (0 , int(ycenter)) , (int(xcenter*2) , int(ycenter)) , (255,0,0) , 5)


        try :

            self.x, self.y = self.result[choice][0], self.result[choice][1]
            self.w , self.h =  self.result[choice][2], self.result[choice][3]

            

            obj_xcenter, obj_ycenter = self.x + self.w/2, self.y + self.h/2        

            linear_x , linear_y , linear_z = [0]*3 

            if xcenter > obj_xcenter + self.w/2 :
                linear_x = -1

            elif xcenter < obj_xcenter - self.w/2 :
                linear_x = 1

            else:
                linear_x = 0

            if ycenter > obj_ycenter + self.h/2 :
                linear_y = 1
                
            elif ycenter < obj_ycenter - self.h/2 :
                linear_y = -1
                
            else:
                linear_y = 0
                
            self.output = [ frame , linear_x , linear_y, self.w ]
            return self.output

        except :
            print("Object with ID {} not found".format(choice))
            return [ frame, 0, 0, 0 ]



    def predict( self, frame ) :
        LABELS = open(LABELS_FILE).read().strip().split("\n")

        np.random.seed(4)
        COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
        	dtype="uint8")


        net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)
        '''if is_cuda:
            print("Attempty to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)'''

        #image = cv2.imread(INPUT_FILE)
        image = cv2.resize( frame, ( 640,640 ))
        (H, W) = image.shape[:2]

        

        # determine only the *output* layer names that we need from YOLO
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        	swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()


        #print("[INFO] YOLO took {:.6f} seconds".format(end - start))


        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
        	# loop over each of the detections
        	for detection in output:
        		# extract the class ID and confidence (i.e., probability) of
        		# the current object detection
        		scores = detection[5:]
        		classID = np.argmax(scores)
        		confidence = scores[classID]

        		# filter out weak predictions by ensuring the detected
        		# probability is greater than the minimum probability
        		if confidence > CONFIDENCE_THRESHOLD:
        			# scale the bounding box coordinates back relative to the
        			# size of the image, keeping in mind that YOLO actually
        			# returns the center (x, y)-coordinates of the bounding
        			# box followed by the boxes' width and height
        			box = detection[0:4] * np.array([W, H, W, H])
        			(centerX, centerY, width, height) = box.astype("int")

        			# use the center (x, y)-coordinates to derive the top and
        			# and left corner of the bounding box
        			x = int(centerX - (width / 2))
        			y = int(centerY - (height / 2))

        			# update our list of bounding box coordinates, confidences,
        			# and class IDs
        			boxes.append([x, y, int(width), int(height)])
        			confidences.append(float(confidence))
        			classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD,
        	CONFIDENCE_THRESHOLD)

        print("len(idxs) : ", len(idxs))

        
        result = []
        # ensure at least one detection exists
        if len(idxs) > 0:
            coords = []
            
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                coords = [x, y, w, h, i]

                result.append( coords )

                color = [int(c) for c in COLORS[classIDs[i]]]

                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

                text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                1, color, 2)
                
        result.append( image )
        self.result = result
        #print("Result : ", self.result)
        return self.result

# show the output image
#cv2.imwrite("example.png", image)

if __name__ == "__main__" :
    dnn = DNN()

    while True :

        ret, frame = cap.read()

        if ret :
            result = dnn.give_output( frame )
            print( result[1],result[2] )

            cv2.imshow( "Image", result[0]) 
            if cv2.waitKey(1) & 0xFF == ord('q') :
                exit()

cap.release()
cv2.destroyAllWindows()

    