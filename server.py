import torch
import torchvision
import socket;
import numpy

box=[0, 0, 0, 0, 0]
HOST = "127.0.0.1"
PORT = 8888
threshold=0.8
#Model Evaluation
resnet = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
resnet.eval()
COCO_INSTANCE_CATEGORY_NAMES = ['__background__', 'person','bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table', 'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
img=numpy.zeros((720,1280,3))

#Socket Programming
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
server_socket.bind((HOST, PORT))
server_socket.listen()

count = 0

while True:
    try:
        client_socket, addr = server_socket.accept()
        print('Connected by', addr)
        rgb_array_buffer=[]

        while len(rgb_array_buffer) != 2764800:
            data = client_socket.recv(1280)
            b = bytearray(data)
            rgb_array = numpy.frombuffer(b, dtype=numpy.uint8)
            rgb_array_buffer = numpy.concatenate((rgb_array_buffer,rgb_array),axis=0)
            

            if not data:
                break

        #Reshape
        rgb_array_total = numpy.reshape(rgb_array_buffer,(3,720,1280))
        rgb_tensor = torch.tensor(rgb_array_total)
        rgb_tensor/=255

        #Human Detection
        rgb_tensor = rgb_tensor.float()
        pred = resnet([rgb_tensor])
        pred_class = [COCO_INSTANCE_CATEGORY_NAMES[i] for i in list(pred[0]['labels'].numpy())]
        pred_boxes = [[(i[0], i[1]),(i[2], i[3])] for i in list(pred[0]['boxes'].detach().numpy())]
        pred_score = list(pred[0]['scores'].detach().numpy())
        pred_t = [pred_score.index(x) for x in pred_score if x>threshold][-1]
        pred_boxes = pred_boxes[:pred_t+1]
        pred_class = pred_class[:pred_t+1]
        
        box[0] = int(pred_boxes[0][0][0]*255/1280) #xmin
        box[1] = int(pred_boxes[0][0][1]*255/720) #ymin
        box[2] = int(pred_boxes[0][1][0]*255/1280) #xmax
        box[3] = int(pred_boxes[0][1][1]*255/720) #ymax
        box[4] = count

        count = count + 1

        if count >= 256:
            count = 0

        r=bytearray(box)
        client_socket.send(r)
        client_socket.close()
    except KeyboardInterrupt:
        break
    except:
        client_socket.close()
        continue

server_socket.close()