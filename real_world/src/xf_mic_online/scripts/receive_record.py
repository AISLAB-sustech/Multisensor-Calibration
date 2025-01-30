#!/usr/bin/env python3
#encoding:utf-8
import rospy
import numpy as np
import os
from xf_mic_online.msg import   Pcm_Msg
from std_msgs.msg import Header
import struct
import threading
import queue as Queue
import torch
from SRP_PHAT import STFT,Covariance,SrpPhat
from utils.bss import Duet
import  matplotlib.pyplot as plt
import time

def extend_domain(quadrant, azi,cross):
    if azi >= 0 and azi < 90:
        belonging = 1
    elif azi >= 90 and azi < 180:
        belonging = 2
    elif azi >= 180 and azi < 270:
        belonging = 3
    elif azi >= 270 and azi < 360:
        belonging = 4
    
    if quadrant is not None:
        if belonging == 1 and quadrant == 4:
            cross = cross + 1
        elif belonging ==4 and quadrant == 1:
            cross = cross - 1
        azi =  azi + 360*cross
    return belonging, azi,cross

class sound_signal_extract:
    def __init__(self,audio_queue):
        self.queue = audio_queue
        self.dataset = 2
        self.abs_path = os.path.realpath(__file__)
        self.path = self.abs_path.replace("scripts/receive_record.py", "ros_record_audio/record_{}.pcm".format(self.dataset))
        # self.pcm_file = open(self.path,'wb')
        self.lane_sub = rospy.Subscriber('/master/mic/pcm/deno',Pcm_Msg, self.callback,tcp_nodelay=True,queue_size=5)
        print("***************************************\nNote that if the script does not terminate automatically, rerun it.\n***************************************") 
        print("Waiting for data...")

    def callback(self, msg):                                             
        # [chnnel 1 (char 4)+ chnnel 2 (char 4)+...+chnnel 8 (char 4)] *512 samples
        data = msg.pcm_buf
        # print(type(data))
        data = self.bytes_to_signed_integers(data)
        data = np.array(data,dtype=np.int32).reshape(-1, 8)[:,:6]
        self.queue.put(data)

    def bytes_to_signed_integers(self, byte_sequence):
        num_integers = 4096
        format_string = '<' + 'i' * num_integers
        integer_values = struct.unpack(format_string, byte_sequence)
        return integer_values

# filter the DOA estimation results
class kalman_filter(object):
    def __init__(self,Q=1e-1, R=1e-1):
        self.Q = Q
        self.R = R
        self.K = 0.0  
        self.P = 0.0 
        self.x = 0.0

    def update(self,data_last,data):
        self.x = data_last
        self.P = self.P+self.Q

        self.K = self.P/(self.P+self.R)
        self.x = self.x + self.K * (data-self.x)
        self.P = (1-self.K)*self.P
        return self.x

class sound_localization:
    def __init__(self):
        self.mics = torch.zeros((6, 2))
        angle = torch.Tensor([0, 60, 120, 180, -120, -180]) / 180 * np.pi
        rou = 70.5 / 1000 / 2
        for i in range(len(self.mics)):
            self.mics[i, :] = torch.Tensor([rou * torch.cos(angle[i]), rou * torch.sin(angle[i])])
        self.stft = STFT(sample_rate=16000)
        self.cov = Covariance()
        self.srpphat = SrpPhat(mics=self.mics, sample_rate=16000, speed_sound=346)
        self.pos_pub = rospy.Publisher('src_pos2', Header, queue_size=2)
        self.seq = 0

        # Outlier detection and filter
        self.smoothed_data = []
        self.window_size = 5
        self.smooth_filter = kalman_filter(Q=3e-2, R=9e-2)

        self.quadrant = None
        self.cross = 0

    def detect_and_replace_outliers(self,data, value, threshold=40):
        mean = np.mean(data)
        clean_data = np.copy(data)
        if abs(value - mean) > threshold: 
            clean_data = np.append(clean_data, mean)
        else:
            clean_data = np.append(clean_data, value)
        return clean_data

    def cal_doa(self, data,time_stamp):
        mask = np.ones(6, bool)
        audio_data = np.transpose(data)[mask, :]
        duet = Duet(audio_data, n_sources=1, sample_rate=16000, output_all_channels=True, )
        estimates = duet()
        audio_data = estimates.astype(np.float32)
        # audio_data = np.array([data]).astype(np.float32)
        Xs = self.stft(torch.from_numpy(audio_data))
        XXs = self.cov(Xs)
        doas = self.srpphat(XXs)
        doas = doas[:, 0, :]
        azi = torch.atan2(doas[0, 1], doas[0, 0])/np.pi*180
        
        if azi < 0: azi = azi+360
        self.quadrant, azi,self.cross = extend_domain(self.quadrant, azi,self.cross)

        # filter    
        if self.seq >= self.window_size:
            smoothed_data_in_window = self.smoothed_data[self.seq- self.window_size:]
            smoothed_data_in_window = self.detect_and_replace_outliers(smoothed_data_in_window, azi)
            azi = self.smooth_filter.update(self.smoothed_data[-1], smoothed_data_in_window[-1])
        
        x_hat = azi
        self.smoothed_data = np.append(self.smoothed_data, x_hat)
        self.seq = self.seq + 1
        self.pos_pub.publish(Header(self.seq,time_stamp,"{:.3f}".format(x_hat)))

def vertical_merge(matrix_block):
    result = matrix_block[0]
    for matrix in matrix_block[1:]:
        result = np.vstack((result, matrix))
    return result

result = []
def audio_process(audio_queue):
    global result
    localization = sound_localization()
    while not rospy.is_shutdown():
        if audio_queue.qsize() >= 8:
            time_stamp = rospy.Time.now()
            start = time.time()
            with lock:
                data1 = audio_queue.get() 
                data2 = audio_queue.get()
                data3 = audio_queue.get()
                data4 = audio_queue.get()
                data5 = audio_queue.get()
                data6 = audio_queue.get()
                data7 = audio_queue.get()
                data8 = audio_queue.get()
            audio_data = vertical_merge([data1,data2,data3,data4,data5,data6,data7,data8])
            localization.cal_doa(audio_data,time_stamp)
            end  = time.time()
            print("time cost: ",end - start)
    result = localization.smoothed_data
            
if __name__ == '__main__':
    rospy.init_node('listerner', anonymous=True)
    audio_queue = Queue.Queue(10)
    lock = threading.Lock()
    audio_process_thread = threading.Thread(
        target=audio_process,name="audio_process",args=(audio_queue,)).start()
    extractor = sound_signal_extract(audio_queue)
    while not rospy.is_shutdown():
        rospy.spin()

    plt.plot(result)
    plt.show()
    print("finish")
