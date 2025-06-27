#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import sys, os
import rospy
import numpy as np
import math
import std_msgs 
import smach
import yaml
import rosparam
import threading
import roslib.packages
from std_msgs.msg import String
from scipy.spatial import distance
from happymimi_msgs.srv import StrToStr, StrTrg, SetFloat, SimpleTrg, SetStr, SetStrRequest
from happymimi_navigation.srv import NaviLocation
from std_msgs.msg import Float64
from happymimi_voice_msgs.srv import TTS, YesNo, StringToString
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import StringToString,StringToStringResponse
from happymimi_msgs.srv import Str2Str, Str2StrRequest, StrToStr, StrToStrRequest
from happymimi_voice_msgs.srv import SpeechToText
from happymimi_voice_msgs.srv import TextToSpeech, TextToSpeechResponse
from happymimi_msgs.srv import StrToStrResponse
from happymimi_recognition_msgs.srv import RecognitionFind,RecognitionLocalize, RecognitionList,RecognitionLocalizeRequest,RecognitionListRequest
from happymimi_msgs.msg import StrInt
from happymimi_voice_msgs.srv import TextToSpeech
happymimi_voice_path=roslib.packages.get_pkg_dir("happymimi_voice")+"/.."
sys.path.insert(0,happymimi_voice_path)
from happymimi_recognition_msgs.srv import Clip, ClipResponse,depth_meterRequest,depth_meter
import time


file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

# 音声出力関数（サービスクライアント）
tts_topic = rospy.Publisher('mimic1/tts/topic', String, queue_size=10)
tts_service = rospy.ServiceProxy("mimic1/tts/service", TextToSpeech)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

tts_pub = rospy.Publisher('/mimic1/tts/topic', String, queue_size=10)
happymimi_voice_path=roslib.packages.get_pkg_dir("happymimi_voice")+"/.."
sys.path.insert(0,happymimi_voice_path)

yaml_path = roslib.packages.get_pkg_dir('person_feature_extraction') + "/config/extract.yaml"


def positionSum(list):
    position_dict = {}
    for i in list:
        position = i[0]
        if position in position_dict:
            position_dict[position] += 1
        else:
            position_dict[position] = 1

    return position_dict



ESTIMATE_POSTION_ANGLE = [
    ["chair_1_fmm",45,"long table"],
    ["chair_1_fmm",90,"long table"],
    ["chair_1_fmm",135,"long table"],
    ["chair_1_fmm",180,"long table"],
    ["chair_1_fmm",225,"long table"],
    ["chair_1_fmm",270,"long table"],
    ["chair_1_fmm",315,"long table"],
    ["chair_1_fmm",360,"long table"],
    ["operator_fmm",45,"long table"],
    ["operator_fmm",90,"long table"],
    ["operator_fmm",135,"long table"],
    ["operator_fmm",180,"long table"],
    ["operator_fmm",225,"long table"],
    ["operator_fmm",270,"long table"],
    ["operator_fmm",315,"long table"],
    ["operator_fmm",360,"long table"],

]

# 人の目の前までに寄る状態
class GetClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['get_close_finish','all_finish','continue_get_close'],
                            input_keys=['g_num', 'g_value','x_value','list_sum'],
                            output_keys=['g_num', 'g_value','x_value','list_sum'])

        self.coord_gen_srv = rospy.ServiceProxy('/human_coord_generator',StrToStr)
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 10)
        self.recognition_find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.recognition_locallize_srv = rospy.ServiceProxy('/recognition/localize', RecognitionLocalize)
        self.bc = BaseControl()
        self.depthMaskChange = rospy.ServiceProxy("depth_mask",depth_meter)
        self.current_position:str = ""
        self.current_angle:int = 0

        rospy.Subscriber("positionestimator/humanangle",Float64,self.humanAngleDef)
        self.humanAngle:float = 0

    def humanAngleDef(self,data):
        self.humanAngle = data.data

    def translateDist(self, kyori):
        self.bc.translateDist(kyori)
    
    def ttsSrv(self, message):
        tts_pub.publish(message)

    def ttsService(self, message):
        tts_service(message)

    def rotateAngle(self, roatate_angle)   :
        self.bc.rotateAngle(roatate_angle, 0, 0.9, 3)

    def headPub(self, angle):
        self.head_pub.publish(angle)


    def naviSrv(self, position):
        self.navi_srv(position)



    def depthMask(self, num):
        requestDepthMask = depth_meterRequest()
        requestDepthMask.meter = num
        self.depthMaskChange(requestDepthMask)

    def sortList():
        dict = {}
        list = []
        num = positionSum(ESTIMATE_POSTION_ANGLE)
        
        for i in userdata.list_sum.keys():
            for j in num.keys():
                if i == j:
                    dict[i] = num[i] / userdata.list_sum[i] 

        sorted_dict =  sorted(dict.items(), key=lambda x:x[1], reverse=True)
        for i in sorted_dict:
            key = i[0]
            for index,value in enumerate(ESTIMATE_POSTION_ANGLE):
                if value[0] == key:
                    list.append(ESTIMATE_POSTION_ANGLE_1[index])
        return list
                    

    def execute(self, userdata):
        rospy.loginfo("Executing state: APPROACH_GUEST")

        #探索済み人数
        g_num = userdata.g_num 
        print(f"find_person{g_num}")

        #探索済み場所
        g_value = userdata.g_value
        print(f"find_position{g_value}")
        
        if len(ESTIMATE_POSTION_ANGLE) => 0:
            return 'all_finish'


        estimate_postion = str(ESTIMATE_POSTION_ANGLE[0][0])
        estimate_angle = int(ESTIMATE_POSTION_ANGLE[0][1])

        
        navi_thread = threading.Thread(target=self.naviSrv, args=(estimate_postion,), name='navi_thread')
        rotateAngle_thread = threading.Thread(target=self.rotateAngle, args=(180,), name='rotateAngle_thread')
        head_pub_thread = threading.Thread(target=self.headPub, args=(-3,), name='head_pub_thread')
        tts_srv_thread_1 = threading.Thread(target=self.ttsSrv, args=("Start Find My Mates. Move to guest",), name='tts_srv_thread')
        tts_srv_thread_2 = threading.Thread(target=self.ttsSrv, args=("Move to guest",), name='tts_srv_thread')

        rotateAngle_thread.start()


        if g_num == 0 and g_value == 0:
            tts_srv_thread_1.start()       
            self.current_position = estimate_postion
            self.current_angle =
        else:
            tts_srv_thread_2.start()
            
        head_pub_thread.start()
        rotateAngle_thread.join()
                # y:float = 0
                # success_flag: bool = False

        rospy.loginfo(f"self.current_position:{self.current_position}  self.current_angle:{self.current_angle}")

        head_pub_thread.join()
        navi_thread.join()
        g_value == 0

        while True:
            if len(ESTIMATE_POSTION_ANGLE) <= g_value:
                return 'all_finish'
            # rospy.loginfo(f"now location:{g_value}" )
            estimate_postion = str(ESTIMATE_POSTION_ANGLE[0][0])
            estimate_angle = int(ESTIMATE_POSTION_ANGLE[0][1])


            if self.current_position != estimate_postion :
                navi_thread = threading.Thread(target=self.naviSrv, args=(estimate_postion,), name='navi_thread')
                navi_thread.start()
                rospy.loginfo(f"移動します〜{estimate_postion}")
                # self.navi_srv(estimate_postion)
                # self.current_angle = 0
                self.current_position = estimate_postion
                self.current_angle = 0

                navi_thread.join()




            roatate_angle = estimate_angle - self.current_angle
            rospy.loginfo(f"回転します〜 estimate_angle:{estimate_angle} - current_angle:{self.current_angle } = {roatate_angle}")
            self.current_angle = estimate_angle
            self.bc.rotateAngle(roatate_angle, 0, 0.4, 3)
                # y:float = 0
                # success_flag: bool = False
            # rotateAngle_thread.start()tts_srv

                # self.bc.rotateAngle(roatate_angle, 0, 0.4, 3)


            requestDepthMask = depth_meterRequest()
            requestDepthMask.meter = 2.4
            self.depthMaskChange(requestDepthMask)
            result = self.recognition_find_srv("person").result
            rospy.sleep(3)
            rospy.loginfo(result)
            if (result): #もし人がいたら
                # 人に近づく
                count = 0
                x:float = 0
                y:float = 0
                success_flag: bool = False
                while True:
                    try:
                        request = RecognitionLocalizeRequest()
                        request.target_name = "person"
                        request.sort_option = StrInt()
                        request.sort_option.data = "left"
                        request.sort_option.num = 0
                        response = self.recognition_locallize_srv(request) # このときXの値だけを取得する。なぜならこれはpositionEstimator.cppを実行したとき奥行きがXだから。x
                        position = response.point
                        x = position.x 
                        y = position.y
                        if math.isnan(x):
                            g_value += 1
                            userdata.g_value = g_value
                            del ESTIMATE_POSTION_ANGLE[0]

                            rospy.loginfo("math.insan")
                            break
                        if (x == 0):
                            tts_srv_thread = threading.Thread(target=self.ttsSrv, args=("Miss three dimentional!!!!",), name='tts_srv_thread')
                            tts_srv_thread.start()

                            # self.tts_srv("Miss three dimentional!!!!")
                            self.bc.translateDist(-0.3)
                            rospy.sleep(1.3)
                            if count > 3:
                                g_value += 1
                                userdata.g_value = g_value
                                rospy.loginfo(f"x == 0 {count}")

                                break
                            count = count + 1
                            continue
                        success_flag = True
                        break
                    except:                            
                        tts_srv_thread = threading.Thread(target=self.ttsSrv, args=("Can't find person.",), name='tts_srv_thread')
                        tts_srv_thread.start()
                        # self.tts_srv("Can't find person.")
                        self.bc.translateDist(-0.3)
                        rospy.sleep(1.3)

                        if count > 3:
                            # self.watched_corner = self.watched_corner + 1
                            g_value += 1
                            del ESTIMATE_POSTION_ANGLE[0]


                            userdata.g_value =g_value

                            rospy.loginfo(f"if count > 3 {count}")
                            
                            break
                        count = count + 1
                    tts_srv_thread = threading.Thread(target=self.ttsSrv, args=("MOU Ikkai",), name='tts_srv_thread')
                    tts_srv_thread.start()
                    # self.tts_srv("MOU Ikkai")
                    if count > 2:
                        kyori = 0.3 * 3
                        self.bc.translateDist(kyori)
                        rospy.sleep(3)
                        rospy.loginfo(f"if count > 2 {count}")

                        continue
                
                if success_flag:

                    # self.bc.rotateAngle(degree, 0.3, 0.5, 4)
                    rospy.loginfo("y:" + str(y) + "x:" + str(x))
                    # 距離を計算して直接近づく
                    try:
                        # arc_sin = np.arcsin(y/x)
                        # rospy.loginfo(arc_sin)
                        # degree = np.degrees(arc_sin)
                        # rospy.loginfo(degree)
                        # degree = float(degree) + 15
  
                        self.bc.translateDist(0.3 * count)
                        # rospy.loginfo(f"degreeは{degree}")

                        # if count != 0:
                        #     translate_dist_thread=threading.Thread(target=self.translate_dist_thread, args=(0.3 * count,), name='translate_dist_thread')
                        #     translate_dist_thread.start()
                        #     translate_dist_thread.join()

                        # if degree < 0:
                        #     rospy.loginfo("０より小さい角度で呼ばれた")
                        #     degree = 360 + degree + 15

                        rospy.loginfo(f"humanAngleは{self.humanAngle}")
                        rotateAngle_thread=threading.Thread(target=self.rotateAngle, args=(-self.humanAngle), name='rotateAngle_thread')
                        rotateAngle_thread.start()
                        # self.bc.rotateAngle(-self.humanAngle, 0.3, 0.5, 4)
                        # translate_dist_thread=threading.Thread(target=self.translateDist, args=(x - 0.5,), name='translate_dist_thread')
                        userdata.x_value = x -0.5

                        requestDepthMask = depth_meterRequest()
                        requestDepthMask.meter = 3
                        self.depthMaskChange(requestDepthMask)
                        rotateAngle_thread.join()
                        # userdata.g_value_out = userdata.g_value

                        userdata.g_value = g_value
                        return "get_close_finish"
                    except ValueError as e:
                        print(e)
                        userdata.g_value = g_value
                        rospy.loginfo("ValueError")
                        return "continue_get_close"

                    except:
                        userdata.g_value = g_value

                        break

                else:
                    break

            else:
                # self.watched_corner = self.watched_corner + 1
                g_value += 1
                userdata.g_value = g_value

        #whileから抜けてしまった場合一時的コード
        rospy.loginfo("whileから抜けてしまった")
        userdata.g_value = g_value
        return "continue_get_close"






class GetFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['get_feature_finish'],
                            input_keys=['g_num', 'g_value','g_sentence', 'g_str','x_value'],
                            output_keys=['g_num', 'g_value','g_sentence', 'g_str','x_value'],)
        
        self.getold_srv = rospy.ServiceProxy('/person_feature/old', SetStr)
        self.feature_srv = rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.per_fea_srv = rospy.ServiceProxy('/person_feature/gpt', Clip)
        self.feature_pub = rospy.Publisher('/clip_sign', String, queue_size = 10)
        #whisperを変える
        self.stt_question_srv = rospy.ServiceProxy('/whisper_stt', SetStr)
        self.yesno = rospy.ServiceProxy('/yes_no', YesNo)

        # depthMeter
        self.depthMaskChange = rospy.ServiceProxy("depth_mask",depth_meter)
        
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 10)
        self.objectList = rospy.ServiceProxy('/recognition/list',RecognitionList)

        # self.guest_name  = "null"
        self.object_list = []
        self.guest_loc   = "nuhead_publl"
        self.gn_sentence = "null"        
   
        self.bc = BaseControl()
        self.detectobjectList = []

        self.name = ""

    
    def translateDist(self, kyori):
        self.bc.translateDist(kyori)
    
    def ttsSrv(self, message):
        tts_pub.publish(message)
        rospy.sleep(2)

    def ttsService(self, message):
        tts_service(message)

    def rotateAngle(self, roatate_angle)   :
        self.bc.rotateAngle(roatate_angle, 0, 0.4, 3)

    def headPub(self, angle):
        self.head_pub.publish(angle)


    def naviSrv(self, position):
        self.navi_srv(position)



    def depthMask(self, num):
        requestDepthMask = depth_meterRequest()
        requestDepthMask.meter = num
        self.depthMaskChange(requestDepthMask)



    def getName(self):


        self.name = "null"
        self.ttsService("Hi..What's your name?")

        while True:
            self.name = self.stt_question_srv(SetStrRequest()).result
            self.ttsService("Are you"+ self.name +"? please answer, yes or no.")
            if self.yesno().result:
                break
            else:
                self.ttsService("What's your name again?")       


    def getNearObject(self):
        # 人の近くにある物を認識する。
        self.detectobjectList = []
        self.head_pub.publish(0)
        rospy.sleep(5)
        request = RecognitionListRequest()
        request.target_name = "any"
        request.sort_option= "left"
        self.detectobjectList = self.objectList(request).object_list
        # self.detectobjectList = [item for item in self.objectList if item != "person"]
        self.head_pub.publish(-20)
        rospy.sleep(1)
        return self.detectobjectList

    def depthmask(self, num):
        requestDepthMask = depth_meterRequest()
        requestDepthMask.meter = num
        self.depthMaskChange(requestDepthMask)

    def featurePub(self, str):
        self.feature_pub.publish(str)


    def execute(self, userdata):



        head_pub_thread = threading.Thread(target=self.headPub, args=(-20,), name='head_pub_thread')
        head_pub_thread.start()  

        translateDist_thread = threading.Thread(target=self.translateDist, args=(userdata.x_value,), name='translateDist_thread')
        translateDist_thread.start()  
        rospy.loginfo("Executing state: FIND_FUATURE")
        # self.head_pub.publish(-20)
        g_num = userdata.g_num
        g_value = userdata.g_value
        g_sentence = userdata.g_sentence
        g_str = userdata.g_str


        head_pub_thread.join()
        getName_thread = threading.Thread(target=self.getName, name='getName_thread')
        getName_thread.start()
        requestDepthMask = depth_meterRequest()
        requestDepthMask.meter = 8
        self.depthMaskChange(requestDepthMask)

        g_num = userdata.g_num


        if g_num == 0:
            future_num = "first"
            self.feature_pub.publish("first")
        elif g_num == 1:
            future_num = "second"
            # feature_pub_thread_2.start()
            self.feature_pub.publish("second")
        elif g_num == 2:
            future_num = "third"
            self.feature_pub.publish("third")
        else:
            return 'get_feature_finish'



        # if g_num == 0:
        #     rospy.sleep(3.0)
        #     self.feature_pub.publish("first")
        #     rospy.sleep(5.0)
        # elif g_num == 1:
        #     rospy.sleep(1.0)
        #     self.feature_pub.publish("second")
        # elif g_num == 2:
        #     rospy.sleep(1.0)
        #     self.feature_pub.publish("third")
        # else:
        #     return 'get_feature_finish'
        # feature_pub_thread.join()
        requestDepthMask.meter = 4
        self.depthMaskChange(requestDepthMask)


        # userdata.gn_out = self.gn_sentence



        getName_thread.join()
        # feature_pub_thread.join()
        self.gn_sentence = f"{self.name} 's near object is {str(ESTIMATE_POSTION_ANGLE[g_value][2])}" # Takumiが編集する
        self.bc.rotateAngle(180, 0.3, 0.5, 4)
        userdata.g_sentence = self.gn_sentence
        userdata.g_num = g_num + 1
        userdata.g_value = g_value + 1
        # feature_pub_thread.join()

        return 'get_feature_finish'

# ゲスト度に取得した特徴２つをオペレーターへ伝える状態
class Tell(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['tell_finish', 'all_finish'],

                            input_keys=['g_num', 'g_value','g_sentence', 'g_str'],
                            output_keys=['g_num', 'g_value','g_sentence', 'g_str'],)

        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)
        self.bc = BaseControl()
        self.sentence = ""

    def translateDist(self, kyori):
            self.bc.translateDist(kyori)
    
    def ttsSrv(self, message):
        tts_pub.publish(message)
        rospy.sleep(2)

    def rotateAngle(self, roatate_angle)   :
        self.bc.rotateAngle(roatate_angle, 0, 0.9, 3)

    def headPub(self, angle):
        self.head_pub.publish(angle)


    def naviSrv(self, position):
        self.navi_srv(position)



    def depthMask(self, num):
        requestDepthMask = depth_meterRequest()
        requestDepthMask.meter = num
        self.depthMaskChange(requestDepthMask)



    def execute(self, userdata):

        tts_srv_thread = threading.Thread(target=self.ttsSrv, args=("move to operator",), name='tts_srv_thread')
        tts_srv_thread.start()
        rotateAngle_thread = threading.Thread(target=self.rotateAngle, args=(180,), name='rotateAngle_thread')
        rotateAngle_thread.start()   
        rotateAngle_thread.join()
        tts_srv_thread.join()


        # navi_result = self.navi_srv('operator_fmm').result

        # inputとoutptに気を付ける
        count_num = userdata.g_num
        # wave_srv("/fmm/move_operator")

        navi_thread = threading.Thread(target=self.naviSrv, args=('operator_fmm',), name='navi_thread')
        navi_thread.start()

        g_num = userdata.g_num
        g_value = userdata.g_value
        g_sentence = userdata.g_sentence
        g_str = userdata.g_str


        # オペレーターへ自律移動
        # self.bc.rotateAngle(180, 1, 0.7, 5)
        # rospy.sleep(0.5)


        # self.head_pub.publish(-20)

        headPub_thread = threading.Thread(target=self.headPub, args=(-20,), name='headPub_thread')
        headPub_thread.start()   

        rospy.sleep(0.2)
        tts_srv_thread.join()
        navi_thread.join()
        headPub_thread.join()
    
        self.ttsSrv(str(g_sentence))   

        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
                
            # ファイルの内容が空でないか確認
            if data is not None:
                # 辞書内のすべての文を順に表示
                for key, value in data.items():
                    print(f"{key}: {value}")
                    # tts_srv(str(value))
                    # self.sentence = (str(value))
                    self.ttsSrv(str(value))

                    rospy.sleep(1.0)   
            else:
                rospy.logwarn("YAMLファイルが空です。")




        if count_num >= 3: 
            self.ttsSrv("Finish Find My Mates. Thank you very much")
            # wave_srv("/fmm/finish_fmm")
            return 'all_finish'
        else:
            return 'tell_finish'




# print(positionSum(ESTIMATE_POSTION_ANGLE))

if __name__ == "__main__":
    rospy.init_node('find_mm')
    rospy.loginfo("Start Find My Mates")
    

    # sm = smach()
    sm = smach.StateMachine(outcomes = ['fmm_finish'])
    sm.userdata.g_num = 0 # 完了したゲストの数
    sm.userdata.g_value = 0 # 完了したpositionの数
    sm.userdata.g_sentence = ""
    sm.userdata.g_str = ""
    sm.userdata.list_sum = positionSum(ESTIMATE_POSTION_ANGLE)
    starttime = time.time()


    with sm:
        smach.StateMachine.add("GetClose",
                               GetClose(),
                               transitions={"get_close_finish": "GetFeature",
                                            "all_finish": "fmm_finish",
                                            "continue_get_close":"GetClose"},
                               remapping={"g_num": "g_num",
                                        "g_value": "g_value",
                                        "x_value":"x_value,
                                        "list_sum":"list_sum"})
        smach.StateMachine.add("GetFeature",
                               GetFeature(),
                               transitions={"get_feature_finish": "Tell"},
                               remapping={"g_num": "g_num",
                                        "g_value": "g_value",
                                        "g_sentence":"g_sentence",
                                        "g_str":"g_str",
                                        "x_value":"x_value"})

        smach.StateMachine.add("Tell",
                               Tell(),
                               transitions={"tell_finish": "GetClose",
                                            "all_finish": "fmm_finish"},
                               remapping={"g_num": "g_num",
                                        "g_value": "g_value",
                                        "g_sentence":"g_sentence",
                                        "g_str":"g_str"})
    outcome = sm.execute()
    endtime = time.time()
    timediff = endtime - starttime
    print(f"zikkou zikan wa {timediff}")