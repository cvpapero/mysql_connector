#!/usr/bin/env python
#coding: UTF-8

import sys, time
import rospy
import roslib
import json
import mysql.connector
from humans_msgs.msg import Humans
from geometry_msgs.msg import PointStamped
from humans_msgs.srv import *
from std_msgs.msg import String
import math
import datetime
from datetime import timedelta
import locale


config = {
    'user': 'root',
    'password': 'robot15',
    'host': '127.0.0.1',
    'db': 'data1',
    'charset':'utf8'
}

class human_relations:

    def __init__(self):
        self.t_name = '201551'
        self.r_name = ''
        self.con = mysql.connector.connect(**config)
        self.cur = self.con.cursor()
        self.rel_srv = rospy.Service('relation', Int32Srv, self.rel_req)
        self.viz_srv = rospy.Service('viz_relation', Int32Srv, self.viz_req)
        self.time_srv = rospy.Service('time_relation', TimeZoneSrv, self.time_req)
        self.select_pub = rospy.Publisher('select_elem', String, queue_size=10)
        self.time_pub = rospy.Publisher('select_time', String, queue_size=10)

        self.okao_id = 0
        self.time_stamp = datetime.datetime.now()
        self.name = ''
        self.tracking_id = 0
        self.px = 0
        self.py = 0
        self.joints = ''
        self.ins_qy = ''
        self.d_max_xy = 2
        self.d_max_t = 10
        self.t_buf = {}
        self.stream = 0

        self.select_msg = ""
        #self.db_init()

    def datetime(self):
        d = datetime.datetime.today()
        return str(d.year)+str(d.month)+str(d.day)

    def db_init(self, okao_id):
        self.r_name = self.datetime() +"-"+str(okao_id)+"-relations"
        qy = "create table `"+self.r_name+"` "+\
             "(stream INT(11), okao_id INT(11), hist DOUBLE, "+\
             "time_stamp DATETIME, name VARCHAR(45), "+\
             "laboratory VARCHAR(45), grade VARCHAR(45), tracking_id BIGINT(20), "+\
             "px DOUBLE, py DOUBLE, pz DOUBLE, joints TEXT)"
        try:
            self.cur.execute(qy)
            print qy
            self.ins_qy = "INSERT INTO `"+self.r_name+"` (stream, okao_id,"+\
                          " hist, time_stamp, name, laboratory, grade, tracking_id,"+\
                          " px, py, pz, joints) VALUES "
            return True
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_TABLE_EXISTS_ERROR:
                print "already exists."
            else:
                print err 
            return False



    def rel_req(self, req):
        okao_id = req.n
        print "request okao_id:"+str(okao_id)
        #create database for master
        res = String()
        if self.db_init(okao_id):
            #first get tracking id binded okao_id
            self.tracking_select(okao_id)
            #second get okao id binded all data
            self.okao_select(okao_id)
            res = 'analysis ok! id:'+str(okao_id)
        else: 
            res = 'already exist! id:'+str(okao_id)

        return humans_msgs.srv.Int32SrvResponse(res)

    #network vizualization
    def viz_req(self, req):
        okao_id = req.n
        print "request okao_id:"+str(okao_id)
        self.r_name = self.datetime() +"-"+str(okao_id)+"-relations"
        self.req_select(okao_id)
        msg = String()
        msg.data = self.select_msg
        self.select_pub.publish(msg)
        res = String()
        res = 'set data id:'+str(okao_id)
        return humans_msgs.srv.Int32SrvResponse(res)

    def time_req(self, req):
        start = req.start
        end = req.end
        okao_id = req.n
        print "start:"+start+", end:"+end
        self.r_name = self.datetime() +"-"+str(okao_id)+"-relations"
        self.time_select(start, end)
        msg = String()
        msg.data = self.select_msg
        self.time_pub.publish(msg)
        return humans_msgs.srv.TimeZoneSrvResponse()

    def tracking_select(self, okao_id):
        first_qy = "SELECT DISTINCT tracking_id FROM `"+\
                   self.t_name+"` WHERE state = 2 and okao_id = "+str(okao_id)
        try:
            self.cur.execute(first_qy)
            res = self.cur.fetchall()
            index = 0
            for row in res:
                self.t_buf.update({row[0]:index})
                index = index + 1
            return True
        except mysql.connector.Error as err:
            print err
            return False 


    def okao_select(self, okao_id):
        second_qy = "SELECT * FROM `"+self.t_name+"`"
        self.cur.execute(second_qy)
        res = self.cur.fetchall()
        #print "len:" +str(len(res))
        get_data_num = 0
        for row in res:
            if row[1] == okao_id and row[0] == 2:
                #input okao_id bind data
                self.rec_data(row)
                get_data_num = get_data_num + 1
            elif row[1] != okao_id and row[0] == 2:
                self.com_data(row)
                get_data_num = get_data_num + 1
                
        if get_data_num != 0:
            self.cur.execute(self.ins_qy[:-1])
            self.con.commit()
            print "analysis ok"
            return True
        else:
            return False



    def add_data(self, row):

        stream = str(self.stream)
        okao_id = str(row[1])
        hist = str(row[2])
        time_stamp = str(row[3])
        name = str(row[4])
        laboratory = str(row[5])
        grade = str(row[6])
        tracking_id = str(row[7])
        px = str(row[8])
        py = str(row[9])
        pz = str(row[10])
        joints = str(row[11])
        self.ins_qy = self.ins_qy + " ("+stream+", "+okao_id+", "+hist+", '"+time_stamp+\
                      "', '"+name+"', '"+laboratory+"', '"+grade+"', "+tracking_id+\
                      ", "+px+", "+py+", "+pz+", '"+joints+"'),"


    def rec_data(self, row):
        self.okao_id = row[1]
        self.time_stamp = row[3]
        self.name = row[4]
        self.tracking_id = row[7]
        self.px = row[8]
        self.py = row[9]
        self.joints = row[10]
        self.stream = self.t_buf[self.tracking_id]

        self.add_data(row)

    def com_data(self, row):
        if self.okao_id != 0:
            d_xy = math.sqrt( (self.px-row[8])**2 + (self.py-row[9])**2 )
            d_t = row[3] - self.time_stamp 
            delta_max = timedelta(seconds=self.d_max_t)
            delta_min = timedelta(seconds=0)
            #print type(row[3])
            if d_xy < self.d_max_xy and d_t <= delta_max and delta_min <= d_t:
                self.add_data(row)
                #this input human relations data
                #print "ather okao_id:"+str(row[1])
                #print "time now_data:"+str(row[3])
                #print "time okao:"+str(self.time_stamp)
                #print "t_delta:"+str(d_t.total_seconds())
                #print "xy_delta:" + str(d_xy)

    def time_select(self, start, end):
        self.select_msg = "["
        time_qy = "SELECT * FROM data1.`"+\
                   self.r_name+"` where time_stamp between '"+ start + "' and '" + end + "'"
        print time_qy
        self.cur.execute(time_qy)
        res = self.cur.fetchall()

        for row in res:
            self.select_msg = self.select_msg + self.json_line(row) + ", "

        self.select_msg = self.select_msg[:-2] + " ]"
        print "time:"+self.select_msg 
        return True

    def req_select(self, okao_id):
        self.select_msg = "["
        select_qy = "SELECT * FROM data1.`"+\
                   self.r_name+"`"
        print select_qy
        self.cur.execute(select_qy)
        res = self.cur.fetchall()

        for row in res:
            self.select_msg = self.select_msg + self.json_line(row) + ", "

        self.select_msg = self.select_msg[:-2] + " ]"
        print "select:"+self.select_msg 
        return True

    def json_line(self, row):
        line = {"stream":str(row[0]),"okao_id":str(row[1]),"hist":str(row[2]),
                "time_stamp":str(row[3]),
                "name":str(row[4]),"laboratory":str(row[5]),"grade":str(row[6]),
                "tracking_id":str(row[7]),
                "px":str(row[8]),"py":str(row[9]), "pz":str(row[10])}
        #, "joints":str(row[11])}
        return json.dumps(line)
    

def main(args):
    hr = human_relations()
    rospy.init_node('human_relations', anonymous=True)
    try:
        rospy.spin()
    except KeybordInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
