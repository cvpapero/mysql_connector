#!/usr/bin/env python
#coding: UTF-8

import sys, time
import rospy
import roslib
import json
import mysql.connector
from humans_msgs.msg import Humans
from geometry_msgs.msg import PointStamped
from humans_msgs.srv import Int32Srv
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
        self.comma = 0

        #self.db_init()

    def datetime(self):
        d = datetime.datetime.today()
        return str(d.year)+str(d.month)+str(d.day)

    def db_init(self, okao_id):
        self.r_name = self.datetime() +"-"+str(okao_id)+"-relations"
        qy = "create table `"+self.r_name+"` "+\
             "(state TINYINT, okao_id INT(11), hist DOUBLE, "+\
             "time_stamp DATETIME, name VARCHAR(45), "+\
             "laboratory VARCHAR(45), grade VARCHAR(45), tracking_id BIGINT(20), "+\
             "px DOUBLE, py DOUBLE, pz DOUBLE, joints TEXT)"
        
        try:
            print qy
            self.cur.execute(qy)
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_TABLE_EXISTS_ERROR:
                print "already exists."
            else:
                print err 

        self.ins_qy = "INSERT INTO `"+self.r_name+"` (state, okao_id,"+\
                      " hist, time_stamp, name, laboratory, grade, tracking_id,"+\
                      " px, py, pz, joints) VALUES "

    def rel_req(self, req):
        print "request okao_id:"+str(req.n)

        #create database for master
        self.db_init(req.n)

        #first get tracking id binded okao_id

        qy = "SELECT * FROM `"+self.t_name+"`"
        self.cur.execute(qy)
        res = self.cur.fetchall()
        print "len:" +str(len(res))
        get_data_num = 0
        for row in res:
            if row[1] == req.n and row[0] == 2:
                #input okao_id bind data
                self.rec_data(row)
                get_data_num = get_data_num + 1
            elif row[1] != req.n and row[0] == 2:
                self.com_data(row)
                get_data_num = get_data_num + 1

        #print get_data_num
        if get_data_num != 0:
            #print self.ins_qy
            self.cur.execute(self.ins_qy)
            self.con.commit()

        self.comma = 0

    def add_data(self, row):
        if self.comma == 1:
            self.ins_qy = self.ins_qy + ", "
        else:
            self.comma = 1
        state = str(row[0])
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
        self.ins_qy = self.ins_qy + " ("+state+", "+okao_id+", "+hist+", '"+time_stamp+\
                      "', '"+name+"', '"+laboratory+"', '"+grade+"', "+tracking_id+\
                      ", "+px+", "+py+", "+pz+", '"+joints+"')"


    def rec_data(self, row):
        self.add_data(row)
        self.okao_id = row[1]
        self.time_stamp = row[3]
        self.name = row[4]
        self.tracking_id = row[7]
        self.px = row[8]
        self.py = row[9]
        self.joints = row[10]

    def com_data(self, row):
        d_xy = math.sqrt( (self.px-row[8])**2 + (self.py-row[9])**2 )
        d_t = row[3] - self.time_stamp 
        delta_max = timedelta(seconds=self.d_max_t)
        delta_min = timedelta(seconds=0)
        #print type(row[3])
        if d_xy < self.d_max_xy and d_t <= delta_max and delta_min <= d_t:
            self.add_data(row)
            #this input human relations data
            print "ather okao_id:"+str(row[1])
            print "time now_data:"+str(row[3])
            print "time okao:"+str(self.time_stamp)
            print "t_delta:"+str(d_t.total_seconds())
            print "xy_delta:" + str(d_xy)



def main(args):
    hr = human_relations()
    rospy.init_node('human_relations', anonymous=True)
    try:
        rospy.spin()
    except KeybordInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
