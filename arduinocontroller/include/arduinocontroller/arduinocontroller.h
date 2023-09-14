#ifndef ARDUINOCONTROLLER_H
#define ARDUINOCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <stdio.h> 
#include <stdlib.h>     
#include <time.h> 
#include <fstream>
#include <iostream>


class Arduino
{
	public:
		Arduino();
		void spinner(void);
	private:
		ros::NodeHandle nh;
		void Callback(const std_msgs::Int8::ConstPtr& Corr);
		void CallbackUR(const std_msgs::Int8::ConstPtr& P);
		void NextPieceCallback(const std_msgs::Int8::ConstPtr& data);
		ros::Publisher PieceNumber;
		bool send_next_piece = true;
		int Cor = 0;
		int Pos = 0;
        int j = 0;
		int pe = 0;
		int k = 0;
		int n = 0;
		int pez = 0;
		std::vector<int> randpz;
		std::ofstream myfile;

        std_msgs::Int8 pnumb;
		ros::Subscriber PieceFind;
		ros::Subscriber PosUR;
		ros::Subscriber NextPieceSub;

		ros::Time starttime;
		ros::Time tappDown;
		ros::Time tappUp;
		ros::Time pospez;
		ros::Duration timepos;
		ros::Duration timeTapDown;
		ros::Duration timeTapUp;
};

#endif /* ARDUINOCONTROLLER_H */