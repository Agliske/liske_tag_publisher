/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#include <stdio.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>

#include <iostream>
#include <vector>

#include <modal_start_stop.h>
#include <modal_pipe_client.h>
#include "liske_hello_client.h"


// this is the directory used by the voxl-hello-server for named pipes
#define PIPE_NAME	"tag_detections"

// you may need a larger buffer for your application!
#define PIPE_READ_BUF_SIZE	1024
#define CLIENT_NAME	"liske-tag-publisher"

static int en_debug;
static int en_auto_reconnect = 0;	// set with --auto-reconnect arg






static void _print_usage(void)
{
	printf("\n\
This is a test of libmodal_pipe. It connects to the pipe dir /run/mpa/hello/\n\
created by modal-hello-server.\n\
\n\
Run this in debug mode to enable debug prints in the libmodal_pipe client code.\n\
You can also try the auto-reconnect mode which is useful for some projects but\n\
not all. This is a good example of using the simple helper feature.\n\
\n\
See the voxl-inspect-* examples in the voxl-mpa-tools repository for examples on\n\
reading other sorts of MPA data such as camera and IMU data\n\
\n\
-a, --auto-reconnect        enable auto-reconnect mode\n\
-d, --debug                 print debug info\n\
-h, --help                  print this help message\n\
\n");
	return;
}

// called whenever the simple helper has data for us to process
static void _simple_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	
	ros::Publisher* tag_pub_ptr = (ros::Publisher*) context; //we're assigning a type to the 'context', informing compiler that it is a ros::publisher
	
	liske_tag_publisher::tag_detection_msg msg;  //creating an empty message object with the custom message type tag_detection_msg

	//validating that the data from the pipe makes sense
	int n_packets, i;
	tag_detection_t* data_array = pipe_validate_tag_detection_t(data,bytes, &n_packets);

	
	if (data_array == NULL){
		printf("pipe is empty\n");
		return;
	} 


	//looping through detections
	for(i=0;i<n_packets;i++){
		
		tag_detection_t d = data_array[i];  //creating an data_detection_t object to read from.

		//filling in some basic data into the message
		msg.id = d.id;
		msg.size_m = d.size_m;
		msg.timestamp_ns = d.timestamp_ns;
		msg.loc_type = d.loc_type;

		
		
		// filling in tag rotation matrix relative to camera into the message
		int matrix_iterations = 0;
		std::vector<double> msg_R_tag_to_cam; //initialize an empty vector to hold our rotation matrix
		for(int i=0;i<3;i++){  
			for(int j=0;j<3;j++){
				
				
				msg_R_tag_to_cam.push_back(d.R_tag_to_cam[i][j]); //filling in the std::vector with the elements of our rotation matrix. This flattens the matrix
				matrix_iterations = matrix_iterations + 1;

			}
		}
		
		msg.R_tag_to_cam = msg_R_tag_to_cam; //filling in the flattened rotation matrix. the msg.R_tag_to_cam is technically a std::vector, so it needs to be
											 //fed that exact datatype otherwise it throws a fit during compilation. It wont take a C array

		
		//filling in [X,Y,Z] location vector into the message
		std::vector<double> msg_T_tag_wrt_cam;
		for(int i=0;i<3;i++){
			msg_T_tag_wrt_cam.push_back(d.T_tag_wrt_cam[i]);

		}
		msg.T_tag_wrt_cam = msg_T_tag_wrt_cam;

		//if the tag is configured for it in the configuration file, fill in the [X,Y,Z] location vector relative to a fixed position into the message
		if (d.T_tag_wrt_fixed != NULL){
			std::vector<double> msg_T_tag_wrt_fixed;
			for(int i=0;i<3;i++){
				msg_T_tag_wrt_fixed.push_back(d.T_tag_wrt_fixed[i]);

			}
			msg.T_tag_wrt_fixed = msg_T_tag_wrt_fixed;

		}


		//if the tag is configured for it in the configuration file, fill in the rotation matrix relative to a fixed orientation into the message
		if (d.R_tag_to_fixed != NULL){
			matrix_iterations = 0;
			
			std::vector<double> msg_R_tag_to_fixed;
			for(int i=0;i<3;i++){
				for(int j=0;j<3;j++){
				
					
					msg_R_tag_to_fixed.push_back(d.R_tag_to_fixed[i][j]);
					matrix_iterations = matrix_iterations + 1;

				}
			}
			msg.R_tag_to_fixed = msg_R_tag_to_fixed;
		}

		
		//publish the message object that we just filled in
		tag_pub_ptr->publish(msg); 
		
	}
	
	
	return;
}

// called whenever we connect or reconnect to the server
static void _connect_cb(int ch, __attribute__((unused)) void* context)
{
	// int ret;
	fprintf(stderr, "channel %d connected to server\n", ch);

	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "channel %d disconnected from server\n", ch);
	return;
}


// not many command line arguments
static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"auto-reconnect",	no_argument,		0,	'a'},
		{"debug",			no_argument,		0,	'd'},
		{"help",			no_argument,		0,	'h'},
		{0, 0, 0, 0}
	};
	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "adh", long_options, &option_index);
		if(c == -1) break; // Detect the end of the options.
		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;
		case 'a':
			en_auto_reconnect = 1;
			break;
		case 'd':
			en_debug = 1;
			break;
		case 'h':
			_print_usage();
			return -1;
		default:
			_print_usage();
			return -1;
		}
	}
	return 0;
}


int main(int argc, char* argv[])
{
	int ret;

	//initializing ros node and node handle.
	ros::init(argc, argv, "liske_tag_publisher");
	ros::NodeHandle nh;
	ros::Publisher tag_pub;
	tag_pub = nh.advertise<liske_tag_publisher::tag_detection_msg>("tag_detection_t", 100); //creating a publisher object that publishes the custom message "tag_detection_msg"

	ros::Publisher* tag_pub_ptr = &(tag_pub); //create a pointer to the tag_pub publisher object to pass as  context to the callback later on.
	//ros::Publisher is the type, the star means its a pointer to publisher object. tag_pub_ptr is the name of a variable storing the memory address of
	// the original tag_pub object. the & symbol means that it it collecting the memory address of the thing in the parentheses. In this case, my tag_pub object.


	

	// check for options
	if(_parse_opts(argc, argv)) return -1;

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// for this test we will use the simple helper with optional debug mode
	int flags 						= CLIENT_FLAG_EN_SIMPLE_HELPER;
	if(en_debug)			flags  |= CLIENT_FLAG_EN_DEBUG_PRINTS;

	// enable auto reconnect flag if requested
	if(!en_auto_reconnect){
		flags  |= CLIENT_FLAG_DISABLE_AUTO_RECONNECT;
	}
	else{
		// in auto-reconnect mode, tell the user we are waiting
		printf("waiting for server\n");
	}

	// printf("setting client callbacks");

	// assign callabcks for data, connection, and disconnect. the "NULL" arg
	// here can be an optional void* context pointer passed back to the callbacks
	pipe_client_set_simple_helper_cb(0, _simple_cb, (void*) tag_pub_ptr);  //we have passed the tag_pub_ptr pointer as context, so tag_pub can be accessed in the callback above
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// init connection to server. In auto-reconnect mode this will "succeed"
	// even if the server is offline, but it will connect later on automatically
	ret = pipe_client_open(0, PIPE_NAME, CLIENT_NAME, flags, PIPE_READ_BUF_SIZE);


	// keep going until signal handler sets the main_running flag to 0
	while(main_running){
		usleep(500000);
	}

	// all done, signal pipe read threads to stop
	printf("closing\n");
	fflush(stdout);
	pipe_client_close_all();

	return 0;
}
