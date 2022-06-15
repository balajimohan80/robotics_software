#include <iostream>
#include <utility>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"
#include "ball_chaser/DriveToTargetRequest.h"
#include "ball_chaser/DriveToTargetResponse.h"

class Process_Image {
    public:
        Process_Image(ros::NodeHandle&n, const std::string &sub_node_name, const std::string &req_srvr_name):
        nh(n), client(nh.serviceClient<ball_chaser::DriveToTarget>(req_srvr_name, this)), 
        cam_rgb_img_Sub(n.subscribe(sub_node_name, 10, &Process_Image::process_image_callback, this)),
        last_velocity(0.0f, 0.0f) 
        {}
        
       
    private:
        //Define RGB8 camera Pixels
        struct rgb8 {
            unsigned char r;
            unsigned char g;
            unsigned char b;
            constexpr bool operator ==(const rgb8& rhs) {
   	    	    return this->r == rhs.r && this->g == rhs.g && this->b == rhs.b;
            }
        };

        using linear_vel_x = float;
        using angular_vel_z = float;
        using robo_velocity = std::pair<linear_vel_x, angular_vel_z>;
        const rgb8 white_Pix = {255, 255, 255};
     
        //Ros NodeHandle
        ros::NodeHandle nh;

        //Request Response client
        ros::ServiceClient client;

        //Subcriber
        ros::Subscriber cam_rgb_img_Sub;

        //Store last known velocity
        //This variable is used to avoid continous request to drive-bot
        //Request to driver-bot node only when change in velocity
        robo_velocity last_velocity;

        /* This member function is used to request to drive
         *  the robot in specific direction.
         */
        void drive_robot(float lin_x, float ang_z) {
            ball_chaser::DriveToTarget srv_req;
            srv_req.request.linear_x  = lin_x;
            srv_req.request.angular_z = ang_z;
            if (!client.call(srv_req)) {
                ROS_ERROR("Error to call service driver_bot req!!!");
            }
        }

        /* This member function is a call-back function and get called by publisher publish camera-image.
         * It has an inteligence to identify the white-ball in the published image by walking through 
         * the each pixel value and check whether white pixel value is detected or not.
         * If the white pixel is detected, based on the pixel col's, it will detemine
         * robot velocity to white's ball direction.
         */
        void process_image_callback(const sensor_msgs::Image &img) {
            //Partition the image frame into 3(Left, Mid, Right).
            //Set the left partition range between 0 and 266(width=800, width/3)
            static std::pair<int, int> left_range  = {0, static_cast<int>(img.width/3)};
            //Set the mid partition range between 267 and 532(266*2) 
            static std::pair<int, int> mid_range   = {left_range.second+1, left_range.second << 1};
            //Set the right partition range between 533 and 800
            static std::pair<int, int> right_range = {mid_range.second+1, img.width};
            //Compute the total image size
            static const size_t image_size = img.height * img.width;
	    static const float total_pixels = static_cast<float>(image_size);
            #if 0       
            std::cout << left_range.first << " " << left_range.second << " " << mid_range.first << " " << mid_range.second << " " 
                      << right_range.first << " " << right_range.second << "\n";
            #endif
            rgb8 *ptr = reinterpret_cast<rgb8 *>(const_cast<unsigned char *>(img.data.data()));
            robo_velocity curr_velocity = {0.0f, 0.0f};
      
	    //Set the white pixel count to zero	    
	    size_t count_white_pix = 0;   
            for (size_t i = 0;i < image_size; i++, ptr++) {
		//Check the pixel is white pix or not    
                if (*ptr == white_Pix) {
                    const size_t col = i % img.width;
                    if (col >= left_range.first && col <= left_range.second) {
                        curr_velocity = {0.5f, 1.0f};
                    } else if (col >= mid_range.first && col <= mid_range.second) {
                        curr_velocity = {0.5f, 0.0f};
                    } else {
                        curr_velocity = {0.5f, -1.0f};              
                    }
		    //Increment the no of white pixel
                    ++count_white_pix;
                }
            }
	    //Calculate the percentage of white pixel
	    float white_pix_percent = static_cast<float>(count_white_pix);
	    white_pix_percent /= total_pixels;
#if 0
	    std::cout << "White Pix: " << white_pix_percent << "\n";
#endif
	    //If the percentage of white pixel is greater than 10%
	    //stop the velocity of the robot 
	    if (white_pix_percent > 0.10f) {
#if 0		    
		std::cout << "Close to the ball, stop the robot...\n";
#endif
		//Stop the velocity of vehicle.
		curr_velocity = {0.0f, 0.0f};
	    }

            if (last_velocity != curr_velocity) {
                last_velocity = curr_velocity;
                #if 0
                std::cout << "---------------------Vel: " << curr_velocity.first << " " << curr_velocity.second << "\n";
                #endif
                drive_robot(curr_velocity.first, curr_velocity.second);
            }
        }
};


int main(int argc, char **argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    Process_Image pr(n,"/camera/rgb/image_raw", "/ball_chaser/command_robot");
    ros::spin();
    return 0;
}
