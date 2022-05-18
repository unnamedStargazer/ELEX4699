////////////////////////////////////////////////////////////////
// ELEX 4618 Template project for BCIT
// Created Oct 5, 2016 by Craig Hennessey
// Last updated Dec 6, 2021
////////////////////////////////////////////////////////////////

#include "sysSelect.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <limits>
#include <sstream>
#include <unistd.h>

// Must include Windows.h after Winsock2.h, so Serial must be included after Client/Server
#include "CBase4618.h"
#include "CRobot.h"

//#define CANVAS_NAME "Display Image"

//enum type
//{
//    DIGITAL = 0,
//    ANALOG,
//    SERVO
//};
//
//enum channel
//{
//    joystickX = 2, joystickOK = 5, accX = 23, accY, accZ, joystickY, button2 = 32, button1, led_blu = 37, led_green, led_red
//};

//void process_msg()
//{
//  MSG msg;
//  while (::PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
//  {
//    ::TranslateMessage(&msg);
//    ::DispatchMessage(&msg);
//  }
//}

//void waitForInput(bool &receivedInput)
//{
//    std::cin.seekg(std::cin.beg); // Set position to beginning to avoid reading newline character from previous inputs
//    std::cin.ignore(INT_MAX, '\n');
//
//    // Wait for Enter key to be pressed
//    receivedInput = true;
//}
//
//////////////////////////////////////////////////////////////////
//// Serial Communication
//////////////////////////////////////////////////////////////////
//void test_com()
//{
//  // Comport class (change port to your MSP device port)
//  Serial com;
//  com.open("COM4");
//
//  // TX and RX strings
//  std::string tx_str = "G " + std::to_string(type::ANALOG) + " " + std::to_string(channel::joystickX) + "\n";
//  std::string rx_str;
//
//  // Begin another thread to wait for the Enter key to be pressed
//  bool receivedInput = false;
//  std::thread wfi(waitForInput, std::ref(receivedInput)); // std::ref needed to pass by reference to thread
//
//  // temporary storage
//  char buff[2];
//  do
//  {
//    com.exchangeData(buff, tx_str, rx_str);
//
//    if (receivedInput == true)
//    {
//        // Ensure wait for input thread is terminated
//        wfi.join();
//        break;
//    }
//
//
//    printf ("\nRX: %s", rx_str.c_str());
//
//    cv::waitKey(1);
//  }
//  while (true);
//}
//
//////////////////////////////////////////////////////////////////
//// Display Image on screen
//////////////////////////////////////////////////////////////////
//void do_image()
//{
//  cv::Mat im;
//
//  im = cv::imread("BCIT.jpg");
//
//  srand(time(0));
//
//  for (int i = 0; i < 500; i++)
//  {
//    float radius = 50 * rand() / RAND_MAX;
//    cv::Point center = cv::Point(im.size().width*rand() / RAND_MAX, im.size().height*rand() / RAND_MAX);
//
//    cv::circle(im, center, radius, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
//
//    im.at<char>(i,i) = 255;
//
//    cv::imshow(CANVAS_NAME, im);
//    cv::waitKey(1);
//  }
//}
//
//////////////////////////////////////////////////////////////////
//// Display Video on screen
//////////////////////////////////////////////////////////////////
//void do_video()
//{
//  cv::VideoCapture vid;
//
//  vid.open(0);
//
//  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//  bool do_canny = true;
//  bool do_aruco = false;
//  int canny_thresh = 30;
//  cvui::init(CANVAS_NAME);
//  cv::Point set_pt = cv::Point(10, 50);
//  std::vector<cv::Scalar> color_vec;
//  color_vec.push_back(cv::Scalar(255, 255, 255));
//  color_vec.push_back(cv::Scalar(255, 0, 0));
//  color_vec.push_back(cv::Scalar(0, 255, 0));
//  color_vec.push_back(cv::Scalar(0, 0, 255));
//  int color_index = 0;
//  if (vid.isOpened() == TRUE)
//  {
//    do
//    {
//      cv::Mat frame, edges;
//      vid >> frame;
//
//      if (frame.empty() == false)
//      {
//        if (do_aruco == true)
//        {
//          std::vector<int> ids;
//          std::vector<std::vector<cv::Point2f> > corners;
//          cv::aruco::detectMarkers(frame, dictionary, corners, ids);
//          if (ids.size() > 0)
//          {
//            cv::aruco::drawDetectedMarkers(frame, corners, ids);
//          }
//        }
//
//        if (do_canny == true)
//        {
//          cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
//          cv::GaussianBlur(edges, edges, cv::Size(7, 7), 1.5, 1.5);
//          cv::Canny(edges, edges, 0, canny_thresh, 3);
//          cv::add(frame, color_vec.at(color_index), frame, edges);
//        }
//
//        cvui::window(frame, set_pt.x, set_pt.y, 200, 190, "Settings");
//        cvui::checkbox(frame, set_pt.x + 5, set_pt.y + 25, "Canny Filter", &do_canny);
//        cvui::checkbox(frame, set_pt.x + 5, set_pt.y + 50, "ArUco", &do_aruco);
//        cvui::text(frame, set_pt.x + 5, set_pt.y + 75, "Canny Threshold");
//        cvui::trackbar(frame, set_pt.x + 5, set_pt.y + 90, 180, &canny_thresh, 5, 120);
//        if (cvui::button(frame, set_pt.x + 5, set_pt.y + 140, 100, 30, "Colour Switch"))
//        {
//          color_index++;
//          if (color_index >= color_vec.size()) { color_index = 0; }
//        }
//
//        cvui::update();
//        cv::imshow(CANVAS_NAME, frame);
//      }
//    }
//    while (cv::waitKey(10) != ' ');
//  }
//}
//
//
//////////////////////////////////////////////////////////////////
//// Demo client server communication
//////////////////////////////////////////////////////////////////
//bool serverthreadexit = false;
//Server serv;
//
//// Send image to TCP server
//void serverimagethread()
//{
//  cv::VideoCapture vid;
//
//  vid.open(0);
//
//  if (vid.isOpened() == true)
//  {
//    do
//    {
//      cv::Mat frame;
//      vid >> frame;
//      if (frame.empty() == false)
//      {
//        imshow("Server Image", frame);
//        process_msg();
//        serv.set_txim(frame);
//      }
//    }
//    while (serverthreadexit == false);
//  }
//}
//
//void serverthread()
//{
//  // Start server
//  serv.start(4618);
//}
//
//void server()
//{
//  char inputchar;
//  std::vector<std::string> cmds;
//
//  // Start image send to server thread
//  std::thread t1(&serverimagethread);
//  t1.detach();
//
//  // Start server thread
//  std::thread t2(&serverthread);
//  t2.detach();
//
//  cv::namedWindow("WindowForWaitkey");
//  do
//  {
//    inputchar = cv::waitKey(100);
//    if (inputchar == 'q')
//    {
//      serverthreadexit = true;
//    }
//
//    serv.get_cmd(cmds);
//
//    if (cmds.size() > 0)
//    {
//      for (int i = 0; i < cmds.size(); i++)
//      {
//        if (cmds.at(i) == "a")
//        {
//          std::cout << "\nReceived 'a' command";
//
//          // Send an 'a' message
//          std::string reply = "Hi there from Server";
//          serv.send_string(reply);
//        }
//        else
//        {
//          std::string reply = "Got some other message";
//          serv.send_string(reply);
//        }
//      }
//    }
//  } while (serverthreadexit == false);
//
//  serv.stop();
//
//  Sleep(100);
//}

void print_menu()
{
	std::cout << "\n***********************************";
	std::cout << "\n* ELEX4618 Template Project";
	std::cout << "\n***********************************";
	std::cout << "\n(1) Robot Car";
	std::cout << "\n(0) Exit";
	std::cout << "\n(2) piCam";
	std::cout << "\nCMD> ";
}

void robotCar()
{
    CRobot robot;
    robot.run();
}

/*void testgpio()
{
    gpioTerminate();

    if (gpioInitialise() < 0)
    {
        //_stop = true;
    }

    else
    {
//        gpioSetMode(ECHO, PI_INPUT);
//        gpioSetMode(TRIG, PI_OUTPUT);

        static double previousTime = 0;
        double currentTime = 0;
        double periodTime = 0; // Used to create a 60 ms period
        double echoRisingEdge = 0;
        double echoFallingEdge = 0;
        float distance = 0;
        int trigState = 1; // Used to create a 60 ms period
        int echoState = 1;

        while (1)
        {
            currentTime = cv::getTickCount();

            if (((currentTime - previousTime) / cv::getTickFrequency()) >= 0.0001) // Generate 100 us TRIG pulse
            {
                if (trigState == 1 || trigState == 2) // Check if TRIG pulse is on the rising edge (1) or falling edge (2)
                {
                    gpioWrite(TRIG, !gpioRead(TRIG)); // Invert TRIG state
                    previousTime = currentTime;

                    if (trigState == 1) // If on the rising edge of the TRIG pulse
                    {
                        periodTime = currentTime; // Capture time for period calculation
                    }

                    trigState++; // Used to generate only a single TRIG pulse during the 60 ms period.
                }
            }

            if (echoState == 1 || echoState == 2)
            {
                if (gpioRead(ECHO) == robotOps::HIGH && echoState == 1)
                {
                    echoRisingEdge = currentTime;
                    echoState++;
                }

                else if (gpioRead(ECHO) == robotOps::LOW && echoState == 2)
                {
                    echoFallingEdge = currentTime;
                    echoState++;
                }
            }

            if (((currentTime - previousTime) / cv::getTickFrequency()) >= 0.06) // Generates 60 ms period
            {
                trigState = 1; // Reset TRIG state after the 60 ms period
                echoState = 1; // Reset ECHO state after the 60 ms period
                distance = ((echoFallingEdge - echoRisingEdge) / cv::getTickFrequency()) * 340 / 2; // Distance in metres
                std::cout << distance << std::endl;
            }
        }
    }
}*/


void motorTest()
{

    int input = -1;

    if (gpioInitialise() < 0)
    {
        input = 0;
    }

    gpioSetMode(4, 1);

    while (input != 0)
    {
        std::cout << "> ";
        std::cin >> input;
        gpioServo(4, input);
    }

    gpioTerminate();
}

int main(int argc, char* argv[])
{
	std::string cmd;

	do
	{
        //std::cin.clear();
		//std::cout.flush();
		print_menu();
		//std::cin.clear();
		//std::cin.clear();
		//cmd.clear();
		std::cin >> cmd;
		//std::getline(std::cin, cmd);
		//std::cin.sync();

		/*if (std::cin.fail())
		{
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }*/

		switch (cmd.front())
		{
		case '1': robotCar();
		break;
        case '0':
        break;
        case '2':
            motorTest();
            break;
        default:
            std::cout << "Invalid entry, try again." << std::endl;
            cmd = "error";
            break;
		}
	} while (cmd.front() != '0'); //&& cmd != "error"
}
