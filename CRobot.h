#ifndef CROBOT_H
#define CROBOT_H

#include "sysSelect.h"
#include "CBase4618.h"
#include "server.h"
#include "Client.h"
#include <map>
#include <vector>
#include <mutex>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "pigpio.h"

#define CANVAS_NAME "piCam"
#define CAM_WIDTH 640
#define CAM_HEIGHT 360
#define AI1 6
#define AI2 5
#define BI1 19
#define BI2 26
#define PWMA 12
#define PWMB 13
#define STBY 9
#define SERVO1 4
#define DUTY_CYCLE_MAX 1000000
#define DUTY_PERCENT 0.3



class CRobot : public CBase4618
{
    public:
        CRobot();
        virtual ~CRobot();

        void draw();
        void update();

    protected:

    private:
        // File
        std::string _filename = "params.txt";
        std::ifstream _inFile;
        std::ofstream _outFile;

        // Seven segment
        std::map<char, int> _sevSegMap;
        std::string _sevSegMessage;
        std::string _sevSegScoreMessage;
        char _sevSegChar;
        int _sevSegDigitSelector;
        std::mutex _sevSegMutex;

        void sevSegUpdate();
        void sevSegMessage(std::string message);
        char sevSegChar(int digit);

        // Camera & ArUco
        void markers();
        int i = 0;
        //float _area;
        std::vector<int> _ids;
        std::vector<float> _area;
        std::vector<std::vector<cv::Point2f> > _corners;

        // Movement Functions
        void goForward();
        void goLeft();
        void goRight();
        void goReverse();
        void stop();
        void drive();

        // Servo
        void fire();

        // Ultrasonic sensor
        double _usPreviousTime;
        double _usCurrentTime;
        double _usPeriodTime; // Used to create a 60 ms period
        double _usEchoRisingEdge; // Time at which the rising edge on the echo pin occurs
        double _usEchoFallingEdge; // Time at which the falling edge on the echo pin occurs
        float _usDistance; // In metres
        int _usTrigState; // Used to create a 60 ms period
        int _usEchoState;
        void ultrasonicUpdate();

        // Server. To communicate with a client computer for remote control.
        Server _server;
        void commServerMain();
        bool _flagServerStarted;

        // Client. To communicate with the arena computer to obtain arena information.
        CClient _client;
        std::string _arenaIP;
        int _arenaPort;
        void commClientStart();
        void commClientMain();

        // Arena
        int _target1;
        int _target2;
        int _target3;
        int _target4;
        void extractArenaInfo(std::string response);

        // Settings
        int _threshholdLow; // trackbar
        int _threshholdHigh; // trackbar
        int _currentState;
        int _pwmFreq;
        float _dutyPercentForward;
        float _dutyPercentLeft;
        float _dutyPercentRight;
        float _dutyPercentReverse;
        float _c1areaMin;
        float _c1areaMax;
        float _c2areaMin;
        int _c2sleep;

        // Threads
        std::vector<std::thread> _threadVector;
        void thread_sevSegUpdate();
        void thread_ultrasonicUpdate();
        void thread_commServerStart();
        void thread_commServerMain();
        void thread_commClientStart();
        void thread_commClientMain();

        // Init
        void init_sevSeg();
        void init_gpio();
        void init_threads();
        void init_ultrasonic();
        void init_file();
};

namespace robotOps
{
    enum sevSeg
    {
        seg_dig1 = 23, seg_dig2 = 18, seg_dig3 = 15, seg_dig4 = 14,
        seg_a = 16, seg_b = 11, seg_c = 20, seg_d = 22, seg_e = 27, seg_f = 7, seg_g = 8, seg_dp = 25
    };

    enum ultrasound
    {
        us_trig = 2, us_echo = 3
    };

    enum digitalState
    {
        ON = 1, OFF = 0,
        HIGH = 1, LOW = 0
    };
}

#endif // CROBOT_H
