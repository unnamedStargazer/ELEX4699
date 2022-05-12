#ifndef CROBOT_H
#define CROBOT_H

#include "sysSelect.h"
#include "CBase4618.h"
#include "server.h"
#include <map>
#include <vector>
#include <mutex>


class CRobot : public CBase4618
{
    public:
        CRobot();
        virtual ~CRobot();

        void draw();
        void update();

    protected:

    private:
        // Seven segment
        std::map<char, int> _sevSegMap;
        std::string _sevSegMessage;
        char _sevSegChar;
        int _sevSegDigitSelector;
        std::mutex _sevSegMutex;

            void sevSegUpdate();
            void sevSegMessage(std::string message);
            char sevSegChar(int digit);

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
        void commClient();

        // Threads
        std::vector<std::thread> _threadVector;
        void thread_sevSegUpdate();
        void thread_ultrasonicUpdate();
        void thread_commServerStart();
        void thread_commServerMain();

        // Init
        void init_sevSeg();
        void init_gpio();
        void init_threads();
        void init_ultrasonic();
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
