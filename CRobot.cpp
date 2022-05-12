#include "CRobot.h"
#include <thread>
//#include <bitset>

CRobot::CRobot()
{
    // Reset flags
    _stop = false;
    _flagServerStarted = false;

    // Generate cv window
    cv::imshow("Program exit", cv::Mat::zeros(cv::Size(100, 100), CV_8UC3));

    // Initialise GPIO
    gpioTerminate();


    if (gpioInitialise() < 0)
    {
        _stop = true;
    }

    // Initialise others
    init_sevSeg();
    init_ultrasonic();
    init_threads();
}

CRobot::~CRobot()
{
    gpioWrite(robotOps::seg_dig1, robotOps::OFF);
    gpioWrite(robotOps::seg_dig2, robotOps::OFF);
    gpioWrite(robotOps::seg_dig3, robotOps::OFF);
    gpioWrite(robotOps::seg_dig4, robotOps::OFF);
    gpioTerminate();
}

void CRobot::init_sevSeg()
{
    _sevSegDigitSelector = 0;
    _sevSegChar = '_';
    sevSegMessage("INIT");

    // Digits
    _sevSegMap.insert(std::pair<char,int>('0', 0b0000001));
    _sevSegMap.insert(std::pair<char,int>('1', 0b1001111));
    _sevSegMap.insert(std::pair<char,int>('2', 0b0010010));
    _sevSegMap.insert(std::pair<char,int>('3', 0b0000110));
    _sevSegMap.insert(std::pair<char,int>('4', 0b1001100));
    _sevSegMap.insert(std::pair<char,int>('5', 0b0100100));
    _sevSegMap.insert(std::pair<char,int>('6', 0b0100000));
    _sevSegMap.insert(std::pair<char,int>('7', 0b0001111));
    _sevSegMap.insert(std::pair<char,int>('8', 0b0000000));
    _sevSegMap.insert(std::pair<char,int>('9', 0b0000100));

    // Letters
    _sevSegMap.insert(std::pair<char,int>('A', 0b0001000));
    _sevSegMap.insert(std::pair<char,int>('B', 0b1100000));
    _sevSegMap.insert(std::pair<char,int>('C', 0b1110010));
    _sevSegMap.insert(std::pair<char,int>('D', 0b1000010));
    _sevSegMap.insert(std::pair<char,int>('E', 0b0110000));
    _sevSegMap.insert(std::pair<char,int>('F', 0b0111000));
    _sevSegMap.insert(std::pair<char,int>('G', 0b0100001));
    _sevSegMap.insert(std::pair<char,int>('H', 0b1001000));
    _sevSegMap.insert(std::pair<char,int>('I', 0b1001111));
    _sevSegMap.insert(std::pair<char,int>('J', 0b1000011));
    _sevSegMap.insert(std::pair<char,int>('K', 0b1001110));
    _sevSegMap.insert(std::pair<char,int>('L', 0b1110001));
    _sevSegMap.insert(std::pair<char,int>('M', 0b0101011));
    _sevSegMap.insert(std::pair<char,int>('N', 0b1101010));
    _sevSegMap.insert(std::pair<char,int>('O', 0b1100010));
    _sevSegMap.insert(std::pair<char,int>('P', 0b0011000));
    _sevSegMap.insert(std::pair<char,int>('Q', 0b0001100));
    _sevSegMap.insert(std::pair<char,int>('R', 0b1111010));
    _sevSegMap.insert(std::pair<char,int>('S', 0b0100100));
    _sevSegMap.insert(std::pair<char,int>('T', 0b1110000));
    _sevSegMap.insert(std::pair<char,int>('U', 0b1000001));
    _sevSegMap.insert(std::pair<char,int>('V', 0b1100011));
    _sevSegMap.insert(std::pair<char,int>('W', 0b1010101));
    _sevSegMap.insert(std::pair<char,int>('X', 0b0110110));
    _sevSegMap.insert(std::pair<char,int>('Y', 0b1000100));
    _sevSegMap.insert(std::pair<char,int>('Z', 0b0010010));

    // Blank and decimal point
    _sevSegMap.insert(std::pair<char,int>('_', 0b11111111));
    _sevSegMap.insert(std::pair<char,int>('.', 0b01111111));

    // Masks. 'p' is for decimal point
    _sevSegMap.insert(std::pair<char,int>('a', 0b1000000));
    _sevSegMap.insert(std::pair<char,int>('b', 0b0100000));
    _sevSegMap.insert(std::pair<char,int>('c', 0b0010000));
    _sevSegMap.insert(std::pair<char,int>('d', 0b0001000));
    _sevSegMap.insert(std::pair<char,int>('e', 0b0000100));
    _sevSegMap.insert(std::pair<char,int>('f', 0b0000010));
    _sevSegMap.insert(std::pair<char,int>('g', 0b0000001));
    _sevSegMap.insert(std::pair<char,int>('p', 0b10000000));

    sevSegMessage("RDY");
}

void CRobot::init_threads()
{
    _threadVector.push_back((std::thread(&CRobot::thread_sevSegUpdate, this)));
    _threadVector.back().detach();
    _threadVector.push_back((std::thread(&CRobot::thread_ultrasonicUpdate, this)));
    _threadVector.back().detach();
    _threadVector.push_back((std::thread(&CRobot::thread_commServerStart, this)));
    _threadVector.back().detach();
    _threadVector.push_back((std::thread(&CRobot::thread_commServerMain, this)));
    _threadVector.back().detach();
}

void CRobot::init_ultrasonic()
{
        gpioSetMode(robotOps::us_echo, PI_INPUT);
        gpioSetMode(robotOps::us_trig, PI_OUTPUT);

        _usPreviousTime = 0;
        _usCurrentTime = 0;
        _usPeriodTime = 0; // Used to create a 60 ms period
        _usEchoRisingEdge = 0; // Time at which the rising edge on the echo pin occurs
        _usEchoFallingEdge = 0; // Time at which the falling edge on the echo pin occurs
        _usDistance = 0; // In metres
        _usTrigState = 1; // Used to create a 60 ms period
        _usEchoState = 1;
}

void CRobot::thread_sevSegUpdate()
{
    std::cout << "Thread sevSegUpdate started." << std::endl;

    while (!_stop)
	{
		sevSegUpdate();
	}


}

void CRobot::thread_ultrasonicUpdate()
{
    std::cout << "Thread ultrasonicUpdate started." << std::endl;

    while (!_stop)
	{
		ultrasonicUpdate();
	}
}

void CRobot::thread_commServerMain()
{
    std::cout << "Thread commServerMain started." << std::endl;

    while(!_stop)
    {
        //std::this_thread::sleep_for(std::chrono::microseconds(1));
        commServerMain();
    }

    _server.stop();

    std::cout << "Server stopped." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void CRobot::thread_commServerStart()
{
    std::cout << "Thread commServerStart started." << std::endl;
    int port = 4699;
    std::cout << "Starting server on port " << port << "." << std::endl;
    _server.start(port);
}

void CRobot::sevSegMessage(std::string initial)
{
    initial.resize(8);
    _sevSegMutex.lock();
    _sevSegMessage.clear();
    _sevSegMessage.resize(8, '_');
    _sevSegMutex.unlock();

    for (unsigned int index = 0, alnumCount = 0, punctCount = 0; index < initial.size() && alnumCount <= 4 && punctCount <= 4; index++)
    {
        if (isalnum(initial.at(index)) && alnumCount < 4)
        {
            _sevSegMessage.at(alnumCount * 2) = (char)std::toupper(initial.at(index));
            alnumCount++;
        }

        else if (initial.at(index) == '.')
        {
            if (punctCount < alnumCount)
            {
                _sevSegMessage.at((alnumCount * 2 - 1) >= 0 ? alnumCount * 2 - 1 : 1) = '.';
                punctCount++;
            }

            else
            {
                alnumCount++;
                _sevSegMessage.at(alnumCount * 2 - 1) = '.';
                punctCount++;
            }
        }

        else if (alnumCount >= 4 && initial.at(index) != '.')
        {
            break;
        }

        else if (alnumCount < 4 && punctCount < 4)
        {
            _sevSegMessage.at(alnumCount * 2) = '_';
            alnumCount++;
        }
    }
}

char CRobot::sevSegChar(int digit)
{
    _sevSegMutex.lock();
    _sevSegChar = _sevSegMap[_sevSegMessage.at((digit - 1) * 2)] | (_sevSegMap['p'] & _sevSegMap[_sevSegMessage.at((digit - 1) * 2 + 1)]);
    _sevSegMutex.unlock();
    return _sevSegChar;
}

void CRobot::sevSegUpdate()
{
    auto end_time = std::chrono::system_clock::now() + std::chrono::microseconds(1000);
//std::cout << (double) cv::getTickCount() / cv::getTickFrequency() << std::endl;
    //double _CurrentTime = cv::getTickCount();
    //static double _PreviousTime = 0;
    //double difference = (_CurrentTime - _PreviousTime) / cv::getTickFrequency();

//    if (difference >= 0.001)
  //  {
        _sevSegDigitSelector < 4 ? _sevSegDigitSelector++ : _sevSegDigitSelector = 1; // Iterate between digits 1 - 4; return to 1 if greater than 4

        char character = sevSegChar(_sevSegDigitSelector);

        switch (_sevSegDigitSelector)
        {
            case 1:
                gpioWrite(robotOps::seg_dig1, robotOps::ON);
                gpioWrite(robotOps::seg_dig2, robotOps::OFF);
                gpioWrite(robotOps::seg_dig3, robotOps::OFF);
                gpioWrite(robotOps::seg_dig4, robotOps::OFF);
                break;

            case 2:
                gpioWrite(robotOps::seg_dig1, robotOps::OFF);
                gpioWrite(robotOps::seg_dig2, robotOps::ON);
                gpioWrite(robotOps::seg_dig3, robotOps::OFF);
                gpioWrite(robotOps::seg_dig4, robotOps::OFF);
                break;

            case 3:
                gpioWrite(robotOps::seg_dig1, robotOps::OFF);
                gpioWrite(robotOps::seg_dig2, robotOps::OFF);
                gpioWrite(robotOps::seg_dig3, robotOps::ON);
                gpioWrite(robotOps::seg_dig4, robotOps::OFF);
                break;

            case 4:
                gpioWrite(robotOps::seg_dig1, robotOps::OFF);
                gpioWrite(robotOps::seg_dig2, robotOps::OFF);
                gpioWrite(robotOps::seg_dig3, robotOps::OFF);
                gpioWrite(robotOps::seg_dig4, robotOps::ON);
                break;
        }

        // Apply character mask to the current digit and write to display
        gpioWrite(robotOps::seg_a, (character & _sevSegMap['a']) > 0);
        gpioWrite(robotOps::seg_b, (character & _sevSegMap['b']) > 0);
        gpioWrite(robotOps::seg_c, (character & _sevSegMap['c']) > 0);
        gpioWrite(robotOps::seg_d, (character & _sevSegMap['d']) > 0);
        gpioWrite(robotOps::seg_e, (character & _sevSegMap['e']) > 0);
        gpioWrite(robotOps::seg_f, (character & _sevSegMap['f']) > 0);
        gpioWrite(robotOps::seg_g, (character & _sevSegMap['g']) > 0);
        gpioWrite(robotOps::seg_dp, (character & _sevSegMap['p']) > 0);

        //_PreviousTime = _CurrentTime;
    //}

    std::this_thread::sleep_until(end_time);
}

void CRobot::ultrasonicUpdate()
{
    _usCurrentTime = cv::getTickCount();

    if (((_usCurrentTime - _usPreviousTime) / cv::getTickFrequency()) >= 0.0001) // Generate 100 us TRIG pulse
    {
        if (_usTrigState == 1 || _usTrigState == 2) // Check if TRIG pulse is on the rising edge (1) or falling edge (2)
        {
            gpioWrite(robotOps::us_trig, !gpioRead(robotOps::us_trig)); // Invert TRIG state
            _usPreviousTime = _usCurrentTime;

            if (_usTrigState == 1) // If on the rising edge of the TRIG pulse
            {
                _usPeriodTime = _usCurrentTime; // Capture time for period calculation
            }

            _usTrigState++; // Used to generate only a single TRIG pulse during the 60 ms period.
        }
    }

    if (_usEchoState == 1 || _usEchoState == 2)
    {
        if (gpioRead(robotOps::us_echo) == robotOps::HIGH && _usEchoState == 1)
        {
            _usEchoRisingEdge = _usCurrentTime;
            _usEchoState++;
        }

        else if (gpioRead(robotOps::us_echo) == robotOps::LOW && _usEchoState == 2)
        {
            _usEchoFallingEdge = _usCurrentTime;
            _usEchoState++;
        }
    }

    if (((_usCurrentTime - _usPreviousTime) / cv::getTickFrequency()) >= 0.06) // Generates 60 ms period
    {
        _usTrigState = 1; // Reset TRIG state after the 60 ms period
        _usEchoState = 1; // Reset ECHO state after the 60 ms period
        _usDistance = ((_usEchoFallingEdge - _usEchoRisingEdge) / cv::getTickFrequency()) * 340 / 2; // Distance in metres
    }
}

void CRobot::commServerMain()
{
    std::vector<std::string> serverCommand;

    _server.get_cmd(serverCommand);

    if (serverCommand.size() > 0)
    {
        for (unsigned int index = 0; index < serverCommand.size(); index++)
        {
            if (serverCommand.at(index) == "a")
            {
                _server.send_string("Test command received.");
                sevSegMessage("Test");
            }

            else if (serverCommand.at(index) == "Good")
            {
                sevSegMessage("Good");
                _server.send_string("Command acknowledged");
            }

            else if (serverCommand.at(index) == "i")
            {
                sevSegMessage("drv.f");
                _server.send_string("Drive forward");
            }

            else if (serverCommand.at(index) == "k")
            {
                sevSegMessage("drv.r");
                _server.send_string("Reverse");
            }

            else if (serverCommand.at(index) == "j")
            {
                sevSegMessage("trn.l");
                _server.send_string("Turn left");
            }

            else if (serverCommand.at(index) == "l")
            {
                sevSegMessage("trn.r");
                _server.send_string("Turn right");
            }

            else if (serverCommand.at(index) == ",")
            {
                sevSegMessage("Brak");
                _server.send_string("Brake");
            }

            else
            {
                sevSegMessage("UNKN");
                _server.send_string("Unknown command");
            }
        }
    }
}

void CRobot::draw()
{
    /*//_sevSegMessage = "A___";
    sevSegMessage("A");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    //_sevSegMessage = "AB__";
    sevSegMessage("AB");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    //_sevSegMessage = "ABC_";
    sevSegMessage("ABC");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    //_sevSegMessage = "ABCD";
    sevSegMessage("ABCD");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("A.BCD");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("A.B.CD");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("A.B.C.D");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("A.B.C.D.");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("3.141");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("27.31");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("192.1");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sevSegMessage("9576.");
    std::this_thread::sleep_for(std::chrono::seconds(2));*/
}

void CRobot::update()
{
}
