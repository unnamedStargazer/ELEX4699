#include "CRobot.h"
#include <thread>
#include <string>
#include <vector>

CRobot::CRobot()
{
    _useManual = false;
    init_main();
}

CRobot::CRobot(bool manual)
{
    _useManual = true;
    init_main();
}

void CRobot::init_main()
{
	// Reset flags
	_stop = false;
	_flagServerStarted = false;

	// Arena
	_arenaIP = "192.168.1.100";
	_arenaPort = 47001;
	_target1 = 0;
	_target2 = 0;
	_target3 = 0;
	_target4 = 0;
	_qrIndex = 0;

	// Generate cv window
	cv::imshow("Program exit", cv::Mat::zeros(cv::Size(100, 100), CV_8UC3));
	cv::imshow("Settings", cv::Mat::zeros(cv::Size(400, 400), CV_8UC3));
	_frame = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);

	// Initialise GPIO
	gpioTerminate();


	if (gpioInitialise() < 0)
	{
		_stop = true;
	}


	// Initialise others
	init_file();
	init_gpio();
	init_sevSeg();
	//init_ultrasonic();
	init_threads(); // Seven-segment, ultrasonic, server start, server main, client
}

CRobot::~CRobot()
{
	_threadVector.clear();
	_client.close_socket();

	gpioWrite(robotOps::seg_dig1, robotOps::OFF);
	gpioWrite(robotOps::seg_dig2, robotOps::OFF);
	gpioWrite(robotOps::seg_dig3, robotOps::OFF);
	gpioWrite(robotOps::seg_dig4, robotOps::OFF);
	stop();

	gpioTerminate();
}

void CRobot::init_gpio()
{
	gpioSetMode(AI1, PI_OUTPUT);
	gpioSetMode(AI2, PI_OUTPUT);
	gpioSetMode(BI1, PI_OUTPUT);
	gpioSetMode(BI1, PI_OUTPUT);
	gpioSetMode(STBY, PI_OUTPUT);
	gpioSetMode(PWMA, PI_OUTPUT);
	gpioSetMode(PWMB, PI_OUTPUT);
	gpioSetMode(SERVO1, PI_OUTPUT);


	// Enable Standby
	gpioWrite(STBY, 1);

	gpioWrite(AI1, robotOps::LOW);
	gpioWrite(AI2, robotOps::LOW);
	gpioWrite(BI1, robotOps::LOW);
	gpioWrite(BI2, robotOps::LOW);

}
void CRobot::init_sevSeg()
{
    _sevSegPreviousTime = 0;
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
	//_threadVector.push_back((std::thread(&CRobot::thread_ultrasonicUpdate, this)));
	//_threadVector.back().detach();
	//std::thread threadSevSeg (&CRobot::thread_sevSegUpdate, this);
	//threadSevSeg.detach();

	if (_useManual)
	{
        _threadVector.push_back((std::thread(&CRobot::thread_commServerStart, this)));
        _threadVector.back().detach();
        _threadVector.push_back((std::thread(&CRobot::thread_commServerMain, this)));
        _threadVector.back().detach();
        _threadVector.push_back((std::thread(&CRobot::thread_commClientStart, this)));
        _threadVector.back().detach();
        _threadVector.push_back((std::thread(&CRobot::thread_commClientMain, this)));
        _threadVector.back().detach();
	}
}

void CRobot::init_ultrasonic()
{
/*
		gpioSetMode(robotOps::us_echo, PI_INPUT);
		gpioSetMode(robotOps::us_trig, PI_OUTPUT);

		_usPreviousTime = 0;
		_usCurrentTime = 0;
		_usPeriodTime = 0; // Used to create a 60 ms period
		_usEchoRisingEdge = 0; // Time at which the rising edge on the echo pin occurs
		_usEchoFallingEdge = 0; // Time at which the falling edge on the echo pin occurs
		_usDistance = 0; // In metres
		_usTrigState = 1; // Used to create a 60 ms period
		_usEchoState = 1;*/
}

void CRobot::init_file()
{
	_filename = "parameters.txt";
	std::string variableName;
	_inFile.open(_filename);

	_inFile >> variableName >> _threshholdLow;
	std::cout << variableName << " " << _threshholdLow << std::endl;

	_inFile >> variableName >> _threshholdHigh;
	std::cout << variableName << " " << _threshholdHigh << std::endl;

	_inFile >> variableName >> _sevSegUpdateTime;
	std::cout << variableName << " " << _sevSegUpdateTime << std::endl;

	_inFile >> variableName >> _currentState;
	std::cout << variableName << " " << _currentState << std::endl;

	_inFile >> variableName >> _pwmFreq;
	std::cout << variableName << " " << _pwmFreq << std::endl;

	_inFile >> variableName >> _dutyPercentForward;
	std::cout << variableName << " " << _dutyPercentForward << std::endl;

	_inFile >> variableName >> _dutyPercentLeft;
	std::cout << variableName << " " << _dutyPercentLeft << std::endl;

	_inFile >> variableName >> _dutyPercentRight;
	std::cout << variableName << " " << _dutyPercentRight << std::endl;

	_inFile >> variableName >> _dutyPercentReverse;
	std::cout << variableName << " " << _dutyPercentReverse << std::endl;

	_inFile >> variableName >> _c1areaMax;
	std::cout << variableName << " " << _c1areaMax << std::endl;

	_inFile >> variableName >> _c2sleep;
	std::cout << variableName << " " << _c2sleep << std::endl;

	_inFile >> variableName >> _c3sleep;
	std::cout << variableName << " " << _c3sleep << std::endl;

	_inFile >> variableName >> _c4areaMax;
	std::cout << variableName << " " << _c4areaMax << std::endl;

	_inFile >> variableName >> _c5sleep;
	std::cout << variableName << " " << _c5sleep << std::endl;

	_inFile >> variableName >> _c6areaMax;
	std::cout << variableName << " " << _c6areaMax << std::endl;

	_inFile >> variableName >> _c7sleep;
	std::cout << variableName << " " << _c7sleep << std::endl;

	_inFile >> variableName >> _c8sleep;
	std::cout << variableName << " " << _c8sleep << std::endl;

	_inFile >> variableName >> _c9areaMax;
	std::cout << variableName << " " << _c9areaMax << std::endl;

	_inFile >> variableName >> _c10sleep;
	std::cout << variableName << " " << _c10sleep << std::endl;

	_inFile >> variableName >> _c11areaMax;
	std::cout << variableName << " " << _c11areaMax << std::endl;

	_inFile >> variableName >> _c12sleep;
	std::cout << variableName << " " << _c12sleep << std::endl;

	_inFile >> variableName >> _c13sleep;
	std::cout << variableName << " " << _c13sleep << std::endl;

	_inFile >> variableName >> _c14sleep1;
	std::cout << variableName << " " << _c14sleep1 << std::endl;

	_inFile >> variableName >> _c14sleep2;
	std::cout << variableName << " " << _c14sleep2 << std::endl;

	_inFile >> variableName >> _c15sleep1;
	std::cout << variableName << " " << _c15sleep1 << std::endl;

	_inFile >> variableName >> _c15sleep2;
	std::cout << variableName << " " << _c15sleep2 << std::endl;

	_inFile >> variableName >> _c16sleep1;
	std::cout << variableName << " " << _c16sleep1 << std::endl;

	_inFile >> variableName >> _c16sleep2;
	std::cout << variableName << " " << _c16sleep2 << std::endl;

	_inFile >> variableName >> _c17areaMax;
	std::cout << variableName << " " << _c17areaMax << std::endl;

	_inFile >> variableName >> _c18sleep;
	std::cout << variableName << " " << _c18sleep << std::endl;

	_inFile >> variableName >> _c19sleep;
	std::cout << variableName << " " << _c19sleep << std::endl;

	_inFile >> variableName >> _c20sleep;
	std::cout << variableName << " " << _c20sleep << std::endl;

	_inFile >> variableName >> _c21sleep;
	std::cout << variableName << " " << _c21sleep << std::endl;
}

void CRobot::thread_sevSegUpdate()
{
	std::cout << "Thread sevSegUpdate started." << std::endl;

	while (!_stop)
	{
		sevSegUpdate();
	}
}

/*void CRobot::thread_sevSegUpdate(CRobot* ptrToSelf)
{
	std::cout << "Thread sevSegUpdate started." << std::endl;

	while (!ptrToSelf->_stop)
	{
		ptrToSelf->sevSegUpdate();
	}
}*/

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

void CRobot::thread_commClientStart()
{
	std::cout << "Thread commClient started." << std::endl;

	_client.connect_socket(_arenaIP, _arenaPort);
}

void CRobot::thread_commClientMain()
{
	while (!_stop)
	{
		commClientMain();
	}
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
	//auto end_time = std::chrono::system_clock::now() + std::chrono::microseconds(1000);

	_sevSegCurrentTime = cv::getTickCount();

    if (((_sevSegCurrentTime - _sevSegPreviousTime) / cv::getTickFrequency()) >= _sevSegUpdateTime)
    {
        //_sevSegDigitSelector = 1;
        _sevSegDigitSelector < 4 ? _sevSegDigitSelector++ : _sevSegDigitSelector = 1; // Iterate between digits 1 - 4; return to 1 if greater than 4

        _sevSegPreviousTime = _sevSegCurrentTime;

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

        //std::this_thread::sleep_until(end_time);
	}
}

void CRobot::ultrasonicUpdate()
{
	/*_usCurrentTime = cv::getTickCount();

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
	}*/


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
				goForward();
			}

			else if (serverCommand.at(index) == ",")
			{
				sevSegMessage("drv.r");
				_server.send_string("Reverse");
				goReverse();
			}

			else if (serverCommand.at(index) == "j")
			{
				sevSegMessage("trn.l");
				_server.send_string("Turn left");
				goLeft();
			}

			else if (serverCommand.at(index) == "l")
			{
				sevSegMessage("trn.r");
				_server.send_string("Turn right");
				goRight();
			}

			else if (serverCommand.at(index) == "k")
			{
				sevSegMessage("Brak");
				_server.send_string("Brake");
				stop();
			}

			else if (serverCommand.at(index) == ";")
			{
				sevSegMessage("load");
				_server.send_string("Load");
				load();
			}

			else if (serverCommand.at(index) == "'")
			{
				sevSegMessage("Fire");
				_server.send_string("Fire");
				fire();
			}

			else
			{
				sevSegMessage("UNKN");
				_server.send_string("Unknown command");
			}
		}
	}
}

void CRobot::commClientMain()
{
	std::string response;
	std::string command = "G 0";

	_client.tx_str(command);

	if (_client.rx_str(response) == true)
	{
		std::cout << "Transmitted: " << command << "\t Received: " << response << std::endl;
	}

	extractArenaInfo(response);

	/*else // Uncommenting this portion breaks the communication for some reason
	{
		// No response, disconnect and reconnect
		_client.close_socket();
		_client.connect_socket(_arenaIP, _arenaPort);
	}*/
}

void CRobot::extractArenaInfo(std::string response)
{
	if (response.length() > 0)
	{
		int start1, start2, start3, start4;

		start1 = response.find(',') + 1;
		start2 = response.find(',', start1) + 1;
		start3 = response.find(',', start2) + 1;
		start4 = response.find(',', start3) + 1;

		_target1 = std::stoi(response.substr(start1, start2 - start1 - 1));
		_target2 = std::stoi(response.substr(start2, start3 - start2 - 1));
		_target3 = std::stoi(response.substr(start3, start4 - start3 - 1));
		_target4 = std::stoi(response.substr(start4, response.find_last_of(',') - start4));

		std::string target1, target2, target3, target4;

		target1 = std::to_string(_target1).at(0);
		target2 = std::to_string(_target2).at(0);
		target3 = std::to_string(_target3).at(0);
		target4 = std::to_string(_target4).at(0);

		std::string _sevSegScoreMessage = target1 + "." + target2 + "." + target3 + "." + target4;
		//sevSegMessage(_sevSegScoreMessage);
	}
}

void CRobot::markers()
{
    _vid.open(cv::CAP_V4L2);
	_vid.set(cv::CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
	_vid.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	if (_vid.isOpened() == true)
	{
		do
		{
            _vid >> _frame;
            //usleep(1000000);

			if (_frame.empty() == false)
			{
				cv::aruco::detectMarkers(_frame, dictionary, _corners, _ids);
				if (_ids.size() > 0)
                {
                    for (_qrIndex = 0; _qrIndex < _ids.size(); _qrIndex++)
                    {
                        cv::aruco::drawDetectedMarkers(_frame, _corners, _ids);
                        _area.push_back(std::abs((((_corners[_qrIndex][0].x*_corners[_qrIndex][1].y) - (_corners[_qrIndex][0].y*_corners[_qrIndex][1].x)) + ((_corners[_qrIndex][1].x*_corners[_qrIndex][2].y) - (_corners[_qrIndex][1].y*_corners[_qrIndex][2].x)) + ((_corners[_qrIndex][2].x*_corners[_qrIndex][3].y) - (_corners[_qrIndex][2].y*_corners[_qrIndex][3].x)) + ((_corners[_qrIndex][3].x*_corners[_qrIndex][0].y) - (_corners[_qrIndex][3].y*_corners[_qrIndex][0].x))) / 2));
                        //sevSegMessage(std::to_string(_area[_qrIndex]));
                        //std::cout << "updating area... " << _area[_qrIndex] << std::endl;
                        _xCentre.push_back((((_corners[_qrIndex][0].x + _corners[_qrIndex][3].x) / 2) + ((_corners[_qrIndex][1].x + _corners[_qrIndex][2].x) / 2)) / 2);
                        //int xcentre = (((_corners[_qrIndex][0].x + _corners[_qrIndex][3].x) / 2) + ((_corners[_qrIndex][1].x + _corners[_qrIndex][2].x) / 2)) / 2;
                    }
                }
			}
			//sevSegUpdate();
			cv::flip(_frame, _frame, -1);
            //\processColour();
			drive();
			_area.clear();
			_xCentre.clear();
			//fire();

			//cv::imshow("LED", _colourFrame);
			//cv::imshow(CANVAS_NAME, _frame);

		}
		while (cv::waitKey(10) != 'q'); // while _stop?
	}
}

void CRobot::processColour()
{
    static int hlow = 60;
    static int slow = 105;
    static int vlow = 50;
    static int hhi = 255;
    static int shi = 255;
    static int lhi = 255;
    cv::createTrackbar("H Low", "Settings", &hlow, 255);
	cv::createTrackbar("S Low", "Settings", &slow, 255);
	cv::createTrackbar("V Low", "Settings", &vlow, 255);
	cv::createTrackbar("H High", "Settings", &hhi, 255);
	cv::createTrackbar("S High", "Settings", &shi, 255);
	cv::createTrackbar("V High", "Settings", &lhi, 255);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(_frame, _colourFrame, cv::COLOR_BGR2HSV);
    cv::inRange(_colourFrame, cv::Scalar(hlow, slow, vlow), cv::Scalar(hhi, shi, lhi), _colourFrame);
    cv::erode(_colourFrame, _colourFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(_colourFrame, _colourFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(_colourFrame, _colourFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(_colourFrame, _colourFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::findContours(_colourFrame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (unsigned int index = 0; index < contours.size(); index++)
    {
        _pcbArea = cv::boundingRect(contours.at(index));
        //std::cout << _pcbArea.area() << std::endl;

    }
}

int CRobot::checkCentre(float& multiplier)
{
    if (_xCentre[_qrIndex] >= (CAM_WIDTH / 2 - 50) && _xCentre[_qrIndex] <= (CAM_WIDTH / 2 + 50))
    {    std::cout << _xCentre[_qrIndex] << " " << "Centred" << std::endl;
        multiplier = 1.0;
        return 1;
    }

    else if (_xCentre[_qrIndex] > CAM_WIDTH / 2)
    {
        multiplier = ((float)_xCentre[_qrIndex] - (CAM_WIDTH / 2)) / (CAM_WIDTH / 2);
        std::cout << _xCentre[_qrIndex] << " " << "Veer left" << " " << multiplier << std::endl;
        return 0;
    }

    else if (_xCentre[_qrIndex] < CAM_WIDTH / 2)
    {
        multiplier = ((float)_xCentre[_qrIndex]) / (CAM_WIDTH / 2);
        std::cout << _xCentre[_qrIndex] << " " << "Veer right" << " " << multiplier << std::endl;
        return 2;
    }
}

void CRobot::goForward()
{
	//std::cout << "going forward\n";
	gpioWrite(AI1, 1);
	gpioWrite(AI2, 0);
	gpioWrite(BI1, 0);
	gpioWrite(BI2, 1);

	gpioHardwarePWM(PWMA, _pwmFreq, _dutyPercentForward * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, _dutyPercentForward * DUTY_CYCLE_MAX);
}

void CRobot::goLeft()
{
	std::cout << "going left" << std::endl;
	gpioWrite(AI1, 1);
	gpioWrite(AI2, 0);
	gpioWrite(BI1, 1);
	gpioWrite(BI2, 0);
	gpioHardwarePWM(PWMA, _pwmFreq, _dutyPercentLeft * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, _dutyPercentRight * DUTY_CYCLE_MAX);
}

void CRobot::goRight()
{
	std::cout << "going right" << std::endl;
	gpioWrite(AI1, 0);
	gpioWrite(AI2, 1);
	gpioWrite(BI1, 0);
	gpioWrite(BI2, 1);
	gpioHardwarePWM(PWMA, _pwmFreq, _dutyPercentLeft * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, _dutyPercentRight * DUTY_CYCLE_MAX);
}

void CRobot::veerLeft(float multiplier)
{
    std::cout << "Veering left\n" << std::endl;
    gpioWrite(AI1, 1);
	gpioWrite(AI2, 0);
	gpioWrite(BI1, 0);
	gpioWrite(BI2, 1);
	gpioHardwarePWM(PWMA, _pwmFreq, _dutyPercentForward * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, (0.25 * multiplier + 0.75) * (_dutyPercentForward*DUTY_CYCLE_MAX));
}

void CRobot::veerRight(float multiplier)
{
    std::cout << "Veering right\n" << std::endl;
    gpioWrite(AI1, 1);
	gpioWrite(AI2, 0);
	gpioWrite(BI1, 0);
	gpioWrite(BI2, 1);
	gpioHardwarePWM(PWMA, _pwmFreq, (0.25 * (1 - multiplier) + 0.75) * (DUTY_CYCLE_MAX*_dutyPercentForward));
	gpioHardwarePWM(PWMB, _pwmFreq, _dutyPercentForward * DUTY_CYCLE_MAX);
}

void CRobot::goReverse()
{
	gpioWrite(AI1, 0);
	gpioWrite(AI2, 1);
	gpioWrite(BI1, 1);
	gpioWrite(BI2, 0);
	gpioHardwarePWM(PWMA, _pwmFreq, _dutyPercentReverse * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, _dutyPercentReverse * DUTY_CYCLE_MAX);
}

void CRobot::stop()
{
	//std::cout << "stopped" << std::endl;
	gpioWrite(AI1, 0);
	gpioWrite(AI2, 0);
	gpioWrite(BI1, 0);
	gpioWrite(BI2, 0);
	gpioHardwarePWM(PWMA, _pwmFreq, 0 * DUTY_CYCLE_MAX);
	gpioHardwarePWM(PWMB, _pwmFreq, 0 * DUTY_CYCLE_MAX);
}

void CRobot::drive()
{
    float multiplier;

	if(_ids.size() > 0)
	{
		for (_qrIndex = 0; _qrIndex < _ids.size(); _qrIndex++)
		{
			switch (_currentState)
			{
				case 1: // drive towards 30
					// if QR 30 area is between .. and ..
					sevSegMessage("c1");
					if (_ids[_qrIndex] == 30)
					{
                        int isCentred = checkCentre(multiplier);

                        if (isCentred == 1) // 1 = centred, 0 = veering left, 2 = veering right
                        {
                            if (_area[_qrIndex] >= 0 && _area[_qrIndex] <= _c1areaMax)
                            {
                                goForward();
                            }
                        }

                        else if (isCentred == 0)
                        {
                            veerLeft(multiplier);
                        }

                        else if (isCentred == 2)
                        {
                            veerRight(multiplier);
                        }
                        //if (_ids[_qrIndex] == 30 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c1areaMax)


                        // break condition: if QR 30 is greater than ..
                        if (_area[_qrIndex] >= _c1areaMax)
                        {
                            //std::cout << "target area reached!\n";
                            stop();
                            _currentState++;
                        }
					}
					break;

				case 2: // turn slightly right towards 26
					// if QR 30 is greater than .. rotate right
					sevSegMessage("c2");
					load();
					goLeft();
					processColour();
					if (_pcbArea.area() >= 100)
					{
                        stop();
                        usleep(_c2sleep);
                        _currentState = 18;
					}
                    //fire();
                    usleep(_c2sleep);
					//stop();
					//_currentState++;
					/*
					if (_ids[_qrIndex] == 30 && _area[_qrIndex] >= _c2areaMin)
					{
						//stop();
						goRight();
						usleep(_c2sleep);
						stop();
					}

					// break condition: if it sees QR 26 and it's between .. and ..
					if (_ids[_qrIndex] == 26 && _area[_qrIndex] >= _c3areaMin && _area[_qrIndex] <= _c3areaMax)
					{
                        stop();
						_currentState++;
					}*/
					break;

				case 3: // drive towards 26
                    sevSegMessage("c3");
                    goRight();
					usleep(_c3sleep);
					stop();
					_currentState++;
//					if (_ids[_qrIndex] == 26 && _area[_qrIndex] >= _c3areaMin && _area[_qrIndex] <= _c3areaMax)
//					{
//                        //std::cout << "go forward" << std::endl;
//						goForward();
//					}
//
//					// break condition: if area of 26 is large enough
//					if (_ids[_qrIndex] == 26 && _area[_qrIndex] >= _c3areaMax)
//					{
//                        stop();
//                        _currentState++;
//					}
					break;

				case 4: // rotate left towards 21
                    sevSegMessage("c4");
                    if (_ids[_qrIndex] == 30 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c4areaMax)
					{
                        //std::cout << "trying to get to target area...\n";
						goForward();
					}

					// break condition: if QR 30 is greater than ..
					if (_ids[_qrIndex] == 30 && _area[_qrIndex] >= _c4areaMax)
					{
                        //std::cout << "target area reached!\n";
                        //stop();
                        _currentState++;
					}

					//if QR 26 is greater than .. rotate right
//					if (_ids[_qrIndex] == 26 && _area[_qrIndex] >= _c3areaMax)
//					{
//                        std::cout << "rotate" << std::endl;
//						//stop();
//						goLeft();
//						usleep(_c4sleep);
//						stop();
//					}
//
//					// break condition: if it sees QR 21 and it's between .. and ..
//					if (_ids[_qrIndex] == 21 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= 8000)
//					{
//
//                        stop();
//                        _currentState++;
//					}
					break;

				case 5: // shoot 21
                    sevSegMessage("c5");
                    goRight();
					usleep(_c5sleep);
					stop();
					_currentState++;
				// go forward towards 21
//				if (_ids[_qrIndex] == 21 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= 8500)
//				{
//                    goForward();
//				}
//				if(_ids[_qrIndex] == 21 && _area[_qrIndex] >= 8500)
//				{
//                    stop();
//                    usleep(100000);
//				}
					break;

				case 6: // rotate right towards 22
                    sevSegMessage("c6");
					if (_ids[_qrIndex] == 34 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c6areaMax)
					{
                        //std::cout << "trying to get to target area...\n";
						goForward();
					}

					// break condition: if QR 30 is greater than ..
					if (_ids[_qrIndex] == 34 && _area[_qrIndex] >= _c6areaMax)
					{
                        //std::cout << "target area reached!\n";
                        //stop();
						_currentState++;
					}
					break;

				case 7: // drive towards 22
                    sevSegMessage("c7");
                    load();
					goLeft();
					usleep(_c7sleep);
					stop();
					//_currentState++;
					_currentState = 19;
					break;

				case 8: // rotate left towards 22
                    sevSegMessage("c8");
					goRight();
					usleep(_c8sleep);
					stop();
					_currentState++;
					break;

				case 9: // rotate right towards 27
                    sevSegMessage("c9");
					if (_ids[_qrIndex] == 34 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c9areaMax)
					{
                        //std::cout << "trying to get to target area...\n";
						goForward();
					}

					// break condition: if QR 30 is greater than ..
					if (_ids[_qrIndex] == 34 && _area[_qrIndex] >= _c9areaMax)
					{
                        //std::cout << "target area reached!\n";
                        //stop();
						_currentState++;
					}
					break;

				case 10: // drive towards 27
                    sevSegMessage("c10");
					goRight();
					usleep(_c10sleep);
					stop();
					_currentState++;
					break;

				case 11: // rotate left towards 27
                    sevSegMessage("c11");
					if (_ids[_qrIndex] == 29 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c11areaMax)
					{
                        //std::cout << "trying to get to target area...\n";
						goForward();
					}

					// break condition: if QR 30 is greater than ..
					if (_ids[_qrIndex] == 29 && _area[_qrIndex] >= _c11areaMax)
					{
                        //std::cout << "target area reached!\n";
                        //stop();
						_currentState++;
					}
					break;

				case 12: // shoot 27
                    sevSegMessage("c12");
                    load();
					goLeft();
					usleep(_c12sleep);
					stop();
					//_currentState++;
					_currentState = 20;
					break;

				case 13: // rotate right towards 23
                    sevSegMessage("c13");
					goRight();
					usleep(_c13sleep);
					stop();
					_currentState++;
					break;

				case 14: // drive towards 23
                    sevSegMessage("c14");
                    goForward();
                    usleep(_c14sleep1);
                    stop();
                    usleep(_c14sleep2);
                    //_currentState++;
                    _currentState = 21;
				/*
					if (_ids[_qrIndex] == 23 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= 1000)
					{
                        //std::cout << "trying to get to target area...\n";
						goForward();
					}

					// break condition: if QR 30 is greater than ..
					if (_ids[_qrIndex] == 23 && _area[_qrIndex] >= 1000)
					{
                        //std::cout << "target area reached!\n";
                        stop();
						_currentState++;
					}*/
					break;

				case 15: // rotate left towards 23
                    sevSegMessage("c15");
					goReverse();
					usleep(_c15sleep1);
					stop();
					usleep(_c15sleep2);
					_currentState++;
					break;

				case 16: // rotate towards 27
                    sevSegMessage("c16");
					// code
					goLeft();
					usleep(_c16sleep1);
					stop();
					usleep(_c16sleep2);
					_currentState++;
					break;

				case 17: // drive towards 27
                    sevSegMessage("c17");
					// code
					if (_ids[_qrIndex] && _area[_qrIndex] >= 0 && _area[_qrIndex] <= _c17areaMax)
					{
                        goForward();
					}
					if (_ids[_qrIndex] && _area[_qrIndex] >= _c17areaMax)
					{
                        stop();
					}
					break;

				case 18: // drive towards 27
                    sevSegMessage("f1");
					stop();
					usleep(_c18sleep);
					fire();
					usleep(_c18sleep);
					_currentState = 3;
					break;

				case 19: // rotate right towards 29
                    sevSegMessage("f2");
					stop();
					usleep(_c19sleep);
					fire();
					usleep(_c19sleep);
					_currentState = 8;
					break;

				case 20: // drive towards 29
                    sevSegMessage("f3");
					stop();
					usleep(_c20sleep);
					fire();
					usleep(_c20sleep);
					_currentState = 13;
					break;

                case 21:
                    sevSegMessage("f4");
                    stop();
					usleep(_c21sleep);
					fire();
					usleep(_c21sleep);
					_currentState = 15;
                    break;
			}

            //sevSegMessage(std::to_string(_currentState));
			std::cout << "Current State: " << _currentState << " " << "ID: " << " " << _ids[_qrIndex] << " " << "Area: " << _area[_qrIndex] << std::endl;

		}
	}

	else
	{
        sevSegMessage("look");
		std::cout << "Looking around... Current state: " << _currentState << " " << std::endl;

//		switch (_currentState)
//		{
//            case 1:
//                usleep(1000000);
//                goForward();
//                break;
//
//            case 4: // rotate left
//                //if (_ids[0] == 26 && _area[0] >= _c3areaMax){}
//                //std::cout << "stop\n";
//                //stop();
//                //gpioDelay(500000);
//                std::cout << "else\n";
//                goLeft();
//                gpioDelay(250000);
//                stop();
//                gpioDelay(250000);
//                //stop();
//                if(_ids[_qrIndex] == 21 && _area[_qrIndex] >= 0 && _area[_qrIndex] <= 8500)
//                {
//                    stop();
//                }
//                break;

//		}
	}
}

void CRobot::fire()
{
    gpioServo(SERVO1, 500); // Fire = 500
}

void CRobot::load()
{
    gpioServo(SERVO1, 2500); // Fire = 500
}


void CRobot::draw()
{

}

void CRobot::update()
{
//for (int i = 0; i < 10; i++)
//    {
//        sevSegMessage(std::to_string(i));
//        sevSegUpdate();
//        usleep(1000000);
//    }
//    for ( int i = 0; i < 10; i++)
//    {
//        std::cout << "load\n";
//        load();
//        usleep(1000000);
//        std::cout << "fire\n";
//        fire();
//        usleep(1000000);
//    }
    if (!_useManual)
    {
        markers();
    }
}
