#include "sysSelect.h"

//#include "CControl.h"
#include "CBase4618.h"
#include <thread>

void CBase4618::updateThreaded()
{
	while (!_waitKeyFlag)
	{
		update();
	}
}

void CBase4618::drawThreaded()
{
	while (!_waitKeyFlag)
	{
 		draw();
	}
}

CBase4618::CBase4618()
{
	_waitKeyFlag = false;
	_stop = false;
}

CBase4618::~CBase4618()
{
}

//void CBase4618::run()
//{
//	if (!_stop)
//	{
//		CBase4618::_waitKeyFlag = false;
//
//		// Open threads for update() and draw()
//		CBase4618::_waitKeyVector.push_back(std::thread(&CBase4618::updateThreaded, this));
//		CBase4618::_waitKeyVector.push_back(std::thread(&CBase4618::drawThreaded, this));
//
//		while (!_waitKeyFlag)
//		{
//			char waitKeyReceived = cv::waitKey(1);
//			if (waitKeyReceived == 'q' || waitKeyReceived == 'Q' || _stop == true)
//			{
//				_waitKeyFlag = true;
//			}
//		}
//
//		// Close threads
//		if (!CBase4618::_waitKeyVector.empty()) // Don't try to clear an empty vector
//		{
//			// Ensure update() and draw() threads are terminated
//			CBase4618::_waitKeyVector.front().join();
//			CBase4618::_waitKeyVector.back().join();
//			CBase4618::_waitKeyVector.clear(); // Remove all elements in this vector
//			CBase4618::_waitKeyFlag = true;
//		}
//
//		cv::destroyWindow(_canvasTitle);
//	}
//
//	else
//	{
//		std::cout << "Error: Could not run!" << std::endl;
//	}
//}

void CBase4618::run()
{
	if (!_stop)
	{
		CBase4618::_waitKeyFlag = false;

		// Open threads for update() and draw()
		//CBase4618::_waitKeyVector.push_back(std::thread(&CBase4618::updateThreaded, this));
		//CBase4618::_waitKeyVector.push_back(std::thread(&CBase4618::drawThreaded, this));

		while (!_waitKeyFlag)
		{
			_waitKeyReceived = cv::waitKey(10);
			draw();
			update();
			if (_waitKeyReceived == 'q' || _waitKeyReceived == 'Q' || _stop == true)
			{
				_waitKeyFlag = true;
				_stop = true;
			}
		}

		// Close threads
//		if (!CBase4618::_waitKeyVector.empty()) // Don't try to clear an empty vector
//		{	/*if(_ids.size() > 0)

//			// Ensure update() and draw() threads are terminated
//			CBase4618::_waitKeyVector.front().join();
//			CBase4618::_waitKeyVector.back().join();
//			CBase4618::_waitKeyVector.clear(); // Remove all elements in this vector
//			CBase4618::_waitKeyFlag = true;
//		}

		//cv::destroyWindow(_canvasTitle);
		cv::destroyAllWindows();
	}

	else
	{
		std::cout << "Error: Could not run!" << std::endl;
	}
}
