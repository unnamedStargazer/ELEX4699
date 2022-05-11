#pragma once

#include <thread>
#include <vector>
//#include "CControl.h"

/*! @mainpage
*
* The following sections indicate what classes and files were used for each lab.
*
* @section lab03 Lab 03: Embedded Control
* @subsection lab03classes Classes
*
* - CBase4618
* - CControl
*
* @subsection lab03otherclasses Other classes
*
* - Serial
*
* @subsection lab03otherfiles Other files
*
* - 4618_Template.cpp
* - cvui.h
*
*
* @section lab04 Lab 04: Etch-A-Sketch
* @subsection lab04classes Classes
*
* - CBase4618
* - CControl
* - CSketch
*
* @subsection lab04otherclasses Other classes
*
* - Serial
*
* @subsection lab04otherfiles Other files
*
* - 4618_Template.cpp
* - cvui.h
*
*
* @section lab05 Lab 05: Pong
* @subsection lab05classes Classes
*
* - CBase4618
* - CControl
* - CPong
* - CShape
* - CPaddle
* - CBall
* - CText
*
* @subsection lab05otherclasses Other classes
*
* - Serial
*
* @subsection lab05otherfiles Other files
*
* - 4618_Template.cpp
* - cvui.h
*
*
* @section lab06 Lab 06: Asteroids
* @subsection lab06classes Classes
*
* - CBase4618
* - CControl
* - CAsteroidGame
* - CGameObject
* - CShip
* - CAsteroidObject
* - CMissile
*
* @subsection lab06otherclasses Other classes
*
* - Serial
*
* @subsection lab06otherfiles Other files
*
* - 4618_Template.cpp
* - cvui.h
*
*/

/**
* @class CBase4618
* @brief The abstract base class that provides basic input/output and display functionality.
*
*/

class CBase4618
{
private:
	/**
	* @brief A flag that is set when the user presses the 'q' key; stops the run() loop.
	*/
	bool _waitKeyFlag;

	/**
	* @brief A vector used to start a new thread that waits for the 'q' key to be pressed.
	*/
	std::vector<std::thread> _waitKeyVector;

	/**
	* @brief Polls the device for updates from the analog and digital interfaces.
	*
	* This member function is called by update() in a new `CBase4618::_waitKeyVector` vector element.
	* The thread is closed when the exit key (ex. 'q') is pressed.
	*/
	void updateThreaded();

	/**
	* @brief Draws objects on and refreshes the canvas to produce frames.
	*
	* This member function is called by draw() in a new `CBase4618::_waitKeyVector` vector element.
	* The thread is closed when the exit key (ex. 'q') is pressed.
	*/
	void drawThreaded();

protected:
	/** @brief The control object that provides the capability to communicate with the MSP432.
	*
	*/
//	CControl _control;

	/** @brief The OpenCV matrix object that will be used.
	*/
	cv::Mat _canvas;

	/**
	* @brief The desired title of the OpenCV window.
	*/
	std::string _canvasTitle;

	/**
	* @brief The desired size in pixels of the OpenCV window.
	*/
	cv::Size _canvasSize;

	/**
	* @brief A flag that can be set by derived classes to halt execution.
	*/
	bool _stop;

	char _waitKeyReceived;

public:
	/** @brief The default constructor that instantiates a CBase4618 object.
	*
	*/
	CBase4618();

	/** @brief The default destructor that destorys a CBase4618 object.
	*
	*/
	~CBase4618();

	/** @brief Virtual function that gets the necessary metrics from the MSP432.
	*
	* This member function creates a new thread in a new `CBase4618::_waitKeyVector` element and calls on updateThreaded().
	*/
	virtual void update() = 0;

	/** @brief Virtual function that draws to the OpenCV window.
	*
	* This member function creates a new thread in a `CBase4618::_waitKeyVector` element and calls on drawThreaded().
	*/
	virtual void draw() = 0;

	/** @brief Loops through update() and draw() as long as the 'q' button is not pressed.
	*/
	void run();
};

