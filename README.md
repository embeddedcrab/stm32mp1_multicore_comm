# stm32mp1


## This repository contains project done on STM32Mp157C-DK2 Kit, which utilizes both ARM Cortex A7 and ARM Cortex M4 processors.

M4 part generates some data and send it to A7.
A7 has a server running which accepts messages and copy those into a shared memory region which will be read by some other application.
Other Application takes data from shared memory and is responsible to process that data according to message request.

_________________________________________________________________________________________________________________
|		________							________			|
|		|Core 1|							|Core 2|			|
|		========							========			|
|	     				   Shared Memory Region							|
|	--------------------------	|			|	    ------------------------		|
|	|	IPC Sender	 |=====>|			|==========>|	   IPC Reader	   |		|
|	--------------------------	|			|	    ------------------------		|
|		    ||			|			|						|
|	--------------------------	|			|	    -------------------------		|
|	| Active Request Handler |	|			|	    | Thread Pool Processing |		|
|	--------------------------	|			|	    -------------------------		|
|		    ||			|			|						|
|	|------------------------	|			|						|
|	|			|	|			|						|
|	|	SERVER		|	|			|						|
|	-------------------------	|			|						|
|		  ||  ^  ||		|			|						|
|		  ||  |  ||											|	
|		  ||  |	 ||											|
|														|
|-----------------ttyRPMSG0-------------------------------------------------------------------------------------|
|		 //  ||  \\											|
|		///  ||  \\\											|
|		|||  ||	 |||											|
|		|||  ||	 |||<--- Sending some Message in particular format					|
|		|||  ||	 |||											|
|		|||  ||	  --------------------									|
|		||| ========================== M4 Processor							|
|		------------------------------									|
|														|
|														|
|				Fig. Sample Processing inside system						|
|														|
|														|
-----------------------------------------------------------------------------------------------------------------


Folders Description:

1. Utilities (C++)
Utility Library containing following functionalties:
	a. PMR (Polymorphic Resource Allocator) which can be used with C++14. Only Monotonic Buffer resource is added but others can be added easily.
	b. POSIX Shared Memory Adaptor layer.
	c. POSIX Semaphore Adaptor layer.
	d. Thread Pool: can be used to process parallel requests of same type or different.
	e. Generic Signal Handler Interface: used to handle signals in system and can also call System Reset Handler.
	f. Generic Reset Handler Interface: used to handle system level failures.

This library can be used as a generic library in any project where some system programming using C++ is needed. Particularly for Automotive where C++14 is being used.
It can easily be extended for further enhancements.


2. HtServer (C++)
Small Server Application to receive data from M4 Core. It process the requests and put the data into shared memory region.
It has 3 major sections:
	a. Initialization of Application
	b. Core Server to receive data
	c. IPC channel using shared memory

It receives data on ttyRPMSG0 channel. Whenever a request comes it is placed into a queue using active thread which transfers data to shared memory using request queue in a separate thread.


3. HtClient (C++)
Application to read data from shared memory and process each request.
It also has 3 major sections:
	a. Initialization of Application
	b. Core Receiver
	c. IPC channel

it receives data from shared memory and put the request into a separate thread. Just priting the received message as dummy processing.


4. M4A7Comm (C)
It is OpenAMP-FreeRTOS based sample application whcih sends some data to A7 core using ttyRPMSG0. There is a possibility to receive data from A7 also using ttyRPMSG0.

It has majorly 3 Threads:
	a. LED Task to blink led or some regular system related tasks
	b. OpenAMP Receive Task: to check the data on virtual UARTs
	c. Send Data Thread: sends data in particular format to ttyRPMSG0 after a fixed interval. Actual ADC data or other different data can also be sent accoridng the system behavior.

[It also receives data from A7 and sends it back to A7 (whatever was sent from there) on ttyRPMSG0, can easily be found official website as a demo part.]



How to Build:

yocto_steps.txt file can be found to build A7 (Linux Application) using yocto project.

Simple cmake can be used to build CM4 based projects.
	a. Create build folder in project directory
	b. Go to build directory. cmake .. and then make
It should generate following files after successful compilation:
linker.map, M4A7CommApp.bin, M4A7CommApp.elf, M4A7CommApp.hex





Cheers!!
Happy to Help and Share :-)

