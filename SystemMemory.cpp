//////////////////////////////////////////////////////////////////////////////////////////////////////////
// File		: SystemMemory.cpp
// Version	: 1.0.1
// Date		: 2019.08.20
// Writer	: Lee, Seungmin (CDSL)
//////////////////////////////////////////////////////////////////////////////////////////////////////////


//include//
#include "stdafx.h"
#include <map>
#include "SharedMemory.h"
#include "SystemMemory.h"
#include <mutex>



CSystemMemory g_SystemMemory;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class Name	: CSystemMemory::_CSystemMemory
// Summury		: Private Class for CSystemMemory
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class CSystemMemory::_CSystemMemory {

	// Define ////////////////////////////////////////////////////////
public:

protected:

private:
	typedef std::map<std::string, CSyncSharedMemory*> List_t;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Class Name	: _CEasyMutex
	// Summury		: private class in _CSystemMemory. Auto lock and unlock mutex.
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	class _CEasyMutex {

		// Define ////////////////////////////////////////////////////////
	public:

	protected:

	private:


		// Method ////////////////////////////////////////////////////////
	public:
		////////////////////////////////////////////////////////////////////////////////////////////
		// Method	: _CEasyMutex
		// Input	: None
		// Output	: None
		// Summury	: Standard constructor. Auto lock.
		////////////////////////////////////////////////////////////////////////////////////////////
		_CEasyMutex(std::mutex *poMutex) :
			_poMutex(poMutex) {
			_poMutex->lock();
		}



		////////////////////////////////////////////////////////////////////////////////////////////
		// Method	: ~_CEasyMutex
		// Input	: None
		// Output	: None
		// Summury	: Standard destructor. Auto unlock.
		////////////////////////////////////////////////////////////////////////////////////////////
		~_CEasyMutex() {
			_poMutex->unlock();
		}


	protected:

	private:


		// Member ////////////////////////////////////////////////////////
	public:

	protected:

	private:
		std::mutex *_poMutex;
	};

	// Method ////////////////////////////////////////////////////////
public:

	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: _CSystemMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard constructor
	////////////////////////////////////////////////////////////////////////////////////////////
	_CSystemMemory() {

	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: ~_CSystemMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard destructor
	////////////////////////////////////////////////////////////////////////////////////////////
	~_CSystemMemory() {

		_CEasyMutex mutex(&_mutex);

		for (List_t::iterator it = _memList.begin(); it != _memList.end(); it++) {
			delete it->second;
			//_memList.erase(it);
		}

		_memList.clear();
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: CreateMemory
	// Input	: Memory block name(std::string), Memory size(size_t)
	// Output	: Result(bool)
	// Summury	: Create shared memory.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool CreateMemory(std::string name, size_t size) {

		_CEasyMutex mutex(&_mutex);
		List_t::iterator memory = _findElement(name);

		if (memory != _memList.end())
			return false;

		std::pair<std::string, CSyncSharedMemory*> element;
		element.first = name;
		element.second = new CSyncSharedMemory;
		element.second->CreateSharedMemory(size);

		_memList.insert(element);

		return true;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: DeleteMemory
	// Input	: Memory block name(std::string)
	// Output	: Result(bool)
	// Summury	: Delete shared memory.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool DeleteMemory(std::string name) {

		_CEasyMutex mutex(&_mutex);
		List_t::iterator memory = _findElement(name);

		if (memory != _memList.end()) {
			delete memory->second;
			_memList.erase(memory);
			return true;
		}

		return false;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: GetMemory
	// Input	: Memory block name(std::string)
	// Output	: Memory block pointer(CSyncSharedMemory*)
	// Summury	: Return memory block pointer.
	////////////////////////////////////////////////////////////////////////////////////////////
	CSyncSharedMemory* GetMemory(std::string name) {

		_CEasyMutex mutex(&_mutex);
		List_t::iterator memory = _findElement(name);

		if (memory != _memList.end())
			return memory->second;

		return 0;
	}

protected:

private:

	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: _findElement
	// Input	: Memory block name(std::string)
	// Output	: iterator(List_t::iterator)
	// Summury	: Find and return iterator.
	////////////////////////////////////////////////////////////////////////////////////////////
	List_t::iterator _findElement(std::string key) {

		return _memList.find(key);
	}

	// Member ////////////////////////////////////////////////////////
public:

protected:

private:
	std::map<std::string, CSyncSharedMemory*> _memList;
	std::mutex _mutex;

};



CSystemMemory::CSystemMemory()
{
	_poSystemMemory = new _CSystemMemory;
}



CSystemMemory::~CSystemMemory()
{
	delete _poSystemMemory;
}



bool CSystemMemory::CreateMemory(std::string name, size_t size) {

	return _poSystemMemory->CreateMemory(name, size);
}



bool CSystemMemory::DeleteMemory(std::string name) {

	return _poSystemMemory->DeleteMemory(name);
}



CSyncSharedMemory* CSystemMemory::GetMemory(std::string name) {

	return _poSystemMemory->GetMemory(name);
}