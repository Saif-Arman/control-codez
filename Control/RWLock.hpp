//======================================================================== 
// Package	 : putils - Multi-platform utilities library
// Authors	 : Vilas Kumar Chitrakaran (vilas@ieee.org)
// Version       : 2.0 (Apr 2005)
// Compatibility : POSIX, GCC, MSVC 2005
// File          : RWLock.hpp
// Description   : POSIX reader/writer locks
//
// Copyright (c) 2005 Vilas Chitrakaran
//
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//========================================================================  

#ifndef _RWLOCK_HPP_INCLUDED
#define _RWLOCK_HPP_INCLUDED

#ifdef PUTILS_DLL
 #ifdef PUTILS_DLL_EXPORT
  #define PUTILS_DLL_API __declspec(dllexport)
 #else
  #define PUTILS_DLL_API __declspec(dllimport)
 #endif
#else
 #define PUTILS_DLL_API
#endif

#include <pthread.h>
#include <errno.h>
#include "GenericException.hpp"

//==============================================================================
// class RWLock
//------------------------------------------------------------------------------
// \brief
// The pthread reader-writer lock.
//
// <ul>
// <li>A Reader-Writer lock allows concurrent access to multiple processes for 
//     reading shared data, but restricts writing to shared data only when 
//     no readers are present.
// <li>Conversely, when a writer has access to shared data, 
//     all other writers and readers are blocked until the writer is done.
// <li>This class will throw an exception of type GenericException in case of
//     errors.
// </ul>
//
// <b>Example Program:</b>
// \include RWLock.t.cpp
//==============================================================================

class PUTILS_DLL_API RWLock
{
 public:
  inline RWLock();
   // Constructor initializes the lock.
   
  inline ~RWLock();
   // Destroys the lock.
   
  inline void readLock();
   // Acquire the shared lock for read access. 
   // If the lock is not available, block until it is.
   
  inline int tryReadLock();
   // Try to acquire the shared lock for read access. 
   // If the lock is not available, return immediately.
   //  return  0 on successful acquisition of lock, else -1

  inline void writeLock();
   // Acquire the shared lock for exclusive write access. 
   // If the lock is not available, block until it is.

  inline int tryWriteLock();
   // Try to acquire the shared lock for exclusive write access. 
   // If the lock is not available, return immediately.
   //  return  0 on successful acquisition of lock, else -1

  inline void unlock();
   // Unlock the shared lock. If the calling thread doesn't own
   // the lock, the behavior of this function is undefined.
   
  //======== END OF INTERFACE ========
 private:
  inline void errorCheck(int code);
   // This will throw an exception on any error.

  pthread_rwlock_t d_rwl;
   // The pthread lock
  
};


//==============================================================================
RWLock::RWLock()
//==============================================================================
{
 #ifdef _WIN32 // attribute setting does not seem to work in windows
  d_rwl = PTHREAD_RWLOCK_INITIALIZER;
 #else 
  pthread_rwlockattr_t attr;
  errorCheck( pthread_rwlockattr_init(&attr) );
  errorCheck( pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_PRIVATE) );
  errorCheck( pthread_rwlock_init(&d_rwl, &attr) );
  pthread_rwlockattr_destroy(&attr);
 #endif
}


//==============================================================================
RWLock::~RWLock()
//==============================================================================
{
 pthread_rwlock_destroy(&d_rwl);
}


//==============================================================================
void RWLock::readLock()
//==============================================================================
{
 errorCheck( pthread_rwlock_rdlock(&d_rwl) );
}


//==============================================================================
int RWLock::tryReadLock()
//==============================================================================
{
 int retVal = 0;

 retVal = pthread_rwlock_tryrdlock(&d_rwl);

 if( retVal == 0 )
  return 0;
  
 if( retVal == EAGAIN || retVal == EBUSY ) 
  return -1;
 
 errorCheck(retVal);
 return -1;
}


//==============================================================================
void RWLock::writeLock()
//==============================================================================
{
 errorCheck( pthread_rwlock_wrlock(&d_rwl) );
}


//==============================================================================
int RWLock::tryWriteLock()
//==============================================================================
{
 int retVal = 0;
 retVal = pthread_rwlock_trywrlock(&d_rwl);
 
 if( retVal == 0 )
  return 0;
 
 if( retVal == EAGAIN || retVal == EBUSY ) 
  return -1;
 
 errorCheck(retVal);
 return -1;
}


//==============================================================================
void RWLock::unlock()
//==============================================================================
{
 errorCheck( pthread_rwlock_unlock(&d_rwl) );
}


//==============================================================================
void RWLock::errorCheck(int code)
//==============================================================================
{
 if(code == 0) return;
 throw(GenericException(code), "[RWLock]");
}

#endif // _RWLOCK_HPP_INCLUDED
