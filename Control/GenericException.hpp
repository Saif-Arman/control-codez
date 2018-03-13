//======================================================================== 
// Package	 : putils - Multi-platform utilities library
// Authors	 : Vilas Kumar Chitrakaran (vilas@ieee.org)
// Version       : 2.0 (Apr 2005)
// Compatibility : POSIX, GCC, MSVC 2005
// File          : GenericException.hpp
// Description   : General purpose run time exception handling
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

#ifndef _GENERICEXCEPTION_HPP_INCLUDED
#define _GENERICEXCEPTION_HPP_INCLUDED

#ifdef PUTILS_DLL
 #ifdef PUTILS_DLL_EXPORT
  #define PUTILS_DLL_API __declspec(dllexport)
 #else
  #define PUTILS_DLL_API __declspec(dllimport)
 #endif
#else
 #define PUTILS_DLL_API
#endif

#include <string>

//==============================================================================
// class GenericException
//------------------------------------------------------------------------------
// \brief
// A basic exception object.
//
// <b>Example Program:</b>
// \include GenericException.t.cpp
//==============================================================================

class PUTILS_DLL_API GenericException
{
 public:
  inline GenericException();
   // Default constructor sets error to 0 (no error)
  
  inline GenericException(int error, const char *desc=NULL);
   // This constructor allows initialization
   //  error  set integer error code. (0 reserved for no error)
   //  desc   set a short description [less than
   //         80 chars], possibly just the object that
   //         set the error.
   
  inline GenericException(const GenericException &e);
   // Copy constructor
   
  inline GenericException &operator=(const GenericException &e);
   // Assignment operation 
  
  ~GenericException(){};
   // Destructor does nothing
   
  inline void setError(int error, const char *desc=NULL);
   // Set an error
   //  error  set integer error code. (0 reserved for no error)
   //  desc   set a short description [less than
   //         80 chars], possibly just the object that
   //         set the error.
   
  inline int getErrorCode() const;
   //  return  latest error code. (0 means no error).
   
  inline const char *getErrorDesc() const;
   //  return  any descriptive message that was set with the 
   //          error.  

  //======== END OF INTERFACE ========

 private:
  int d_errno;     // error number
  char d_desc[80]; // description
};


//==============================================================================
GenericException::GenericException()
//==============================================================================
{
 d_errno = 0;
 d_desc[0]='\0';
}


GenericException::GenericException(int error, const char *desc)
{
 setError(error, desc);
}


GenericException::GenericException(const GenericException &e)
{
 setError(e.getErrorCode(), e.getErrorDesc());
}


//==============================================================================
GenericException &GenericException::operator=(const GenericException &e) 
//==============================================================================
{
 setError(e.getErrorCode(), e.getErrorDesc());
 return *this;
}


//==============================================================================
void GenericException::setError(int error, const char *desc)
//==============================================================================
{
 d_errno = error;
 if(desc)
  strncpy(d_desc, desc, 79);
 d_desc[79] = '\0';
}


//==============================================================================
int GenericException::getErrorCode() const
//==============================================================================
{
 return d_errno;
}


//==============================================================================
const char *GenericException::getErrorDesc() const
//==============================================================================
{
 return d_desc;
}

#endif //_GENERICEXCEPTION_HPP_INCLUDED
