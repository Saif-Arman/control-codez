//======================================================================== 
// Package		: The Math Library - Ex
// Authors		: Vilas Kumar Chitrakaran
// Start Date		: Wed Dec 20 11:08:28 GMT 2000
// Compiler		: GNU C++ 2.95.3 and above
// ----------------------------------------------------------------------  
// File: ColumnVector.hpp
// Interface of the class ColumnVector.
//========================================================================  


#ifndef INCLUDED_ColumnVector_hpp
#define INCLUDED_ColumnVector_hpp

#ifdef QMATH_DLL
#ifdef QMATH_DLL_EXPORT
#define QMATHDLL_API __declspec(dllexport)
#else
#define QMATHDLL_API __declspec(dllimport)
#endif
#else
#define QMATHDLL_API
#endif

#include "Matrix.hpp"
#include "VectorBase.hpp"


//====================================================================
// class ColumnVector
// -------------------------------------------------------------------
// \brief
// A class for column vectors.
//
// The class \c ColumnVector is derived from the base classes \c Matrix
// and \c VectorBase, and provides methods for operations such as 
// cross product, dot product and element-by-element multiplication.
//
// <b>Example Program:</b>
// \include Vector.t.cpp
//========================================================================  

template<int size, class T> class Vector;

template<int size, class T = double>
class QMATHDLL_API ColumnVector : public Matrix<size, 1, T>, public VectorBase<T>
{
public:
	inline ColumnVector() : Matrix<size, 1, T>(), VectorBase<T>() {}
	// The default constructor. The elements are not initialized.

	inline ColumnVector(const ColumnVector<size, T> &v);
	// Copy Constructor. 

	inline ColumnVector(const Matrix<size,1,T> &m);
	// The conversion constructor for conversion
	// of a \c Matrix type of single column into
	// type \c ColumnVector.

	inline ColumnVector(const Vector<size,T> &v);
	// The conversion constructor for conversion
	// of a \c Vector type into \c ColumnVector.

	virtual ~ColumnVector(){}
	// The default destructor.

	virtual T *getElementsPointer() const { return (T *)(this->d_element); }
	//  return A pointer to the first element in the vector.

	inline virtual T getElement(int index) const;
	//  return  The value at position specified by index 
	//          (index = 1 is the first element).

	inline virtual void setElement(int index, T value);
	// Sets an element to a value at the specified position.
	//  index  Position of the desired element.
	//  value  The desired element is set to this value.

	virtual bool isRowVector() const {return false;}
	//  return  false

	virtual int getNumElements() const {return size;}
	//  return The number of elements in the vector.

	inline T operator()(int index) const;
	inline T &operator()(int index);
	// Access or assign the element at the position specified by 
	// index. For example: 
	// \code 
	// myVector(2)=12.65; 
	// \endcode

	inline ColumnVector<size, T> &operator=(const VectorBase<T> &v);
	// Assign a \c VectorBase type to a \c ColumnVector type. Both objects 
	// must have the same dimensions.

	MatrixInitializer<T> operator=(const T &value);
	// Initialize a vector object.
	//  value  The value to which all elements in the vector are initialized.
	// The initialization of the vector object can also
	// be done as a comma seperated list. For example:
	// \code 
	// ColumnVector<3> myVector;
	// myVector = 67.88, 45.89, 90; 
	// \endcode
private:
};


template<class T>
QMATHDLL_API ColumnVector<3,T> crossProduct(const ColumnVector<3,T> &v1, const ColumnVector<3,T> &v2);
// Generates the cross product of two 3 dimensional column vectors. 
//  v1, v2  The 3D column-vector arguments.
//  return  The cross product.

template<int size, class T>
QMATHDLL_API ColumnVector<size, T> elementProduct(const ColumnVector<size,T> &v1, const ColumnVector<size,T> &v2);
// This function performs multiplication between two column vectors 
// element-by-element.
//  v1, v2  The column-vector arguments
//  return  The product.

template<int size, class T>
QMATHDLL_API T dotProduct(const ColumnVector<size,T> &v1, const ColumnVector<size,T> &v2);
// Dot (inner) product between two column-vectors.
//  v1, v2 The column-vector arguments.
//  return  The scalar product.

template<int size, class T>
inline QMATHDLL_API RowVector<size, T> transpose(const ColumnVector<size, T> &vector);
//  return  The transpose of type \c RowVector.

// ========== END OF INTERFACE ==========



//==============================================================
// Definition of inlined functions.
//==============================================================

//======================================================================== 
// ColumnVector::ColumnVector: Constructor of the ColumnVector class
//======================================================================== 
template<int size, class T>
ColumnVector<size, T>::ColumnVector(const Matrix<size,1,T> &m)
	: Matrix<size,1,T>(m), VectorBase<T>()
{
}

template<int size, class T>
ColumnVector<size, T>::ColumnVector(const ColumnVector<size, T> &v)
	: Matrix<size,1,T>(v), VectorBase<T>()
{
}

template<int size, class T>
ColumnVector<size, T>::ColumnVector(const Vector<size, T> &v)
	: Matrix<size,1,T>(v), VectorBase<T>()
{
}


//======================================================================== 
// ColumnVector::getElement
//========================================================================  
template<int size, class T>
T ColumnVector<size, T>::getElement(int i) const
{
	if( i > size || i < 1)
	{
		static MathException exception;
		exception.setErrorType(QMathException_illegalIndex);
		throw exception;
	}
	return this->d_element[(i-1)];
}


//======================================================================== 
// ColumnVector::setElement
//========================================================================  
template<int size, class T>
void ColumnVector<size, T>::setElement(int i, T value)
{
	if( i > size || i < 1)
	{
		static MathException exception;
		exception.setErrorType(QMathException_illegalIndex);
		throw exception;
	}
	this->d_element[(i-1)] = value;
}


//======================================================================== 
// ColumnVector::operator()
//========================================================================  
template<int size, class T>
T ColumnVector<size, T>::operator()(int i) const
{
	if( i > size || i < 1)
	{
		static MathException exception;
		exception.setErrorType(QMathException_illegalIndex);
		throw exception;
	}
	return this->d_element[(i-1)];
}

template<int size, class T>
T &ColumnVector<size, T>::operator()(int i)
{
	if( i > size || i < 1)
	{
		static MathException exception;
		exception.setErrorType(QMathException_illegalIndex);
		throw exception;
	}
	return this->d_element[(i-1)];
}


//======================================================================== 
// crossProduct
//========================================================================  
template<class T>
ColumnVector<3,T> crossProduct(const ColumnVector<3,T> &v1, 
	const ColumnVector<3,T> &v2)
{
	ColumnVector<3, T> cp;
	cp(1) = v1(2)*v2(3) - v1(3)*v2(2);
	cp(2) = v1(3)*v2(1) - v1(1)*v2(3);
	cp(3) = v1(1)*v2(2) - v1(2)*v2(1);
	return (cp);
}


//======================================================================== 
// elementProduct
//========================================================================  
template<int size, class T>
ColumnVector<size, T> elementProduct(const ColumnVector<size,T> &v1, 
	const ColumnVector<size,T> &v2)
{
	ColumnVector<size, T> ep;
	for (int i = 1; i <= size; ++i)
		ep(i) = v1(i)*v2(i);
	return (ep);
}


//======================================================================== 
// dotProduct
//========================================================================  
template<int size, class T>
T dotProduct(const ColumnVector<size,T> &v1, const ColumnVector<size,T> &v2)
{ 
	T d = 0;
	for (int i = 1; i <= size; ++i)
		d += v1(i)*v2(i);
	return (d);
}


//======================================================================== 
// Transpose
//========================================================================  
template<int size, class T>
RowVector<size, T> transpose(const ColumnVector<size, T> &v)
{
	RowVector<size,T> row;
	for (int i = 1; i <= size; ++i)
		row(i) = v(i);
	return row;
}


//======================================================================== 
// ColumnVector::operator=
//========================================================================  
template<int size, class T>
ColumnVector<size, T> &ColumnVector<size, T>::operator=(const VectorBase<T> &v)
{
	if(this != &v)
		this->VectorBase<T>::operator=(v);
	return *this;
}


template<int size, class T>
MatrixInitializer<T> ColumnVector<size, T>::operator=(const T &value)
{
	for (int i = 0; i < size; ++i)
	{
		this->d_element[i] = value;
	}
	MatrixInitializer<T>  matrixInitialize(size, 1, this->d_element);
	return matrixInitialize;
}


#endif