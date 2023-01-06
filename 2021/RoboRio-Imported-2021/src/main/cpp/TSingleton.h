//==============================================================================
// TSingleton.h
//==============================================================================

#ifndef TSingleton_H
#define TSingleton_H

#include "stddef.h"
#include "frc/Utility.h"



// Template class for implementing the singleton pattern
//
// To use this class derive a class from it, supplying the class as the type parameter
//
// class MySingleClass : public TSingleton<MySingleClass> {
//
template <class T>
class TSingleton {
public:
    // Get the singleton instance
    static T& GetInstance() {
        wpi_assert(ms_instance != NULL);
        return *ms_instance;
    }

    // Get whether the singleton exists yet.
    static bool HasInstance() {
        return (ms_instance != NULL);
    }

protected:
    // Constructor
    //
    // instance Instance of object of type T, passed in from derived class.
    TSingleton(T* instance) {
        wpi_assert(ms_instance == NULL);
        ms_instance = instance;
    }

    // Destructor
    virtual ~TSingleton() {
        wpi_assert(ms_instance != NULL);
        ms_instance = NULL;
    }

private:
    //! Pointer to the singleton instance
    static T* ms_instance;
};

//! Instantiation of the single instance
template <class T> T* TSingleton<T>::ms_instance = NULL;

#endif  // TSingleton_H
