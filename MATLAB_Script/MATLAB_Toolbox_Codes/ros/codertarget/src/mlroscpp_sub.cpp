// Copyright 2021 The MathWorks, Inc.
#include <cstring>
#include "mlroscpp_sub.h"

/**
* Function to get status text.
*/
void getStatusText(bool status, char* mlStatusText) {
    if (status) {
        const char* cStatusText_ = "success";
        size_t mlStatusTextLen = strlen(cStatusText_);
        std::strncpy(mlStatusText, cStatusText_, mlStatusTextLen);
        mlStatusText[mlStatusTextLen] = 0;

    } 
    else {
        const char* cStatusText_ = "timeout";
        size_t mlStatusTextLen = strlen(cStatusText_);
        std::strncpy(mlStatusText, cStatusText_, mlStatusTextLen);
        mlStatusText[mlStatusTextLen] = 0;
    }
}

