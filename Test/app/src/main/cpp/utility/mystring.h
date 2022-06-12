//
// Created by LK on 2022/4/6.
//

#ifndef ORB_SLAM3_ANDROID_MYSTRING_H
#define ORB_SLAM3_ANDROID_MYSTRING_H
#include <string>
#include <sstream>
#include <iomanip>
using namespace std;

namespace std
{
    template < typename T >
    std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << fixed << setprecision(16) << n ;//这里的setprecision(16)是控制浮点数精度用的
        return stm.str() ;
    }
}


#endif //ORB_SLAM3_ANDROID_MYSTRING_H
