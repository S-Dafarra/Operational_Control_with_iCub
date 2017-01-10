#ifndef _ROLL_TEST_H_
#define ROLL_TEST_H_

#include <rtf/yarp/YarpTestCase.h>
#include <yarp/os/all.h>



class rollTest : public YarpTestCase
{
    yarp::os::Network yarp;
    yarp::os::RpcClient port;
    
    std::string look_downCMD;
    std::string rollCMD;
    std::string homeCMD;

public:
    rollTest();
    virtual ~rollTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
