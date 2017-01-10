#include <rtf/dll/Plugin.h>
#include <stdio.h>

#include"roll_test.h"
#include <yarp/sig/Image.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;

// prepare the plugin
PREPARE_PLUGIN(rollTest)

/***********************************************************************************/
rollTest::rollTest() :
                       YarpTestCase("Roll Test")
{
}


/***********************************************************************************/
rollTest::~rollTest()
{
}


/***********************************************************************************/
bool rollTest::setup(Property &property)
{
    string portName = property.check("port",Value("/service")).asString();
    string testPortName = property.check("testPortName", Value("/test")).asString();
    
    look_downCMD = property.check("command_look_down",Value("look_down")).asString();
    rollCMD = property.check("command_roll",Value("roll")).asString();
    homeCMD = property.check("command_home",Value("home")).asString();
    
    RTF_TEST_REPORT(Asserter::format("Commands: \n-%s \n-%s \n-%s",look_downCMD.c_str(),rollCMD.c_str(),homeCMD.c_str()));
    
    RTF_TEST_REPORT(Asserter::format("Opening port with name %s",testPortName.c_str()));
    RTF_ASSERT_ERROR_IF(port.open(testPortName),"Unable to open test port");
    
    RTF_TEST_REPORT(Asserter::format("Connecting to %s",portName.c_str()));
    RTF_ASSERT_ERROR_IF(yarp.connect(testPortName,portName),"Unable to connect!");
    
    return true;
}

/***********************************************************************************/
void rollTest::tearDown()
{
    RTF_TEST_REPORT("Closing test port");
    port.close();
}

/***********************************************************************************/
void rollTest::run()
{
    Bottle cmd;
    
    cmd.addString(look_downCMD);
    RTF_TEST_REPORT(Asserter::format("Sending %s command",look_downCMD.c_str()));
    
    Bottle respond;
    
    
}
