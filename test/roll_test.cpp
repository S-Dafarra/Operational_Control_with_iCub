#include <rtf/dll/Plugin.h>
#include <stdio.h>
#include<cmath>

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
    RTF_ASSERT_ERROR_IF(yarp.connect(testPortName,portName),"Unable to connect to module!");
    
    RTF_TEST_REPORT(Asserter::format("Opening reading ports /head, /torso, /right_arm, /ball",testPortName.c_str()));
    RTF_ASSERT_ERROR_IF(head.open("/head"),"Unable to open /head port");
    RTF_ASSERT_ERROR_IF(torso.open("/torso"),"Unable to open /torso port");
    RTF_ASSERT_ERROR_IF(arm.open("/arm"),"Unable to open /arm port");
    RTF_ASSERT_ERROR_IF(ball.open("/ball"),"Unable to open /ball port");
    
    RTF_TEST_REPORT(Asserter::format("Connecting to simulator ports"));
    RTF_ASSERT_ERROR_IF(yarp.connect("/icubSim/head/state:o","/head"),"Unable to connect to head!");
    RTF_ASSERT_ERROR_IF(yarp.connect("/icubSim/torso/state:o","/torso"),"Unable to connect to torso!");
    RTF_ASSERT_ERROR_IF(yarp.connect("/icubSim/right_arm/state:o","/arm"),"Unable to connect to arm!");
    RTF_ASSERT_ERROR_IF(yarp.connect("/ball","/icubSim/world"),"Unable to connect to world!");
    
    head.setStrict(false);
    torso.setStrict(false);
    arm.setStrict(false);
    
    
    return true;
}

/***********************************************************************************/
void rollTest::tearDown()
{
    RTF_TEST_REPORT("Closing test ports");
    port.close();
    head.close();
    torso.close();
    arm.close();
    ball.close();
}

/***********************************************************************************/
void rollTest::run()
{
    Bottle *reader;
    Bottle head_init, torso_init, arm_init, ball_init;
    Bottle cmd, respond;
    bool passed = false;
    double t0;
    double delta_ball;
    int i=0;
    
    reader = head.read();
    RTF_TEST_REPORT(Asserter::format("Retrieving head initial position: %s",reader->toString().c_str()));
    head_init = *reader;
    RTF_TEST_REPORT(Asserter::format("Retrieving head initial position: %s",head_init.toString().c_str()));
    
    reader->clear();
    reader = torso.read();
    torso_init = *reader;
    RTF_TEST_REPORT(Asserter::format("Retrieving torso initial position: %s",torso_init.toString().c_str()));
    
    reader->clear();
    reader = arm.read();
    arm_init = *reader;
    RTF_TEST_REPORT(Asserter::format("Retrieving arm initial position: %s",arm_init.toString().c_str()));
    
    cmd.clear();
    respond.clear();
    cmd.addString("world");
    cmd.addString("get");
    cmd.addString("ball");
    RTF_TEST_REPORT(Asserter::format("Sending ''world get ball'' command"));
    RTF_ASSERT_ERROR_IF(ball.write(cmd,respond),"Unable to communicate to simulator");
    ball_init = respond;
    RTF_TEST_REPORT(Asserter::format("Retrieving ball initial position: %s",ball_init.toString().c_str()));
    
    cmd.clear();
    cmd.addString(look_downCMD);
    RTF_TEST_REPORT(Asserter::format("Sending %s command",look_downCMD.c_str()));
    RTF_ASSERT_ERROR_IF(port.write(cmd),"Unable to communicate to module");
    
    t0=Time::now();
    passed = false;
    while ((Time::now()-t0<5.0)&&(!passed)){
        if(head.read()->get(0).asDouble() < -25.0)
            passed = true;
        Time::delay(0.1);
    }
    RTF_TEST_CHECK(passed,"Neck pitch lower than -25 degrees");
    
    cmd.clear();
    cmd.addString(rollCMD);
    RTF_TEST_REPORT(Asserter::format("Sending %s command",rollCMD.c_str()));
    RTF_ASSERT_ERROR_IF(port.write(cmd),"Unable to communicate to module");
    
    t0=Time::now();
    passed=false;
    while ((Time::now()-t0<20.0)&&(!passed)){
       cmd.clear();
       respond.clear();
       cmd.addString("world");
       cmd.addString("get");
       cmd.addString("ball");
       RTF_ASSERT_ERROR_IF(ball.write(cmd,respond),"Unable to communicate to simulator");
       for(i=0;i<3;i++){
           delta_ball = respond.get(i).asDouble() - ball_init.get(i).asDouble();
           if (abs(delta_ball) > 0.1)
               passed = true;
       }
       Time::delay(0.1);
    }
    RTF_TEST_CHECK(passed,"Ball moved of at least 0.1m in at least one direction");
    
    cmd.clear();
    cmd.addString(homeCMD);
    respond.clear();
    RTF_TEST_REPORT(Asserter::format("Sending %s command",homeCMD.c_str()));
    RTF_ASSERT_ERROR_IF(port.write(cmd,respond),"Unable to communicate to module");
    
    RTF_TEST_REPORT(Asserter::format("Waiting 3 seconds to let the robot go home"));
    Time::delay(3);
    
    i=1;
    bool completed = false;
    int head_count=0,torso_count=0,arm_count=0;
    Bottle *read_head, *read_torso, *read_arm;
    
    passed=true;
    completed = false;
    head_count=0;
    torso_count=0;
    arm_count=0;


    while((i<15) && (!completed)){
        
        read_head=head.read();
        
        //RTF_TEST_REPORT(Asserter::format("Reading head: %s",read_head->toString().c_str()));

        read_torso=torso.read();
        //RTF_TEST_REPORT(Asserter::format("Reading torso: %s",read_torso->toString().c_str()));

        read_arm=arm.read();
        //RTF_TEST_REPORT(Asserter::format("Reading arm: %s",read_arm->toString().c_str()));
        
        passed=true;
        head_count=0;
        torso_count=0;
        arm_count=0;

        while (passed && !completed){
            completed = true;
            if(head_count < head_init.size()){
                passed = passed && (abs(read_head->get(head_count).asDouble() - head_init.get(head_count).asDouble()) < double(i));
                RTF_TEST_REPORT(Asserter::format("Head %d check %d: %0.2f \n",head_count,passed,abs(read_head->get(head_count).asDouble() - head_init.get(head_count).asDouble())));
                head_count++;
                completed = false;
            }
            if(torso_count < torso_init.size()){
                passed = passed && (abs(read_torso->get(torso_count).asDouble() - torso_init.get(torso_count).asDouble()) < double(i));
                RTF_TEST_REPORT(Asserter::format("Torso %d check %d: %0.2f \n",torso_count,passed,abs(read_torso->get(torso_count).asDouble() - torso_init.get(torso_count).asDouble())));
                torso_count++;
                completed = false;
            }
            if(arm_count < min(arm_init.size(),7)){
                passed = passed && (abs(read_arm->get(arm_count).asDouble() - arm_init.get(arm_count).asDouble()) < double(i));
                RTF_TEST_REPORT(Asserter::format("Arm %d check %d: %0.2f \n",arm_count,passed,abs(read_arm->get(arm_count).asDouble() - arm_init.get(arm_count).asDouble())));
                arm_count++;
                completed = false;
            }
        }
        RTF_TEST_CHECK(completed,Asserter::format("All joints back to home position with %d degree accuracy",i));
        i+=2;
    }
}
