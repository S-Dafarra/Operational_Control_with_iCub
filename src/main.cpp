#include <string>
#include <boost/concept_check.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
/***************************************************/
class CtrlModule: public RFModule
{
protected:
  
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    Vector            home_pose, home_rot, head_pose, head_rot, init_gaze;
        
         
    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    Port imgLPortOut,imgRPortOut;
    RpcServer rpcPort;    
    
    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
      Vector out;
      if(!(igaze->triangulate3DPoint(cogL,cogR,out)))
         out.clear();
      return out;
    }

    /***************************************************/
    bool fixate(const Vector &x)
    {
        bool done=false;
	double traj_time;
	
	igaze->getNeckTrajTime(&traj_time);
      
        igaze->bindNeckYaw(-20,20);
	igaze->bindNeckRoll(-20,20);
        igaze->lookAtFixationPoint(x);                 
        return igaze->waitMotionDone(0.1,20*traj_time); 
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
       Matrix R(3,3);
       
        R(0,0)= -1.0;  R(0,1)= 0.0;  R(0,2)= 0.0;    
	R(1,0)=  0.0;  R(1,1)= 0.0;  R(1,2)=-1.0;    
	R(2,0)=  0.0;  R(2,1)=-1.0;  R(2,2)= 0.0;     
      
      return yarp::math::dcm2axis(R);
    }

    /***************************************************/
    bool approachTargetWithHand(const Vector &x, const Vector &o)
    {
       Vector offset(3);
       Vector new_x(3);
       double time_now;
       double traj_time;
       
       offset = 0;
       
        offset(0) = -0.03;   
        offset(1) = 0.1;
	offset(2) = 0.01;
	new_x=x+offset;
	
	iarm->goToPose(new_x,o,1.0);
	
	bool done=false;
	
	iarm->getTrajTime(&traj_time);
	time_now=Time::now();
	
	
	while ((!done)&&((Time::now()-time_now)<(traj_time*10))) {
	iarm->checkMotionDone(&done);
	}
	
	if((Time::now()-time_now)>=(traj_time*10))      
	   return false;
	
	Time::delay(0.5);   // or any suitable delay
	return true;
    }

    /***************************************************/
    bool makeItRoll(const Vector &x, const Vector &o, double t)
    {
       Vector offset(3);
       Vector new_x(3);
       double time_now;
       int context;
       offset = 0;
        
        offset(0) = -0.03;      
        offset(1) = 0;
	offset(2) = 0.01;
	new_x=x+offset;
	
	iarm->storeContext(&context);
	iarm->goToPose(new_x,o,t);
	
	bool done=false;
	
	time_now=Time::now();
	
	while ((!done)&&((Time::now()-time_now)<(t*10))) {
	iarm->checkMotionDone(&done);
	}
	
	iarm->restoreContext(context);
	iarm->deleteContext(context);
	
	if((Time::now()-time_now)>=(t*10))     
	   return false;
	
	Time::delay(0.04);   // or any suitable delay
	return true;
    }

    /***************************************************/
    bool look_down()
    {
       Vector angles(3);
       int context;
       double traj_time;
       bool output;
	
       angles[0]=0.0;
       angles[1]=-40.0;
       angles[2]=0.0;
       
       igaze->storeContext(&context);
       igaze->getNeckTrajTime(&traj_time);
       igaze->bindNeckYaw(-0.1,0.1);
       igaze->bindNeckRoll(-0.1,0.1);
       igaze->lookAtAbsAngles(angles);
       output = igaze->waitMotionDone(0.1,20*traj_time); 
       igaze->restoreContext(context);
       if (!output)
	 igaze->stopControl();
       return output;
       
       
    }

    /***************************************************/
    bool roll(const Vector &cogL, const Vector &cogR,double t)
    {
        yInfo("detected cogs = (%s) (%s)",
              cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo("retrieved 3D point = (%s)",x.toString(3,3).c_str());

        if (fixate(x))
             yInfo("fixating at (%s)",x.toString(3,3).c_str());
	else{
	           yInfo("Timeout: fixate");
	           return false;
	     }	     

        Vector o=computeHandOrientation();
        yInfo("computed orientation = (%s)",o.toString(3,3).c_str());

        
        if (approachTargetWithHand(x,o)){
	    yInfo("approached");
	    if(makeItRoll(x,o,t))
               yInfo("roll with trajectory time = (%f)!",t);
	    else {
	           yInfo("Timeout: roll");
	           return false;
	         }
	    }
	else {
	           yInfo("Timeout: approach");
	           return false;
	         }
       return true;
    }

    /***************************************************/
    void home()
    {
      Vector angles(3);
      angles = 0.0;
      
      int contextArm, contextGaze, i;
       iarm->storeContext(&contextArm);
       
        //Set new DoF for context
      Vector dof, gains;
       iarm->getDOF(dof); dof=1.0;
       iarm->setDOF(dof,dof);
       
       //Set limits
       iarm->setLimits(0,-0.001,0.001);
       iarm->setLimits(1,-0.001,0.001);
       iarm->setLimits(2,-0.001,0.001);
       iarm->setLimits(dof.length()-1,-0.001,0.001);

       //Motion time = 1 seconds
        iarm->setTrajTime(1);
     
       //Go to pose
       iarm->goToPose(home_pose, home_rot);
       
       
       igaze->storeContext(&contextGaze);
       igaze->lookAtAbsAngles(init_gaze);
       igaze->blockNeckPitch(0);
       igaze->blockNeckRoll(0);
       igaze->blockNeckYaw(0);
             
       iarm->waitMotionDone();
       
       //Restore the previous context
       iarm->restoreContext(contextArm);
       iarm->deleteContext(contextArm);
       
       igaze->waitMotionDone();       
       igaze->stopControl();
       igaze->restoreContext(contextGaze);
       igaze->deleteContext(contextGaze);
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        
        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");
	
	imgLPortIn.setInputMode(true);
	imgRPortIn.setInputMode(true);
	
        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);
	
	Property optionGaze;
        optionGaze.put("device","gazecontrollerclient");
        optionGaze.put("remote","/iKinGazeCtrl");
        optionGaze.put("local","/client/gaze");
	
	drvGaze.open(optionGaze);
	
	Property optionCart;
        optionCart.put("device","cartesiancontrollerclient");
        optionCart.put("remote","/icubSim/cartesianController/right_arm");
        optionCart.put("local","/client/right_arm");
	
	drvArm.open(optionCart);

	
	if (drvArm.isValid()) {
	    drvArm.view(iarm);
	  }
	  else return false;
	  
	if (drvGaze.isValid()) {
	    drvGaze.view(igaze);
	  }
	  else return false;
        bool success;
	success = false;
	while(!success){
	success=iarm->getPose(home_pose,home_rot);
	}
	yInfo("home orientation = (%s)",home_rot.toString(3,3).c_str());
	yInfo("home position = (%s)",home_pose.toString(3,3).c_str());
	igaze->getAngles(init_gaze);
	
	Vector dof;
        iarm->getDOF(dof); dof=1.0;
        iarm->setDOF(dof,dof);
	
	return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            if(look_down())
            reply.addString("Yep! I'm looking down now!");
	    else reply.addString("Timeout!");
        }
        else if (cmd=="roll")
        {	
	    double t;
	    if(command.get(1).isDouble()||command.get(1).isInt()) {t=command.get(1).asDouble();}
	    else t=0.5;
	      
            ImageOf<PixelRgb> *imgL=imgLPortIn.read();
            ImageOf<PixelRgb> *imgR=imgRPortIn.read();
            
            if (getCOG(*imgL, cogL)&&getCOG(*imgR, cogR))
            {
                if(roll(cogL,cogR,t))
                   reply.addString("Yeah! I've made it roll like a charm!");
		else reply.addString("Timeout!");
            }
            else
                reply.addString("I don't see any object!");
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
        }
        else
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images("right")
    }

    /***************************************************/
    bool updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.write(*imgL);
        imgRPortOut.write(*imgR);

        return true;
    }
};


/***************************************************/
int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
        return 1;

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}

