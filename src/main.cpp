#include <string>
#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/math.h>
#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


YARP_DECLARE_DEVICES(icubmod)

#define MAX_TORSO_PITCH     30.0    // [deg]


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    int startup_context_gaze, startup_context_arm;

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
        Vector pos;
        igaze->triangulate3DPoint(cogL, cogR, pos);
        return pos;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        Vector x(4);
        x[0] = 0.0; 
        x[1] = 0.0; 
        x[2] = 0.0; 
        x[3] = 0.0;
        return x;
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        // we assume that only right hand is used
        // and we approach slightly on the right side of the target
        Vector pos(x);
        pos[0] += 0.05;
        pos[1] += 0.10;
        pos[2] += 0.05;
        iarm->goToPose(pos, o);
    }

    /***************************************************/
    void makeItRoll(const Vector &x, const Vector &o)
    {
        // FILL IN THE CODE
    }

    /***************************************************/
    void look_down()
    {
        // look at 0.5 meter in front of the robot
        Vector pos(3);
        pos[0] = -0.3;
        pos[1] =  0.0;
        pos[2] =  0.0;
        igaze->lookAtFixationPoint(pos);
    }

    /***************************************************/
    void roll(const Vector &cogL, const Vector &cogR)
    {
        printf("detected cogs = (%s) (%s)\n",
                cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());

        Vector x=retrieveTarget3D(cogL,cogR);
        printf("retrieved 3D point = (%s)\n",x.toString(3,3).c_str());

        fixate(x);
        printf("fixating at (%s)\n",x.toString(3,3).c_str());

        
        Vector o=computeHandOrientation();
        printf("computed orientation = (%s)\n",o.toString(3,3).c_str());

        approachTargetWithHand(x,o);
        printf("approached\n");

        /*
        makeItRoll(x,o);
        printf("roll!\n");
        */
    }

    /***************************************************/
    void home()
    {
        // these values should be loaded from a config file
        // for now we just put them in the code
        Vector x(3);
        x[0] = -0.3;
        x[1] =  0.4;
        x[2] =  0.2;
        iarm->goToPosition(x);

        // look down for the homing position
        look_down();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        std::string name = "MakeItRoll"; 

        // open a client interface to connect to the gaze server
        Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local",("/"+name+"/gaze").c_str());

        if (!drvGaze.open(optGaze))
            return false;

        // open the view
        drvGaze.view(igaze);
        if(!igaze)
            return false;

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the tracking mode, the neck limits and so on.
        igaze->storeContext(&startup_context_gaze);

        // set trajectory time:
        igaze->setNeckTrajTime(0.8);
        igaze->setEyesTrajTime(0.4);


        // open a client interface to connect to the arm cartesian control server
        Property optArm("(device cartesiancontrollerclient)");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local",("/"+name+"/cartesian/right_arm").c_str());

        if (!drvArm.open(optArm))
            return false;

        // open the view
        drvArm.view(iarm);

        // latch the controller context in order to preserve
        // it after closing the module
        iarm->storeContext(&startup_context_arm);

        // set trajectory time
        iarm->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        iarm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;

        // impose some restriction on the torso pitch
        int axis=0; // pitch joint
        double min, max;
        // we keep the lower limit
        iarm->getLimits(axis, &min, &max);
        iarm->setLimits(axis, min, MAX_TORSO_PITCH);
        iarm->setDOF(newDof,curDof);


        // Opening the required streaming and rpc ports
        imgLPortIn.open("/MakeItRoll/imgL:i");
        imgRPortIn.open("/MakeItRoll/imgR:i");

        imgLPortOut.open("/MakeItRoll/imgL:o");
        imgRPortOut.open("/MakeItRoll/imgR:o");

        rpcPort.open("/MakeItRoll/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        igaze->stopControl();
        iarm->stopControl();
        return true;
    }

    /***************************************************/
    bool close()
    {
        igaze->restoreContext(startup_context_gaze);
        drvGaze.close();

        iarm->restoreContext(startup_context_arm);
        drvArm.close();

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
        string cmd=command.get(0).asString().c_str();
        if (cmd=="look_down")
        {
            look_down();
            reply.addString("Yep! I'm looking down now!");
            return true;
        }
        else if (cmd=="roll")
        {
            bool allOk;
            Vector cogl, cogr;
            mutex.lock();
            allOk = okL && okR;
            cogl = cogL;
            cogr = cogR;
            mutex.unlock();

            if (allOk)
            {
                roll(cogl,cogr);
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
                reply.addString("I don't see any object!");
            return true;
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
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
    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}



