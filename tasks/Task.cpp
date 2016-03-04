/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include "Task.hpp"

using namespace visp;

Task::Task(std::string const& name)
    : TaskBase(name), tu(vpFeatureThetaU::cdRc), t(vpFeatureTranslation::cdMc)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), tu(vpFeatureThetaU::cdRc), t(vpFeatureTranslation::cdMc)
{
}

Task::~Task()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // Initialize camera parameters.
    // The library desconsider the tangencial values, that's why only the
    // radial values are informed. 
    frame_helper::CameraCalibration cal = _camera_parameters.get();
    cam_cal = vpCameraParameters(cal.fx, cal.fy,cal.cx,cal.cy);

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    double L = _marker_size.get()/2; // 2L is the square size
    point[0].setWorldCoordinates(-L,  L, 0);
    point[1].setWorldCoordinates( L,  L, 0);
    point[2].setWorldCoordinates( L, -L, 0);
    point[3].setWorldCoordinates(-L, -L, 0);

    //Center of the target (reference point)
    P.setWorldCoordinates(0, 0, 0);

    // Define the task
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    switch(_architecture.get().mode)
    {
        case IBVS:
            task.addFeature(p,pd) ;
            break;
        case PBVS:
            task.addFeature(t);
            task.addFeature(tu);
            break;
        case HVS:
            task.addFeature(p,pd) ;
            task.addFeature(depth) ;
            task.addFeature(tu) ;
    }

    // Set the Jacobian (expressed in the end-effector frame)
    eJe.eye(6,6);
    //eJe[3][3] = 0;
    //eJe[5][3] = 1;
    task.set_eJe(eJe);

    return true;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    std::vector<base::Vector2d> corners;
    _marker_corners.read(corners);

    if(state() == RUNNING)
    {
        state(WAITING_FIRST_MEASUREMENTS);
    }

    if (corners.size() > 0 && state() == WAITING_FIRST_MEASUREMENTS)
    {
        // Sets the desired position of the visual feature (reference point)
        updateDesiredPose();
        updateFeatures(corners);
        state(CONTROLLING);
    }
    else if (corners.size() > 0 && state() == CONTROLLING)
    {
        updateDesiredPose();
        updateFeatures(corners);

        // Update this jacobian in the task structure. It will be used to compute
        // the velocity skew (as an articular velocity)
        // qdot = -lambda * L^+ * cVe * eJe * (s-s*)
        task.set_eJe(eJe) ;

        // Set the gain
        setGain();

        // Compute the visual servoing skew vector
        vpColVector v;
        task.set_cVe(cVe);
        v = task.computeControlLaw() ;

        writeVelocities(v);
        corners.clear();
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    task.kill();
}

void Task::updateFeatures(std::vector<base::Vector2d> corners)
{
    //  Compute the initial pose
    pose.clearPoint();

    for (int i=0 ; i < 4 ; i++)
    {
        base::Vector2d aux;
        aux = corners[i];
        //vpImagePoint is instanciated using (v,u) instead of (u,v)
        vpImagePoint a(aux[1],aux[0]);
        double x,y;
        vpPixelMeterConversion::convertPoint(cam_cal,a,x,y);
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]) ;
    }

    pose.computePose(vpPose::LAGRANGE, cMo);
    pose.computePose(vpPose::VIRTUAL_VS, cMo);

    // Update Feature 1 - p
    // Sets the current position of the visual feature
    P.track(cMo);
    vpFeatureBuilder::create(p, P);

    //////////////////////////////
    // Update Feature 2 - depth //
    //////////////////////////////

    //Takes the features, calculate the intersection of the diagonals of the square and
    //given the camera parameters, retrieve the coordinates in meters.
    vpImagePoint refP;
    //double x,y;
    vpImagePoint cog[4];

    for (int i=0; i< 4; i++)
    {
        base::Vector2d aux;
        aux = corners[i];
        //according to vpImagePoint documentation (i,j) is equivalent to (v,u)
        cog[i].set_ij(aux[1], aux[0]);
    }

    // Compute the coordinates of the 2D reference point from the
    // intersection of the diagonals of the square
    double a1 = (cog[0].get_i()- cog[2].get_i()) /
        (cog[0].get_j()- cog[2].get_j());
    double a2 = (cog[1].get_i()- cog[3].get_i()) /
        (cog[1].get_j()- cog[3].get_j());
    double b1 = cog[0].get_i() - a1*cog[0].get_j();
    double b2 = cog[1].get_i() - a2*cog[1].get_j();

    refP.set_j( (b1-b2) / (a2-a1) );
    refP.set_i( a1*refP.get_j() + b1 );
    double x,y;
    vpPixelMeterConversion::convertPoint(cam_cal, refP, x, y);

    // Update the 2D ref point visual feature
    double Z;
    Z = P.get_Z();
    p.set_xyZ(x, y, Z);
    depth.buildFrom(P.get_x(), P.get_y(), Z, log(Z/Zd));

    //////////////////////////////
    // Update Feature 3 - t, tu //
    //////////////////////////////
    cdMc = cdMo*cMo.inverse();
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc) ;

}

void Task::updateDesiredPose()
{
    // Initialise a desired pose to compute s*, the desired 2D point features
    base::LinearAngular6DCommand setpoint;
    if (_cmd_in.read(setpoint) == RTT::NewData)
    {
        vpTranslationVector cdto(setpoint.linear[0], setpoint.linear[1], setpoint.linear[2]);
        vpRxyzVector cdro(setpoint.angular[0], setpoint.angular[1], setpoint.angular[2]);

        //Build the rotation and homogeneus matrix according to desired pose
        vpRotationMatrix cdRo(cdro); // Build the rotation matrix
        cdMo.buildFrom(cdto, cdRo); // Build the homogeneous matrix

        P.track(cdMo);
        vpFeatureBuilder::create(pd, P);
        Zd = P.get_Z();
    }
    else
    {
        exception(MISSING_SETPOINT);
        throw std::runtime_error("There is no setpoint available");
    }

}

void Task::setGain()
{
    if (_use_adaptive_gain.get())
    {
        vpAdaptiveGain  lambda;
        lambda.initStandard(_adaptive_gains.get().gain_at_zero,
                _adaptive_gains.get().gain_at_infinity,
                _adaptive_gains.get().slope_at_zero);

        task.setLambda(lambda) ;
    }
    else
    {
        task.setLambda(_gain.get());
    }
}

void Task::writeVelocities(vpColVector v)
{
    base::LinearAngular6DCommand vel;
    vel.linear[0] = v[0];
    vel.linear[1] = v[1];
    vel.linear[2] = v[2];
    vel.angular[0] = v[3];
    vel.angular[1] = v[4];
    vel.angular[2] = v[5];
    vel.time = base::Time::now();

    _cmd_out.write(vel);
}
