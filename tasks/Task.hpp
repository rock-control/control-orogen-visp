/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */


#ifndef VISP_TASK_TASK_HPP
#define VISP_TASK_TASK_HPP

#include "visp/TaskBase.hpp"
#include <visp/vpCameraParameters.h>
#include <visp/vpServo.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <apriltags/apriltagsTypes.hpp>


namespace visp{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
     task('custom_task_name','visp::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    typedef apriltags::VisualFeaturePoint Points;
    typedef std::vector<apriltags::VisualFeaturePoint> PointsVector;

    class Task : public TaskBase
    {
        friend class TaskBase;
        protected:

        visp::expectedInputs expected_inputs;
        visp::controllerState ctrl_state;
        base::LinearAngular6DCommand setpoint;
        base::Time start_time;

        vpCameraParameters cam_cal;
        vpServo task;
        vpPoint point[4];
        vpPoint P;

        vpHomogeneousMatrix cMb;  //body pose in the camera frame

        //features
        vpFeatureDepth depth;
        vpFeaturePoint p;
        vpFeaturePoint pd;
        vpFeatureThetaU tu;
        vpFeatureTranslation t;
        double Zd;

        //target parameters
        std::string desired_target;
        double rotation_around_z;
        std::vector<visp::targetObjectParameters> target_list;

        vpMatrix eJe;
        vpVelocityTwistMatrix cVb; //transform the velocities of the camera to the body frame


        /** Update the visual features of a hibrid visual controller.
         *  \param detected corners
         *  \param desired object pose in the camera frame
         */
        void updateFeaturesHVS(Points const &corners, vpHomogeneousMatrix const &cdMo);

        /** Update the desired pose according to a setpoint. The setpoint is the desired
         *  object pose in the vehicle's frame.
         *  \param Desired body position with resepct to the visual feature
         *  \return The desired pose of the object in the camera frame
         */
        vpHomogeneousMatrix updateDesiredPose(base::LinearAngular6DCommand setpoint);

        /** Set the controller gain
         *  \param static gain
         *  \param adaptive gains value
         *  \param set the adaptive gains if true. If false, use the static gain instead.
         */
        void setGain(double gain, visp::adaptiveGains const &adaptive_gains, bool use_adaptive);

        /** Write the computed velocities to the output port
         *  \param computed velocities
         */
        void writeVelocities(vpColVector const &v);

        /** Convert a visp homogeneus matrix in to a RigidBodyState
         *  \param Homogeneus matrix representing a pose
         *  \return Pose in the RigidBodyState type
         */
        base::samples::RigidBodyState convertToRbs(vpHomogeneousMatrix pose) const;

        /** Take the expected inputs and set the jacobian matrix for the controller
         *  \param Expected inputs for the setpoint with wrt body frame
         *  \return Jacobian matrix
         */
        vpMatrix jacobianFromExpectedInputs(const visp::expectedInputs &expected_inputs) const;


        /** Read the setpoint input port.
         *  \param Setpoint. Body position in the object frame
         *  \return True if there is any setpoint available, false if the input port was never
         *          writen
         */
        bool readSetpoint(base::LinearAngular6DCommand& setpoint);

        /** Read the input port and select among the VisualFeaturePoint vector the one who
         *  was the same identifier as "desired_target". Then update it features and return
         *  the corners of the desired target.
         *  \param vector of corners
         *  \param desired target identifier
         *  \return true if the corners could be retrieved, false if the desired target was
         *          not detected.
         */
        bool readCorners(Points& corners, std::string const &desired_target);

        /** Look for the parameters of a target in a given target list and update it.
         *  An exception is raised if there is no desired_target in the target list.
         *  \param target's name
         */
        void updateTargetParameters(std::string const &desired_target);

        /** Set the objetc size
         *  \param target width
         *  \param target height
         */
        void setObjectSize(double object_width, double object_height);

        /** Update the dynamic property "desired_target"
         *  \param string containing the desired target's identifier
         */
        virtual bool setDesired_target(std::string const &value);

        public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "visp::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
         needs_configuration
         ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

