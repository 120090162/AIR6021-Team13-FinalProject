/**
 * @file   mujoco_ros.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.1
 * @brief  A class for connecting a MuJoCo simulation with ROS2 communication.
 *
 * @details This class launches a MuJoCo simulation and provides communication channels in ROS2 for controlling it.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 *
 * @license GNU General Public License V3
 *
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */

#include <mujoco_ros2/mujoco_ros.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::MuJoCoROS(const std::string &xmlLocation) : Node("mujoco_node")
{
    // Declare & get parameters for this node
    std::string jointStateTopicName = this->declare_parameter<std::string>("joint_state_topic_name", "joint_state");
    std::string jointCommandTopicName = this->declare_parameter<std::string>("joint_command_topic_name", "joint_commands");
    std::string enableCommandTopicName = this->declare_parameter<std::string>("enable_command_topic_name", "enable_command");
    std::string controlMode = this->declare_parameter<std::string>("control_mode", "TORQUE");
    _simFrequency = this->declare_parameter<int>("simulation_frequency", 1000);
    int visualisationFrequency = this->declare_parameter<int>("visualisation_frequency", 20);
    mass = this->declare_parameter<double>("mass", 1.0);

    // Load the robot model
    char errorMessage[1000] = "Could not load model.";                     // We need this as an input argument.
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, errorMessage, 1000); // Try to load the model
    if (not _model)
        throw std::runtime_error("[ERROR] [MuJoCo NODE] Problem loading model: " + std::string(errorMessage));
    _model->opt.timestep = 1 / ((double)_simFrequency); // Match MuJoCo to node frequency

    // Resize arrays based on the number of joints in the model
    _jointState = mj_makeData(_model); // Initialize joint state
    _jointStateMessage.name.resize(_model->nu);
    _jointStateMessage.position.resize(_model->nu);
    _jointStateMessage.velocity.resize(_model->nu);
    _jointStateMessage.effort.resize(_model->nu);
    _torqueInput.resize(_model->nu, 0.0);

    // Record joint names
    for (int i = 0; i < _model->nu; ++i)
        _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);

    // Set the control mode
    if (controlMode == "POSITION")
        _controlMode = POSITION;
    else if (controlMode == "VELOCITY")
        _controlMode = VELOCITY;
    else if (controlMode == "TORQUE")
        _controlMode = TORQUE;
    else
    {
        throw std::invalid_argument("[ERROR] [MuJoCo NODE] Unknown control mode. "
                                    "Argument was '" +
                                    controlMode + "', but expected 'POSITION', 'VELOCITY', or 'TORQUE'.");
    }

    // Create timers
    _simTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / _simFrequency)),
                                        std::bind(&MuJoCoROS::update_simulation, this));

    _visTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / visualisationFrequency)),
                                        std::bind(&MuJoCoROS::update_visualization, this));

    // Create joint state publisher and joint command subscriber
    _jointCommandSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(jointCommandTopicName, 1, std::bind(&MuJoCoROS::joint_command_callback, this, std::placeholders::_1));

    _pickPlaceCommandSubscriber = this->create_subscription<std_msgs::msg::Bool>(enableCommandTopicName, 1, std::bind(&MuJoCoROS::pick_place_command_callback, this, std::placeholders::_1));

    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(jointStateTopicName, 1);

    // // Initialize Graphics Library FrameWork (GLFW)
    // if (not glfwInit())
    //     throw std::runtime_error("Failed to initialise Graphics Library Framework (GLFW).");

    // _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);

    // if (not _window)
    //     throw std::runtime_error("Failed to create Graphics Library Framework (GLFW) window.");

    // // Make the OpenGL context current
    // glfwMakeContextCurrent(_window);
    // glfwSwapInterval(1); // NOTE TO SELF: CHECK THIS ARGUMENT

    // // Initialize MuJoCo rendering context
    // mjv_defaultCamera(&_camera);
    // mjv_defaultOption(&_renderingOptions);
    // mjv_defaultPerturb(&_perturbation);
    // mjr_defaultContext(&_context);
    // mjv_makeScene(_model, &_scene, 1000);

    // // Declare & get parameters for camera, visualisation
    // _camera.azimuth = this->declare_parameter<double>("camera_azimuth", 135);
    // _camera.distance = this->declare_parameter<double>("camera_distance", 2.5);
    // _camera.elevation = this->declare_parameter<double>("camera_elevation", -35);
    // _camera.orthographic = this->declare_parameter<bool>("camera_orthographic", true);

    // auto focalPoint = this->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.5});

    // for (int i = 0; i < 3; ++i)
    //     _camera.lookat[i] = focalPoint[i];

    // // Create MuJoCo rendering context
    // glfwMakeContextCurrent(_window);
    // mjr_makeContext(_model, &_context, mjFONTSCALE_100);

    uiController = new UIctr(_model, _jointState); // UI control for Mujoco
    uiController->iniGLFW();
    uiController->enableTracking();                            // enable viewpoint tracking of the body 1 of the robot
    uiController->createWindow("MuJoCo Visualization", false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

    RCLCPP_INFO(this->get_logger(), "MuJoCo simulation initiated. "
                                    "Publishing the joint state to '%s' topic. "
                                    "Subscribing to joint %s commands via '%s' topic.",
                jointStateTopicName.c_str(), controlMode.c_str(), jointCommandTopicName.c_str());

    gripperBodyId = mj_name2id(_model, mjOBJ_BODY, "link6");

    // Check if gripper is found
    RCLCPP_INFO(this->get_logger(), "Gripper ID: %d", gripperBodyId);

    if (gripperBodyId == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not find gripper.");
        return;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_simulation()
{
    if (not _model and not _jointState)
    {
        RCLCPP_ERROR(this->get_logger(), "MuJoCo model or data is not initialized. How did that happen?");
        return;
    }

    if (_controlMode == TORQUE)
    {
        for (int i = 0; i < _model->nu; ++i)
        {
            _jointState->ctrl[i] = _torqueInput[i]                // Transfer torque input
                                   + _jointState->qfrc_bias[i]    // Compensate for gravity
                                   - 0.01 * _jointState->qvel[i]; // Add some damping

            _torqueInput[i] = 0.0; // Clear value
        }
    }

    mj_step(_model, _jointState); // Take a step in the simulation

    // Add joint state data to ROS2 message
    // for (int i = 0; i < _model->nq; ++i)
    // {
    //     _jointStateMessage.position[i] = _jointState->qacc[i];
    //     _jointStateMessage.velocity[i] = _jointState->qvel[i];
    //     _jointStateMessage.effort[i] = _jointState->actuator_force[i];
    // }
    for (int i = 0; i < _model->nu; ++i)
    {
        _jointStateMessage.position[i] = _jointState->sensordata[i];
        _jointStateMessage.velocity[i] = _jointState->sensordata[i + _model->nu];
        _jointStateMessage.effort[i] = _jointState->sensordata[i + 2 * _model->nu];
    }

    _jointStateMessage.header.stamp = this->get_clock()->now(); // Add current time stamp (for rqt)

    _jointStatePublisher->publish(_jointStateMessage); // Publish the joint state message
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != _model->nq)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received joint command with incorrect size.");
        return;
    }
    else
    {
        switch (_controlMode)
        {
        case POSITION:
        {
            for (int i = 0; i < _model->nq; ++i)
            {
                _jointState->ctrl[i] = msg->data[i]; // Assign control input directly
            }
            break;
        }
        case VELOCITY:
        {
            for (int i = 0; i < _model->nq; ++i)
            {
                _jointState->ctrl[i] += msg->data[i] / (double)_simFrequency; // Integrate velocity to position level
            }
            break;
        }
        case TORQUE:
        {
            for (int i = 0; i < _model->nq; ++i)
            {
                _torqueInput[i] = msg->data[i];
            }
            break;
        }
        default:
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Unknown control mode. Unable to set joint commands.");
            break;
        }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Handle pick/place commands                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::pick_place_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !_isPicking)
    {
        _isPicking = true;
        mjtNum force[3] = {0, 0, -9.8 * mass}; // 向下施加单位的力
        mjtNum torque[3] = {0, 0, 0};          // 无扭矩
        mjtNum point[3];                       // 作用点坐标
        mju_copy3(point, _jointState->xpos + 3 * gripperBodyId);
        mj_applyFT(_model, _jointState, force, torque, point, gripperBodyId, _jointState->qfrc_applied);
    }
    else if (!msg->data && _isPicking)
    {
        _isPicking = false;
        mju_zero(_jointState->qfrc_applied, _model->nv);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Update the 3D simulation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_visualization()
{
    // glfwMakeContextCurrent(_window); // Ensure OpenGL context is current

    // mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene); // Update 3D rendering

    // // Get framebuffer size
    // int width, height;
    // glfwGetFramebufferSize(_window, &width, &height);
    // mjrRect viewport = {0, 0, width, height};

    // mjr_render(viewport, &_scene, &_context); // Render scene

    // // Swap buffers and process events
    // glfwSwapBuffers(_window);
    // glfwPollEvents();
    uiController->updateScene();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Destructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::~MuJoCoROS()
{
    mj_deleteData(_jointState);
    mj_deleteModel(_model);
    // mjv_freeScene(&_scene);
    // mjr_freeContext(&_context);
    // glfwDestroyWindow(_window);
    // glfwTerminate();
    uiController->Close();
}