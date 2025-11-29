/**
 * @file   mujoco_ros.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.1
 * @brief  A class for connecting a MuJoCo simulation with ROS2 communication.
 *
 * @details This class launches a MuJoCo simulation and provides communication
 * channels in ROS2 for controlling it.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 *
 * @license GNU General Public License V3
 *
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */

// #include "mjmodel.h"
// #include "mjtnum.h"
// #include "mujoco.h"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco_ros2/mujoco_ros.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Constructor //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::MuJoCoROS(const std::string &xmlLocation) : Node("mujoco_node") {
  // Declare & get parameters for this node
  std::string jointStateTopicName = this->declare_parameter<std::string>(
      "joint_state_topic_name", "joint_state");
  std::string jointCommandTopicName = this->declare_parameter<std::string>(
      "joint_command_topic_name", "joint_commands");
  std::string controlMode =
      this->declare_parameter<std::string>("control_mode", "TORQUE");
  _simFrequency = this->declare_parameter<int>("simulation_frequency", 1000);
  int visualisationFrequency =
      this->declare_parameter<int>("visualisation_frequency", 20);

  // Load the robot model
  char errorMessage[1000] =
      "Could not load model."; // We need this as an input argument.
  _model = mj_loadXML(xmlLocation.c_str(), nullptr, errorMessage,
                      1000); // Try to load the model
  if (not _model)
    throw std::runtime_error("[ERROR] [MuJoCo NODE] Problem loading model: " +
                             std::string(errorMessage));
  _model->opt.timestep =
      1 / ((double)_simFrequency); // Match MuJoCo to node frequency

  // Resize arrays based on the number of joints in the model
  _jointState = mj_makeData(_model); // Initialize joint state
  _jointStateMessage.name.resize(_model->nq);
  _jointStateMessage.position.resize(_model->nq);
  _jointStateMessage.velocity.resize(_model->nq);
  _jointStateMessage.effort.resize(_model->nq);
  _torqueInput.resize(_model->nq, 0.0);

  // Record joint names
  for (int i = 0; i < _model->nq; ++i)
    _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);

  // Set the control mode
  if (controlMode == "POSITION")
    _controlMode = POSITION;
  else if (controlMode == "VELOCITY")
    _controlMode = VELOCITY;
  else if (controlMode == "TORQUE")
    _controlMode = TORQUE;
  else {
    throw std::invalid_argument(
        "[ERROR] [MuJoCo NODE] Unknown control mode. "
        "Argument was '" +
        controlMode + "', but expected 'POSITION', 'VELOCITY', or 'TORQUE'.");
  }

  // Create timers
  _simTimer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / _simFrequency)),
      std::bind(&MuJoCoROS::update_simulation, this));

  _visTimer = this->create_wall_timer(
      std::chrono::milliseconds(
          static_cast<int>(1000 / visualisationFrequency)),
      std::bind(&MuJoCoROS::update_visualization, this));

  // Create joint state publisher and joint command subscriber
  _jointCommandSubscriber =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
          jointCommandTopicName, 1,
          std::bind(&MuJoCoROS::joint_command_callback, this,
                    std::placeholders::_1));

  _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(
      jointStateTopicName, 1);

  // Initialize Graphics Library FrameWork (GLFW)
  if (not glfwInit())
    throw std::runtime_error(
        "Failed to initialise Graphics Library Framework (GLFW).");

  _window =
      glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);

  if (not _window)
    throw std::runtime_error(
        "Failed to create Graphics Library Framework (GLFW) window.");

  // Make the OpenGL context current
  glfwMakeContextCurrent(_window);
  glfwSwapInterval(1); // NOTE TO SELF: CHECK THIS ARGUMENT

  glfwSetWindowUserPointer(_window,
                           this); // so we can refer to the instance "MuJoCoROS"
                                  // later and access its members
  glfwSetKeyCallback(_window,
                     delete_trajectory); // handle any key presses; right now
                                         // just delete trajectory path
  glfwSetCursorPosCallback(_window, cursor_position_callback);
  glfwSetMouseButtonCallback(_window, mouse_callback);

  // Initialize MuJoCo rendering context
  mjv_defaultCamera(&_camera);
  mjv_defaultOption(&_renderingOptions);
  mjv_defaultPerturb(&_perturbation);
  mjr_defaultContext(&_context);
  mjv_makeScene(_model, &_scene, 1000);

  // Declare & get parameters for camera, visualisation
  _camera.azimuth = this->declare_parameter<double>("camera_azimuth", 150);
  _camera.distance = this->declare_parameter<double>("camera_distance", 3.3);
  _camera.elevation = this->declare_parameter<double>("camera_elevation", -35);
  _camera.orthographic =
      this->declare_parameter<bool>("camera_orthographic", true);

  auto focalPoint = this->declare_parameter<std::vector<double>>(
      "camera_focal_point", {0.0, 0.0, 0.5});

  for (int i = 0; i < 3; ++i)
    _camera.lookat[i] = focalPoint[i];

  // Create MuJoCo rendering context
  glfwMakeContextCurrent(_window);
  mjr_makeContext(_model, &_context, mjFONTSCALE_100);

  RCLCPP_INFO(this->get_logger(),
              "MuJoCo simulation initiated. "
              "Publishing the joint state to '%s' topic. "
              "Subscribing to joint %s commands via '%s' topic.",
              jointStateTopicName.c_str(), controlMode.c_str(),
              jointCommandTopicName.c_str());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Update the simulation //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_simulation() {
  if (not _model and not _jointState) {
    RCLCPP_ERROR(
        this->get_logger(),
        "MuJoCo model or data is not initialized. How did that happen?");
    return;
  }

  if (_controlMode == TORQUE) {
    for (int i = 0; i < _model->nq; ++i) {
      _jointState->ctrl[i] = _torqueInput[i] // Transfer torque input
                             +
                             _jointState->qfrc_bias[i] // Compensate for gravity
                             - 0.01 * _jointState->qvel[i]; // Add some damping

      _torqueInput[i] = 0.0; // Clear value
    }
  }

  mj_step(_model, _jointState); // Take a step in the simulation

  // Add joint state data to ROS2 message
  for (int i = 0; i < _model->nq; ++i) {
    _jointStateMessage.position[i] = _jointState->qpos[i];
    _jointStateMessage.velocity[i] = _jointState->qvel[i];
    _jointStateMessage.effort[i] = _jointState->actuator_force[i];
  }

  _jointStateMessage.header.stamp =
      this->get_clock()->now(); // Add current time stamp (for rqt)

  _jointStatePublisher->publish(
      _jointStateMessage); // Publish the joint state message
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Handle joint commands //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::joint_command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != _model->nq) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received joint command with incorrect size.");
    return;
  } else {
    switch (_controlMode) {
    case POSITION: {
      for (int i = 0; i < _model->nq; ++i) {
        _jointState->ctrl[i] = msg->data[i]; // Assign control input directly
      }
      break;
    }
    case VELOCITY: {
      for (int i = 0; i < _model->nq; ++i) {
        _jointState->ctrl[i] +=
            msg->data[i] /
            (double)_simFrequency; // Integrate velocity to position level
      }
      break;
    }
    case TORQUE: {
      for (int i = 0; i < _model->nq; ++i) {
        _torqueInput[i] = msg->data[i];
      }
      break;
    }
    default: {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Unknown control mode. Unable to set joint commands.");
      break;
    }
    }
  }
}

void MuJoCoROS::draw_trajectory_path() {
  float rgba[4] = {1.0f, 0.0f, 0.0f, 1.0f}; // red
  mjtNum sphsize[3] = {0.01, 0.01, 0.01};
  mjtNum myrot3x3[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};

  // get the position of the end effector
  int endEffector_id = mj_name2id(_model, mjOBJ_BODY, "wrist_3_link");
  if (endEffector_id == -1)
    throw std::runtime_error("End-effector body not found in model");
  mjtNum endEffectorPos[3];
  mju_copy3(endEffectorPos, _jointState->xpos + 3 * endEffector_id);

  // lambda function that checks if two points are approximately equal based on
  // eps
  auto almost_equal = [](const std::array<mjtNum, 3> &a,
                         const std::array<mjtNum, 3> &b, mjtNum eps = 1e-2) {
    for (int i = 0; i < 3; ++i)
      if (mju_abs(a[i] - b[i]) > eps)
        return false;
    return true;
  };

  // adds the point only if its not the same as the last one
  // it also removes the earlier points of more than 200 points were drawn (so a
  // buffer overflow doesn't occur in the scene.geom buffer)
  std::array<mjtNum, 3> endEffectorPosArray = {
      endEffectorPos[0], endEffectorPos[1], endEffectorPos[2]};
  if (trajectory.empty() ||
      !almost_equal(trajectory.back(), endEffectorPosArray)) {
    if (trajectory.size() > trajectoryLength) {
      trajectory.erase(trajectory.begin());
    }

    if (drawTrajectory)
      trajectory.push_back(std::array<mjtNum, 3>{
          endEffectorPos[0], endEffectorPos[1], endEffectorPos[2]});
  }

  // still track any position changes that occur, but do not draw them
  if (hideTrajectory)
    return;
  for (size_t i = 0; i < trajectory.size(); i++) {
    mjtNum currentPos[3] = {trajectory[i][0], trajectory[i][1],
                            trajectory[i][2]};
    mjvGeom *sphereGeom = &_scene.geoms[_scene.ngeom++];
    // below isn't really needed bas kol el references 7atoha fa just incase
    sphereGeom->objtype = mjOBJ_UNKNOWN;
    sphereGeom->objid = -1;
    sphereGeom->segid = _scene.ngeom;
    sphereGeom->category = mjCAT_DECOR;

    // adds the sphereGeom to the scene :)
    mjv_initGeom(sphereGeom, mjGEOM_SPHERE, sphsize, currentPos, myrot3x3,
                 rgba);
    // adds the line to the scene :)
    if (i > 0) {
      mjtNum previousPos[3] = {trajectory[i - 1][0], trajectory[i - 1][1],
                               trajectory[i - 1][2]};
      mjtNum dx = currentPos[0] - previousPos[0];
      mjtNum dy = currentPos[1] - previousPos[1];
      mjtNum dz = currentPos[2] - previousPos[2];
      mjtNum distance = sqrt(dx * dx + dy * dy + dz * dz);
      if (distance <= 1.0) {
        mjvGeom *lineGeom = &_scene.geoms[_scene.ngeom++];
        lineGeom->objtype = mjOBJ_UNKNOWN;
        lineGeom->objid = -1;
        lineGeom->segid = _scene.ngeom;
        lineGeom->category = mjCAT_DECOR;
        mjv_initGeom(lineGeom, mjGEOM_LINE, sphsize, currentPos, myrot3x3,
                     rgba);
        mjv_connector(lineGeom, mjGEOM_LINE, 2, currentPos, previousPos);
      }
    }
  }
}

// does also other keybinds; mekasel a8yar el esm
void MuJoCoROS::delete_trajectory(GLFWwindow *window, int key, int scancode,
                                  int action, int mods) {
  if (action != GLFW_PRESS)
    return;

  MuJoCoROS *instance =
      static_cast<MuJoCoROS *>(glfwGetWindowUserPointer(window));

  if (key == GLFW_KEY_D) {
    std::cout << "Trajectory path deleted!" << std::endl;
    instance->trajectory.clear();
  }
  // hideTrajectory controls whether the trajectory is drawn on screen or not
  // the end effector is still tracked so if the trajectoy is unhidden, the
  // entire path is still shown
  if (key == GLFW_KEY_H) {
    if (instance->hideTrajectory) {
      instance->hideTrajectory = false;
      std::cout << "Trajectory path visible!" << std::endl;
    } else {
      instance->hideTrajectory = true;
      std::cout << "Trajectory path hidden!" << std::endl;
    }
  }
  // drawTrajectory allows new points to be drawn while the endeffector is
  // moving, if it is false, no new points will be drawn, however the past
  // points will still be shown
  if (key == GLFW_KEY_T) {
    if (instance->drawTrajectory) {
      instance->drawTrajectory = false;
      std::cout << "Stopped drawing trajectory path!" << std::endl;
    } else {
      instance->drawTrajectory = true;
      std::cout << "Drawing trajectory path!" << std::endl;
    }
  }

  if (key == GLFW_KEY_1) {
    instance->moveBoxToMiddle = true;
  }
  if (key == GLFW_KEY_2) {
    instance->moveBoxToEnd = true;
  }
}

void MuJoCoROS::mouse_callback(GLFWwindow *window, int button, int action,
                               int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    MuJoCoROS *instance =
        static_cast<MuJoCoROS *>(glfwGetWindowUserPointer(window));

    // set the current position of the x and y so that dragging doesnt cause a
    // sudden jump
    glfwGetCursorPos(window, &instance->last_x, &instance->last_y);
    if (action == GLFW_PRESS) {
      instance->dragging = true;

    } else if (action == GLFW_RELEASE)
      instance->dragging = false;
  }
}
void MuJoCoROS::cursor_position_callback(GLFWwindow *window, double x,
                                         double y) {
  MuJoCoROS *instance =
      static_cast<MuJoCoROS *>(glfwGetWindowUserPointer(window));
  if (!instance->dragging)
    return;

  double dx = x - instance->last_x;
  double dy = y - instance->last_y;

  const double sensitivity = 0.3;

  instance->_camera.azimuth -= dx * sensitivity;
  instance->_camera.elevation -= dy * sensitivity;
  // dont let the camera go upside down
  if (instance->_camera.elevation > 89)
    instance->_camera.elevation = 89;
  if (instance->_camera.elevation < -89)
    instance->_camera.elevation = -89;

  instance->last_x = x;
  instance->last_y = y;
}

void MuJoCoROS::move_box() {
  double speed = 0.009;
  double middleTarget = 0.45;
  double endTarget = 1.0;
  if (moveBoxToMiddle) {
    // kol da just to get the position of the box :*)
    int box_body_id = mj_name2id(_model, mjOBJ_BODY, "box");
    int joint_id = _model->body_jntadr[box_body_id];
    int qpos_index = _model->jnt_qposadr[joint_id];
    double &pos = _jointState->qpos[qpos_index];
    if (pos < middleTarget) {
      pos += speed;
    } else {
      moveBoxToMiddle = false;
    }
  }
  if (moveBoxToEnd) {
    // kol da just to get the position of the box :*)
    int box_body_id = mj_name2id(_model, mjOBJ_BODY, "box");
    int joint_id = _model->body_jntadr[box_body_id];
    int qpos_index = _model->jnt_qposadr[joint_id];
    double &pos = _jointState->qpos[qpos_index];
    if (pos < endTarget) {
      pos += speed;
    } else {
      moveBoxToEnd = false;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Update the 3D simulation //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_visualization() {
  glfwMakeContextCurrent(_window); // Ensure OpenGL context is current

  if (moveBoxToMiddle || moveBoxToEnd) {
    move_box();
  }

  mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera,
                  mjCAT_ALL, &_scene); // Update 3D rendering
  draw_trajectory_path();

  // Get framebuffer size
  int width, height;
  glfwGetFramebufferSize(_window, &width, &height);
  mjrRect viewport = {0, 0, width, height};

  mjr_render(viewport, &_scene, &_context); // Render scene

  // Swap buffers and process events
  glfwSwapBuffers(_window);
  glfwPollEvents();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Destructor //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::~MuJoCoROS() {
  mj_deleteData(_jointState);
  mj_deleteModel(_model);
  mjv_freeScene(&_scene);
  mjr_freeContext(&_context);
  glfwDestroyWindow(_window);
  glfwTerminate();
}
